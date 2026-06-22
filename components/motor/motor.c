/*
 * 电机驱动实现
 * 硬件：TB6612FNG + 520编码电机
 *
 * TB6612FNG 驱动逻辑：
 *   IN1=0, IN2=0: 停止
 *   IN1=0, IN2=1: 逆时针 (CCW)
 *   IN1=1, IN2=0: 顺时针 (CW)
 *   IN1=1, IN2=1: 制动
 *
 * PWM: 通过LEDC生成PWM信号控制转速
 * 编码器: GPIO中断计数 + 定时器采样计算RPM
 */

#include "motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <string.h>

static const char *TAG = "MOTOR";

// LEDC配置
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT  // 10位分辨率 (0-1023)
#define LEDC_FREQUENCY      1000               // PWM频率 1KHz

// 电机PWM通道映射
#define MOTOR1_PWM_CHANNEL  LEDC_CHANNEL_0
#define MOTOR2_PWM_CHANNEL  LEDC_CHANNEL_1
#define MOTOR3_PWM_CHANNEL  LEDC_CHANNEL_2

// 编码器参数
#define ENCODER_PPR         330     // 编码器线数（AB相，单边计数每转330个脉冲）
#define ENCODER_MULTIPLIER  4       // 四倍频（A+B双边沿触发）
#define GEAR_RATIO          30      // 减速比
// 输出轴每转脉冲数 = 330 * 4 * 30 = 39600
#define PULSES_PER_REV      (ENCODER_PPR * ENCODER_MULTIPLIER * GEAR_RATIO)

// 采样定时器配置
#define SAMPLE_PERIOD_MS    100     // 采样周期 100ms

// 电机引脚配置表（根据引脚接线.md）
static const motor_config_t motor_configs[MOTOR_MAX] = {
    // 电机1: PWMA=1, AIN1=2, AIN2=4, 编码器A=5, B=6
    {
        .pwm_pin = 1,
        .in1_pin = 2,
        .in2_pin = 4,
        .enc_a_pin = 5,
        .enc_b_pin = 6
    },
    // 电机2: PWMB=7, BIN1=15, BIN2=16, 编码器A=17, B=18
    {
        .pwm_pin = 7,
        .in1_pin = 15,
        .in2_pin = 16,
        .enc_a_pin = 17,
        .enc_b_pin = 18
    },
    // 电机3: PWMA=8, AIN1=38, AIN2=39, 编码器A=40, B=41
    {
        .pwm_pin = 8,
        .in1_pin = 38,
        .in2_pin = 39,
        .enc_a_pin = 40,
        .enc_b_pin = 41
    }
};

// 编码器计数器（使用volatile因为会在中断中修改）
static volatile int32_t encoder_counts[MOTOR_MAX] = {0};

// 用于RPM计算的脉冲差值（在ISR中更新，在任务上下文中读取）
static volatile int32_t pulse_deltas[MOTOR_MAX] = {0};

// 上次采样时的编码器计数值
static int32_t last_encoder_counts[MOTOR_MAX] = {0};

// 当前RPM值
static float current_rpm[MOTOR_MAX] = {0};

// 采样定时器句柄
static gptimer_handle_t sample_timer = NULL;

// 编码器中断处理函数
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    uint8_t motor_num = (uint8_t)(uintptr_t)arg;
    if (motor_num < 1 || motor_num > MOTOR_MAX) return;

    const motor_config_t *config = &motor_configs[motor_num - 1];

    // 读取编码器A、B相状态
    int enc_a = gpio_get_level(config->enc_a_pin);
    int enc_b = gpio_get_level(config->enc_b_pin);

    // 根据A、B相状态判断方向并计数
    if (enc_a == enc_b) {
        encoder_counts[motor_num - 1]++;
    } else {
        encoder_counts[motor_num - 1]--;
    }
}

// 定时器回调：采样编码器并计算RPM
static bool IRAM_ATTR sample_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // ISR中不允许浮点运算，只记录脉冲差值
    // RPM计算在 motor_get_rpm() 中（任务上下文）完成
    for (int i = 0; i < MOTOR_MAX; i++) {
        int32_t current_count = encoder_counts[i];
        int32_t last = last_encoder_counts[i];

        // 更新脉冲差值（供 motor_get_rpm 使用）
        pulse_deltas[i] = current_count - last;

        // 更新基准值
        last_encoder_counts[i] = current_count;
    }

    return false; // 返回false表示不需要唤醒任务
}

// 初始化采样定时器
static void sample_timer_init(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1us分辨率
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &sample_timer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = SAMPLE_PERIOD_MS * 1000, // 转换为us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(sample_timer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = sample_timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(sample_timer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(sample_timer));
    ESP_ERROR_CHECK(gptimer_start(sample_timer));

    ESP_LOGI(TAG, "Sample timer initialized (period: %dms)", SAMPLE_PERIOD_MS);
}

// 初始化PWM
static void motor_pwm_init(void)
{
    // 配置LEDC定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // 配置电机1 PWM通道
    ledc_channel_config_t channel_conf1 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR1_PWM_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = motor_configs[0].pwm_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf1);

    // 配置电机2 PWM通道
    ledc_channel_config_t channel_conf2 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR2_PWM_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = motor_configs[1].pwm_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf2);

    // 配置电机3 PWM通道
    ledc_channel_config_t channel_conf3 = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR3_PWM_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = motor_configs[2].pwm_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf3);

    ESP_LOGI(TAG, "PWM initialized");
}

// 初始化GPIO（方向控制和编码器）
static void motor_gpio_init(void)
{
    gpio_config_t io_conf;

    // 配置方向控制引脚为输出
    for (int i = 0; i < MOTOR_MAX; i++) {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << motor_configs[i].in1_pin) |
                               (1ULL << motor_configs[i].in2_pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // 初始状态：停止
        gpio_set_level(motor_configs[i].in1_pin, 0);
        gpio_set_level(motor_configs[i].in2_pin, 0);
    }

    // 配置编码器引脚为输入，带上升沿中断
    for (int i = 0; i < MOTOR_MAX; i++) {
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << motor_configs[i].enc_a_pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);

        // B相只需要输入，不需要中断
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.pin_bit_mask = (1ULL << motor_configs[i].enc_b_pin);
        gpio_config(&io_conf);
    }

    // 安装GPIO中断服务
    gpio_install_isr_service(0);

    // 注册编码器中断处理函数
    for (int i = 0; i < MOTOR_MAX; i++) {
        gpio_isr_handler_add(motor_configs[i].enc_a_pin,
                            encoder_isr_handler,
                            (void *)(uintptr_t)(i + 1));
    }

    ESP_LOGI(TAG, "GPIO initialized");
}

void motor_init(void)
{
    ESP_LOGI(TAG, "Initializing motor driver...");

    // 初始化编码器计数
    for (int i = 0; i < MOTOR_MAX; i++) {
        encoder_counts[i] = 0;
        last_encoder_counts[i] = 0;
        pulse_deltas[i] = 0;
        current_rpm[i] = 0.0f;
    }

    // 初始化PWM
    motor_pwm_init();

    // 初始化GPIO
    motor_gpio_init();

    // 初始化采样定时器
    sample_timer_init();

    ESP_LOGI(TAG, "Motor driver initialized");
    ESP_LOGI(TAG, "Encoder: PPR=%d, gear_ratio=%d, pulses_per_rev=%d",
             ENCODER_PPR, GEAR_RATIO, PULSES_PER_REV);
}

void motor_set(uint8_t motor_num, uint8_t speed, motor_direction_t direction)
{
    // 参数检查
    if (motor_num < 1 || motor_num > MOTOR_MAX) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return;
    }

    if (speed > 100) {
        ESP_LOGW(TAG, "Speed out of range, limiting to 100");
        speed = 100;
    }

    const motor_config_t *config = &motor_configs[motor_num - 1];
    ledc_channel_t channel;

    // 根据电机编号选择PWM通道
    switch (motor_num) {
        case 1: channel = MOTOR1_PWM_CHANNEL; break;
        case 2: channel = MOTOR2_PWM_CHANNEL; break;
        case 3: channel = MOTOR3_PWM_CHANNEL; break;
        default: return;
    }

    // 设置方向
    switch (direction) {
        case MOTOR_DIR_STOP:
            gpio_set_level(config->in1_pin, 0);
            gpio_set_level(config->in2_pin, 0);
            break;
        case MOTOR_DIR_CCW:
            gpio_set_level(config->in1_pin, 0);
            gpio_set_level(config->in2_pin, 1);
            break;
        case MOTOR_DIR_CW:
            gpio_set_level(config->in1_pin, 1);
            gpio_set_level(config->in2_pin, 0);
            break;
        default:
            ESP_LOGE(TAG, "Invalid direction: %d", direction);
            return;
    }

    // 设置PWM占空比 (10位分辨率: 0-1023)
    uint32_t duty = (speed * 1023) / 100;
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);

    ESP_LOGD(TAG, "Motor%d: speed=%d%%, dir=%d, duty=%lu",
             motor_num, speed, direction, duty);
}

void motor_stop_all(void)
{
    for (uint8_t i = 1; i <= MOTOR_MAX; i++) {
        motor_set(i, 0, MOTOR_DIR_STOP);
    }
    ESP_LOGI(TAG, "All motors stopped");
}

int32_t motor_get_encoder(uint8_t motor_num)
{
    if (motor_num < 1 || motor_num > MOTOR_MAX) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return 0;
    }
    return encoder_counts[motor_num - 1];
}

void motor_reset_encoder(uint8_t motor_num)
{
    if (motor_num < 1 || motor_num > MOTOR_MAX) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return;
    }
    encoder_counts[motor_num - 1] = 0;
    last_encoder_counts[motor_num - 1] = 0;
    pulse_deltas[motor_num - 1] = 0;
}

float motor_get_rpm(uint8_t motor_num)
{
    if (motor_num < 1 || motor_num > MOTOR_MAX) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return 0;
    }

    // 读取脉冲差值（int32_t 在32位MCU上是原子读取，无需临界区）
    int32_t delta = pulse_deltas[motor_num - 1];

    // RPM = (脉冲数 / 每转脉冲数) * (60秒 / 采样周期秒)
    float rpm = (float)delta / PULSES_PER_REV * (60.0f / (SAMPLE_PERIOD_MS / 1000.0f));
    current_rpm[motor_num - 1] = rpm;

    return rpm;
}
