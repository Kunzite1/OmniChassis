/*
 * UART0 命令接收实现
 *
 * 接收格式: num,speed,dir\n
 * 示例:
 *   "1,50,0"  -> 电机1 50% 逆时针
 *   "2,80,1"  -> 电机2 80% 顺时针
 *   "3,0,0"   -> 电机3 停止
 */

#include "UART0.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "UART0";

// 队列句柄（保存uart0_init传入的队列）
static QueueHandle_t cmd_queue = NULL;

// 接收缓冲区
#define LINE_BUF_MAX    32

// 解析一行命令: "num,speed,dir"
// 返回 true 表示解析成功
static bool parse_command(const char *line, motor_command_t *cmd)
{
    int num, speed, dir;

    // 跳过空行
    if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n') {
        return false;
    }

    // 解析三个整数
    int matched = sscanf(line, "%d,%d,%d", &num, &speed, &dir);
    if (matched != 3) {
        ESP_LOGW(TAG, "Parse failed, expected: num,speed,dir, got: %s", line);
        return false;
    }

    // 参数校验
    if (num < 1 || num > 3) {
        ESP_LOGW(TAG, "Invalid motor num: %d (expected 1-3)", num);
        return false;
    }
    if (speed > 100) {
        ESP_LOGW(TAG, "Speed clamped: %d -> 100", speed);
        speed = 100;
    }
    if (dir < 0 || dir > 1) {
        ESP_LOGW(TAG, "Invalid dir: %d (expected 0 or 1)", dir);
        return false;
    }

    cmd->motor_num = (uint8_t)num;
    cmd->speed = (uint8_t)speed;

    // dir=0 -> CCW, dir=1 -> CW; speed=0 时自动设为 STOP
    if (speed == 0) {
        cmd->direction = MOTOR_DIR_STOP;
    } else {
        cmd->direction = (dir == 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
    }

    return true;
}

// UART0 接收任务
static void uart_rx_task(void *arg)
{
    uint8_t data[UART0_BUF_SIZE];
    char line[LINE_BUF_MAX];
    int line_pos = 0;

    ESP_LOGI(TAG, "UART0 RX task started");

    while (1) {
        // 读取UART0数据
        int len = uart_read_bytes(UART_NUM_0, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        data[len] = '\0';

        // 逐字节处理，拼装命令行
        for (int i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\n' || c == '\r') {
                // 遇到换行，解析当前行
                if (line_pos > 0) {
                    line[line_pos] = '\0';

                    motor_command_t cmd;
                    if (parse_command(line, &cmd)) {
                        // 发送到队列（不阻塞，队列满则丢弃）
                        if (cmd_queue != NULL) {
                            BaseType_t ret = xQueueSend(cmd_queue, &cmd, 0);
                            if (ret != pdTRUE) {
                                ESP_LOGW(TAG, "Command queue full, dropping: %s", line);
                            }
                        }
                    }
                    line_pos = 0;
                }
            } else if (line_pos < LINE_BUF_MAX - 1) {
                line[line_pos++] = c;
            } else {
                // 行缓冲区溢出，重置
                ESP_LOGW(TAG, "Line buffer overflow, resetting");
                line_pos = 0;
            }
        }
    }
}

esp_err_t uart0_init(QueueHandle_t motor_queue)
{
    cmd_queue = motor_queue;

    // 配置 UART0 参数
    uart_config_t uart_config = {
        .baud_rate = UART0_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART0_TX_GPIO, UART0_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, UART0_BUF_SIZE, 0, 0, NULL, 0));

    // 创建接收任务
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "UART0 initialized (TX:%d, RX:%d, %d baud)", UART0_TX_GPIO, UART0_RX_GPIO, UART0_BAUD_RATE);
    return ESP_OK;
}
