/*
 * 全向轮底盘主程序
 * 硬件：ESP32-S3 + TB6612FNG + 520编码电机 + ICM20948
 *
 * 通过UART0接收指令控制电机，格式: num,speed,dir
 *   示例: "1,50,0\n" -> 电机1 50% 逆时针
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "motor.h"
#include "UART0.h"
#include "wifi_udp.h"
#include "attitude.h"

static const char *TAG = "MAIN";

// 电机指令队列
static QueueHandle_t motor_queue = NULL;

// 电机控制任务：阻塞等待队列指令
static void motor_ctrl_task(void *arg)
{
    motor_command_t cmd;

    ESP_LOGI(TAG, "Motor control task started");

    while (1)
    {
        // 阻塞等待队列消息
        if (xQueueReceive(motor_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            const char *dir_str;
            switch (cmd.direction)
            {
            case MOTOR_DIR_CCW:
                dir_str = "CCW";
                break;
            case MOTOR_DIR_CW:
                dir_str = "CW";
                break;
            default:
                dir_str = "STOP";
                break;
            }
            ESP_LOGI("MotorTask", "CMD: Motor%d speed=%d%% dir=%s", cmd.motor_num, cmd.speed, dir_str);

            motor_set(cmd.motor_num, cmd.speed, cmd.direction);

            // 回显确认
            printf("Gotyo motor_%d %s in %d%%\n", cmd.motor_num, dir_str, cmd.speed);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Omni-Wheel Chassis ===");

    motor_init();    // 初始化电机
    wifi_init_sta(); // 初始化wifi通信

    // 创建电机指令队列 (长度5，最多缓存5条指令)
    motor_queue = xQueueCreate(5, sizeof(motor_command_t));
    if (motor_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create motor queue");
        return;
    }

    // 创建电机控制任务
    xTaskCreate(motor_ctrl_task, "motor_ctrl", 3072, NULL, 10, NULL);
    // 创建IMU姿态计算任务
    xTaskCreatePinnedToCore(calculate_task, "calculate_task", 4096, NULL, 5, NULL, 0);

    // 初始化UART0 (传入队列句柄)
    uart0_init(motor_queue);

    ESP_LOGI(TAG, "System ready, send commands via UART0: num,speed,dir");
}
