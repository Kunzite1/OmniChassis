/*
 * UART0 命令接收组件
 * 通过UART0接收串口指令，格式: num,speed,dir
 *   num:   1-3 (电机编号)
 *   speed: 0-100 (转速百分比)
 *   dir:   0=逆时针(CCW), 1=顺时针(CW)
 */

#ifndef UART0_H
#define UART0_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "motor.h"

// UART0 引脚配置
#define UART0_TX_GPIO       43
#define UART0_RX_GPIO       44
#define UART0_BAUD_RATE     115200
#define UART0_BUF_SIZE      256

// 电机指令结构体（发送到队列）
typedef struct {
    uint8_t motor_num;
    uint8_t speed;
    motor_direction_t direction;
} motor_command_t;

/**
 * @brief 初始化 UART0
 *
 * 配置 UART0 并创建命令接收任务
 *
 * @param motor_queue 用于发送电机指令的队列句柄
 * @return esp_err_t
 */
esp_err_t uart0_init(QueueHandle_t motor_queue);

#endif // UART0_H
