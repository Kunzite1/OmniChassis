/*
 * 电机驱动头文件
 * 硬件：TB6612FNG + 520编码电机
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// 电机编号定义
#define MOTOR_1    1
#define MOTOR_2    2
#define MOTOR_3    3
#define MOTOR_MAX  3

// 旋转方向定义
typedef enum {
    MOTOR_DIR_STOP = 0,     // 停止
    MOTOR_DIR_CCW = 1,     // 逆时针 (Counter-Clockwise)
    MOTOR_DIR_CW = 2       // 顺时针 (Clockwise)
} motor_direction_t;

// 电机引脚配置结构体
typedef struct {
    uint8_t pwm_pin;        // PWM控制引脚
    uint8_t in1_pin;        // 方向控制引脚1
    uint8_t in2_pin;        // 方向控制引脚2
    uint8_t enc_a_pin;      // 编码器A相引脚
    uint8_t enc_b_pin;      // 编码器B相引脚
} motor_config_t;

/**
 * @brief 初始化所有电机
 *
 * 配置PWM通道和GPIO引脚
 */
void motor_init(void);

/**
 * @brief 设置电机转速和方向
 *
 * @param motor_num 电机编号 (1-3)
 * @param speed     转速百分比 (0-100)
 * @param direction 旋转方向 (MOTOR_DIR_STOP/CCW/CW)
 */
void motor_set(uint8_t motor_num, uint8_t speed, motor_direction_t direction);

/**
 * @brief 停止所有电机
 */
void motor_stop_all(void);

/**
 * @brief 获取电机编码器计数值
 *
 * @param motor_num 电机编号 (1-3)
 * @return int32_t 编码器计数值
 */
int32_t motor_get_encoder(uint8_t motor_num);

/**
 * @brief 重置电机编码器计数
 *
 * @param motor_num 电机编号 (1-3)
 */
void motor_reset_encoder(uint8_t motor_num);

/**
 * @brief 获取电机转速 (RPM)
 *
 * 通过定时采样编码器脉冲计算转速
 *
 * @param motor_num 电机编号 (1-3)
 * @return float 转速 RPM（正负表示方向）
 */
float motor_get_rpm(uint8_t motor_num);

#endif // MOTOR_H
