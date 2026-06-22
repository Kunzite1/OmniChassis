# OmniChassis

基于 ESP32-S3 的全向轮底盘项目，使用 TB6612FNG 驱动 520 编码电机，通过 UART 接收命令控制三路电机。

## 硬件配置

| 组件 | 型号/参数 |
|------|-----------|
| MCU | ESP32-S3, 160MHz 双核, 2MB Flash |
| 电机驱动 | TB6612FNG × 2 模块 |
| 电机 | 520 编码电机, 330 线 AB 相编码器, 30:1 减速比 |
| IMU | ICM20948 (I2C, SDA=11, SCL=12) — 待实现 |

### 引脚分配

| 电机 | PWM | IN1 | IN2 | 编码器 A | 编码器 B |
|------|-----|-----|-----|----------|----------|
| 1 | GPIO1 | GPIO2 | GPIO4 | GPIO5 | GPIO6 |
| 2 | GPIO7 | GPIO15 | GPIO16 | GPIO17 | GPIO18 |
| 3 | GPIO8 | GPIO38 | GPIO39 | GPIO40 | GPIO41 |

## 构建与烧录

需要 [ESP-IDF v5.2.2](https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32s3/get-started/index.html)。

```bash
# 进入 IDF 环境
source ~/.bashrc && get_idf_522

# 构建
idf.py build

# 烧录并监控串口
idf.py -p /dev/ttyUSB0 flash monitor    # Ctrl+] 退出

# 清理构建
idf.py fullclean
```

## 通信协议

通过 UART0 (115200 baud, 8N1) 发送命令，格式：

```
<电机号 1-3>,<速度 0-100>,<方向 0-1>\n
```

- **方向**: `0` = 逆时针 (CCW), `1` = 顺时针 (CW)
- **速度**: `0` = 停止（方向参数忽略）

示例：
```
1,80,1      # 电机1, 80%速度, 顺时针
2,50,0      # 电机2, 50%速度, 逆时针
3,0,1       # 电机3, 停止
```

## 软件架构

```
UART 输入 ──> uart_rx_task ──> [FreeRTOS 队列] ──> motor_ctrl_task ──> motor_set() ──> GPIO/PWM
```

- **UART 接收任务**: 解析串口命令，发送到队列（队列满时静默丢弃）
- **电机控制任务**: 阻塞等待队列消息，调用 `motor_set()` 执行

### 关键 API

```c
void motor_init(void);                              // 初始化全部外设
void motor_set(uint8_t motor_num, uint8_t speed, motor_direction_t direction);
void motor_stop_all(void);                          // 停止所有电机
int32_t motor_get_encoder(uint8_t motor_num);       // 读取编码器脉冲计数
void motor_reset_encoder(uint8_t motor_num);        // 重置编码器
float motor_get_rpm(uint8_t motor_num);             // 读取 RPM（正负表示方向）
```

## 当前状态

- ✅ 电机驱动: 3 路 PWM 开环控制 + 编码器计数 + RPM 读取
- ✅ UART 命令解析: `num,speed,dir` 格式
- ❌ IMU (ICM20948): 硬件已接线，驱动未实现
- ❌ 闭环控制: `motor_get_rpm()` 已实现但未接入控制回路
- ❌ Wi-Fi: sdkconfig 已启用，应用层无代码

## License

MIT
