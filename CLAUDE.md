# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 ESP32-S3 的全向轮底盘，TB6612FNG 驱动 520 编码电机（330线编码器），ICM20948 IMU（含 AK09916 磁力计），支持 UART 串口命令和 Wi-Fi UDP 远程控制。使用 ESP-IDF v5.2.2 + FreeRTOS + CMake。

## 常用命令

```bash
# 进入 ESP-IDF 环境（别名，需先 source ~/.bashrc）
get_idf_522

# 构建
idf.py build

# 烧录 + 串口监控（Ctrl+] 退出 monitor）
idf.py -p /dev/ttyUSB0 flash monitor

# 完全清理 + 重新构建（解决编译缓存问题）
idf.py fullclean && idf.py build

# 配置
idf.py menuconfig
```

## 软件架构

### 多任务 + 三队列架构

```
UART 输入 ──> uart_rx_task ──> [motor_queue] ──> motor_ctrl_task ──> motor_set() ──> GPIO/PWM
               (解析命令)       长度 5             (消费执行)

ICM20948 ──> calculate_task ──> WifiUDP_printf() ──> [queue_WIFI_TXBUF] ──> udp_heartbeat_task ──> sendto()
 (I2C)       (200Hz 姿态解算)   (格式化入队)          长度 10                (阻塞出队, UDP 发送)

Wi-Fi STA ──> udp_server_task ──> 记录 peer_addr / global_sock
             (UDP 接收)
```

**任务一览**（优先级均为 10）：

| 任务 | 栈 | 绑核 | 说明 |
|---|---|---|---|
| `uart_rx` | 4096 | - | 解析 UART0 行命令，非阻塞入队 motor_queue |
| `motor_ctrl` | 3072 | - | 阻塞等待 motor_queue，调用 motor_set() |
| `calculate_task` | 3072 (默认) | - | 200Hz 读取 ICM20948 → 窗口滤波 → 姿态解算 → WifiUDP_printf |
| `udp_server` | 4096 | core 1 | UDP 接收/绑定端口 8848，记录对端地址 |
| `udp_heartbeat` | 4096 | core 1 | 从 queue_WIFI_TXBUF 阻塞出队，向对端发 UDP 数据 |

### 组件依赖链

```
main (REQUIRES motor UART0 wifi_udp)     ← 尚未接入 ICM20948/attitude_calculation
  ├── motor (REQUIRES driver)
  ├── UART0 (REQUIRES driver motor)
  ├── wifi_udp (REQUIRES driver esp_wifi esp_event esp_netif lwip nvs_flash)
  ├── ICM20948 (REQUIRES driver)          ← I2C 驱动，含 AK09916 磁力计
  └── attitude_calculation (REQUIRES driver)  ← 缺失 ICM20948 wifi_udp 依赖声明
```

### 姿态解算数据流

```
ICM20948 (I2C)
  │  icm20948_read_agm()  →  icm20948_data_t (原始 int16)
  ▼
process_raw_data()
  │  窗口滤波 (WIN_NUM=5) → 零偏补偿 → 物理量转换
  │  acc: /16384 → g     gyro: /57.1 → °/s    mag: 校准系数 → µT
  ▼
calculate_attitude(cycle=0.005)
  │  ① 加速度计叉乘误差 + 互补滤波
  │  ② 磁力计 yaw 偏航修正 (mag_yaw_bias)
  │  ③ 一阶龙格库塔更新四元数
  │  ④ 四元数 → 旋转矩阵 → 欧拉角 (rol/pitch/yaw)
  │  ⑤ 世界坐标系 acc/mag 变换
  ▼
WifiUDP_printf("%f,%f,%f,%f\n", qw, qx, qy, qz)   ← 四元数通过 UDP 发出
```

## 组件详解

### 电机驱动 (`components/motor/`)

- **PWM**: LEDC 定时器 0，低速模式，1 KHz, 10 位分辨率（0-1023）
- **方向**: TB6612FNG 的 IN1/IN2 — 00=coast, 01=CCW, 10=CW, 11=brake（不会出现）
- **编码器**: A 相上升沿中断，读 A/B 相位判断方向（四倍频）
- **RPM**: gptimer 每 100ms 采样差值 → `motor_get_rpm()` 计算浮点值（任务上下文）
- **停止**: coast 模式（IN1=0, IN2=0），不刹车

```c
#define ENCODER_PPR         330     // 编码器线数
#define ENCODER_MULTIPLIER  4       // 四倍频
#define GEAR_RATIO          30      // 减速比
#define PULSES_PER_REV      39600   // 输出轴每转脉冲 = 330 × 4 × 30
#define SAMPLE_PERIOD_MS    100     // RPM 采样周期
```

### UART0 命令协议 (`components/UART0/`)

- **接口**: UART0, GPIO43(TX), GPIO44(RX), 115200 baud, 8N1, 缓冲区 256 字节
- **命令格式**: 每行一条 LF 终止，`<电机号 1-3>,<速度 0-100>,<方向 0-1>\n`；方向 0=CCW, 1=CW；速度为 0 自动停止
- **行缓冲**: 最大 32 字符，超长清空重置
- **响应**: 回显 `Gotyo motor_%d %s in %d%%\n`
- **队列溢出**: motor_queue 长度仅 5，满时静默丢弃旧消息

### Wi-Fi UDP 通信 (`components/wifi_udp/`)

- **模式**: STA，硬编码 SSID/PASS（`wifi_udp.h` 中宏定义）
- **UDP 接收**: 绑定端口 8848，收到数据后记录对端 IP+端口（`peer_addr`）
- **UDP 发送**: `WifiUDP_printf()` 格式化 → 入队 `queue_WIFI_TXBUF` → `udp_heartbeat_task` 发送
- **重连**: 断线自动重连；NVS 失败自动擦除重试
- **发送队列**: 长度 10，满则丢弃（非阻塞）

### ICM20948 IMU 驱动 (`components/ICM20948/`)

- **接口**: I2C 主机模式，400 KHz，SDA=GPIO2, SCL=GPIO1（当前头文件定义）
- **传感器**: ICM20948 加速度计/陀螺仪 (0x68) + AK09916 磁力计 (0x0C)
- **功能**: `imu_init()` 一键初始化 I2C + 传感器配置；`icm20948_read_agm()` 同步读取九轴原始数据
- **量程**: 加速度 ±2G (16384 LSB/g)，陀螺仪 ±500dps (57.1 LSB/dps)
- **磁力计**: 旁路模式通过 ICM20948 INT_PIN_CFG 访问 AK09916，±4912 µT

### 姿态解算 (`components/attitude_calculation/`)

- **窗口滤波**: 滑动窗口 (WIN_NUM=5) 均值滤波，去除传感器白噪声
- **姿态算法**: 基于四元数的互补滤波，加速度计 + 磁力计修正陀螺仪漂移
  - 加速度计叉乘误差 → 低通 → PI 修正
  - 磁力计 yaw 偏差 → 比例修正
  - 一阶龙格库塔更新四元数
  - 四元数 → 旋转矩阵 → 欧拉角 (roll/pitch/yaw)
- **输出**: 通过 Wi-Fi UDP 发送四元数（`qw,qx,qy,qz\n` 格式），200Hz 频率
- **数学库**: 自定义定点近似 `fast_sqrt`（Q_rsqrt）、`my_sin`/`my_cos`（抛物线近似）、`arctan2`，避免依赖 math.h

## 关键 API

```c
// motor.h
void motor_init(void);
void motor_set(uint8_t motor_num, uint8_t speed, motor_direction_t direction);
void motor_stop_all(void);
int32_t motor_get_encoder(uint8_t motor_num);
void motor_reset_encoder(uint8_t motor_num);
float motor_get_rpm(uint8_t motor_num);       // RPM（正负表示方向）

// UART0.h
typedef struct { uint8_t motor_num, speed, direction; } motor_command_t;
void uart0_init(QueueHandle_t motor_queue);

// wifi_udp.h
void wifi_init_sta(void);                     // 阻塞等待 Wi-Fi 连接成功
void WifiUDP_printf(const char *format, ...); // 格式化发送 UDP（线程安全）
extern QueueHandle_t queue_WIFI_TXBUF;

// ICM20948.h
icm20948_handle_t imu_init(void);             // 一键初始化 I2C + ICM20948 + AK09916
esp_err_t icm20948_read_agm(i2c_master_dev_handle_t dev, i2c_master_dev_handle_t mag, icm20948_data_t *data);
void icm20948_print_rawdata(const icm20948_data_t *data);
uint8_t icm20948_get_who_am_i(i2c_master_dev_handle_t dev);

// attitude.h
void calculate_task(void *pvParameters);      // 姿态解算任务入口（内部调 imu_init）
```

## 启动流程 (`app_main` — 当前状态)

```
motor_init() → wifi_init_sta() → [motor_queue] → motor_ctrl_task → uart0_init()
```

**注意**: `calculate_task` 尚未在 `app_main` 中创建。需新增：
```c
xTaskCreate(calculate_task, "calculate", 4096, NULL, 10, NULL);
```

## 已知问题

1. **`attitude.c` 包含不存在的头文件** — `#include "myuart.h"` 在项目中不存在，会导致编译失败（该调用已注释，但 `#include` 仍在）
2. **`calculate_task` 未集成** — IMU 初始化和姿态解算代码已完成但未在 `app_main` 中启动

## 引脚分配

| 电机 | PWM | IN1 | IN2 | 编码器 A | 编码器 B |
|---|---|---|---|---|---|
| 1 | GPIO1 | GPIO2 | GPIO4 | GPIO5 | GPIO6 |
| 2 | GPIO7 | GPIO15 | GPIO16 | GPIO17 | GPIO18 |
| 3 | GPIO8 | GPIO38 | GPIO39 | GPIO40 | GPIO41 |

**其他**: ICM20948 (I2C): SDA=2, SCL=1 **（注意：头文件定义为 GPIO1/2，引脚接线文档为 GPIO11/12 — 需确认）** | UART0: TX=43, RX=44。

## 当前状态

- ✅ **电机驱动**: 3 路 PWM 开环控制，编码器计数 + RPM 读取
- ✅ **UART 命令解析**: 串口接收 `num,speed,dir` 格式命令
- ✅ **Wi-Fi UDP**: STA 模式 + UDP 收发，`WifiUDP_printf()` 格式化发送
- ✅ **ICM20948 驱动**: I2C 初始化、九轴数据读取、量程配置
- ✅ **姿态解算**: 互补滤波四元数解算、窗口滤波、200Hz 更新
- ❌ **闭环控制**: `motor_get_rpm()` 已实现但未接入控制回路
- ⚠️ **IMU 集成**: 驱动和姿态代码已完成但未接入 `app_main`
- ⚠️ **引脚冲突**: IMU 的 I2C 引脚头文件定义 (GPIO1/2) 与接线文档 (GPIO11/12) 不一致

## 开发注意事项

1. **串口权限**: Linux 需将用户添加到 `dialout` 组
2. **编译数据库**: `.vscode/c_cpp_properties.json` 指向 `build/compile_commands.json`
3. **ISR 中禁止浮点**: 编码器采样在 ISR 执行，RPM 计算在任务上下文
4. **Wi-Fi 凭据**: 硬编码在 `wifi_udp.h` 的 `WIFI_SSID` / `WIFI_PASS` 宏
5. **UDP 端口**: 本地监听端口 `8848` 在 `wifi_udp.h` 的 `UDP_PORT` 宏
6. **IMU I2C 引脚确认**: `ICM20948.h` 中 SDA=GPIO2, SCL=GPIO1，但 `引脚接线.md` 记录为 GPIO11/12 — 烧录前需确认实际接线
7. **姿态解算参数**: 零偏值 `gyro_zero_x/y/z`、`acc_zero_x/y/z`、`mag_xsf/ysf` 在 `attitude.c:init_data()` 中硬编码，不同芯片需重新校准
8. **命令队列满**: motor_queue 深度仅 5，高频命令会被丢弃
9. **GPIO 冲突**: 电机 2 编码器 A/B 使用 GPIO17/18（ESP32-S3 JTAG 默认引脚），调试时可能冲突
