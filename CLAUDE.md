# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 ESP32-S3 的全向轮底盘项目，使用 TB6612FNG 驱动 520 编码电机（330线编码器），配置 ICM20948 IMU 传感器。采用 FreeRTOS 实时操作系统，使用 ESP-IDF v5.2.2 和 CMake 构建系统。

## 常用命令

### 环境设置

项目已安装 ESP-IDF v5.2.2，通过 alias 进入 IDF 终端：

```bash
source ~/.bashrc && get_idf_522
```

### 构建和烧录

```bash
idf.py build
idf.py -p PORT flash monitor    # 按 Ctrl+] 退出 monitor
idf.py fullclean
```

### 配置

```bash
idf.py menuconfig
```

### 清理 + 重新构建（解决编译缓存问题）

```bash
idf.py fullclean && idf.py build
```

## 项目结构

```
ESP32_proj/
├── CMakeLists.txt                 # 项目根 CMake 配置（project: ESP32_PROJ）
├── main/                          # 主应用组件
│   ├── CMakeLists.txt             # REQUIRES motor UART0
│   └── main.c                     # 主程序入口（app_main）
├── components/
│   ├── motor/                     # 电机驱动组件
│   │   ├── CMakeLists.txt         # REQUIRES driver
│   │   ├── motor.h                # 电机驱动接口
│   │   └── motor.c                # 电机驱动实现
│   └── UART0/                     # UART 命令解析组件
│       ├── CMakeLists.txt         # REQUIRES driver motor
│       ├── UART0.h                # UART 接口
│       └── UART0.c                # UART 命令接收与解析
├── .vscode/                       # VSCode 配置
├── 引脚接线.md                    # 硬件接线文档
└── sdkconfig                      # 项目配置（ESP32-S3, 160MHz, 2MB flash）
```

**注意**: `README.md` 和 `pytest_hello_world.py` 是 ESP-IDF 模板遗留文件，未针对本定制化 — 请勿使用。

## 软件架构

### 基于队列的命令模式

整个固件围绕一个 FreeRTOS 队列（`xQueueCreate(5, sizeof(motor_command_t))`）构建：

```
UART 输入 ──> uart_rx_task ──> [队列] ──> motor_ctrl_task ──> motor_set() ──> GPIO/PWM
                 (解析)          长度 5         (消费)            (执行)
```

- **UART 接收任务** (`uart_rx`, 优先级 10, 栈 4096): 解析 `num,speed,dir\n` 格式的行，以非阻塞方式发送至队列
- **电机控制任务** (`motor_ctrl_task`, 优先级 10, 栈 3072): 阻塞等待队列消息，调用 `motor_set()`
- 两个任务优先级均为 10（高于空闲/定时器任务）；若队列满，UART 端静默丢弃旧消息

### 组件依赖链

```
main (REQUIRES motor UART0)
  ├── motor (REQUIRES driver)
  │     └── driver (ESP-IDF 内置)
  └── UART0 (REQUIRES driver motor)
```

### UART 通信协议

- **接口**: UART0, GPIO43(TX), GPIO44(RX), 115200 baud, 8N1, 缓冲区 256 字节
- **命令格式**: 回车符 `\n` 终止，每行一条命令 (LF, 非 CR+LF)
  
  ```
  <电机号 1-3>,<速度 0-100>,<方向 0-1>
  ```
  - 方向 `0` = CCW (逆时针), `1` = CW (顺时针)
  - 速度为 0 时自动设为停止（方向参数会被忽略）
  - 行缓冲最大 32 字符；超长会清空重置
- **响应**: 固件通过 UART 回显 `Gotyo motor_%d %s in %d%%\n`
- **编码**: ASCII, 无校验和、无转义

### 电机驱动 (`components/motor/`)

- **PWM 控制**: LEDC 定时器 0，低速模式，1 KHz, 10 位分辨率（0-1023）
- **方向控制**: GPIO 控制 TB6612FNG 的 IN1/IN2
- **编码器计数**: A 相 GPIO 上升沿中断，读取 A/B 相电平判断方向（四倍频）
- **RPM 计算**: gptimer 每 100ms 采样编码器差值存入 `pulse_deltas[]`，在 `motor_get_rpm()`（任务上下文，非 ISR）中计算浮点 RPM
- **电机停止**: 设置 IN1=0, IN2=0（coast 模式），不刹车

### TB6612FNG 方向控制

- `IN1=0, IN2=0`: 停止 (coast)
- `IN1=0, IN2=1`: 逆时针 (CCW)
- `IN1=1, IN2=0`: 顺时针 (CW)
- `IN1=1, IN2=1`: 刹车 (BRAKE) — 当前未使用，且通过 `motor_set()` 不可能出现此状态

### 编码器参数

```c
#define ENCODER_PPR         330     // 编码器线数
#define ENCODER_MULTIPLIER  4       // 四倍频
#define GEAR_RATIO          30      // 减速比
#define PULSES_PER_REV      39600   // 输出轴每转脉冲数 = 330 × 4 × 30
#define SAMPLE_PERIOD_MS    100     // RPM 采样周期（gptimer）
```

### 关键 API

```c
// motor.h
void motor_init(void);          // 初始化 LEDC PWM + GPIO + 编码器 + 采样定时器
void motor_set(uint8_t motor_num, uint8_t speed, motor_direction_t direction);
void motor_stop_all(void);
int32_t motor_get_encoder(uint8_t motor_num);
void motor_reset_encoder(uint8_t motor_num);
float motor_get_rpm(uint8_t motor_num);   // 返回 RPM（正负表示方向）

// UART0.h
typedef struct {
    uint8_t motor_num;   // 1-3
    uint8_t speed;       // 0-100 (%)
    uint8_t direction;   // 0=CCW, 1=CW
} motor_command_t;

void uart0_init(QueueHandle_t motor_queue);
```

## 引脚分配

### 电机

| 电机  | PWM | IN1 | IN2 | 编码器 A | 编码器 B |
| --- | --- | --- | --- | ----- | ----- |
| 1   | 1   | 2   | 4   | 5     | 6     |
| 2   | 7   | 15  | 16  | 17    | 18    |
| 3   | 8   | 38  | 39  | 40    | 41    |

### 其他外设

- **ICM20948 IMU**: SDA=11, SCL=12 (I2C) — **硬件已接线但尚未实现驱动代码**
- **UART0 控制台**: TX=43, RX=44 (ESP32-S3 默认)

### 供电

- ESP32-S3: 5V / 3.3V
- 电机驱动: VCC=5V, VM≤12V
- 编码器: 5V
- IMU: 3.3V

## sdkconfig 关键设置

| 参数          | 值                    |
| ----------- | -------------------- |
| 目标芯片        | ESP32-S3             |
| IDF 版本      | v5.2.2               |
| CPU 频率      | 160 MHz (双核)         |
| Flash       | 2 MB, DIO 模式, 80 MHz |
| PSRAM       | 未启用                  |
| FreeRTOS 节拍 | 100 Hz (10ms)        |
| 任务 WDT      | 5 秒超时                |
| 日志级别        | Info (默认)            |
| 调试          | GDB Stub 已启用         |

## 当前状态

- ✅ **电机驱动**: 已完成并验证 — 支持 3 路电机，PWM 开环速度控制，编码器计数及 RPM 读取
- ✅ **UART 命令解析**: 已完成 — 串口接收 `num,speed,dir` 格式命令
- ❌ **IMU (ICM20948)**: 硬件已连接（SDA=11, SCL=12），**尚无 I2C 驱动代码**
- ❌ **闭环控制**: `motor_get_rpm()` 已实现但未被调用 — 当前为开环控制
- ❌ **Wi-Fi**: sdkconfig 已启用但应用层无相关代码

## 开发注意事项

1. **串口权限**: Linux 下需将用户添加到 `dialout` 组
2. **编译数据库**: `.vscode/c_cpp_properties.json` 指向 `build/compile_commands.json` — 先编译一次再使用 VSCode IntelliSense
3. **ISR 中禁止浮点运算**: 编码器采样回调在 ISR 中执行，RPM 计算已移到 `motor_get_rpm()` 函数（任务上下文）
4. **PWM 频率**: LEDC 默认 1 KHz，修改 `motor.c` 中的 `LEDC_FREQUENCY`
5. **减速比**: 根据实际电机修改 `motor.c` 中的 `GEAR_RATIO`
6. **命令队列溢出**: 队列长度仅为 5，高频命令会被静默丢弃 — 如需精准控制需增大队列长度或采用覆盖式写入
