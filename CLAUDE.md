# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 ESP32-S3 的全向轮底盘，TB6612FNG 驱动 520 编码电机（330线编码器），ICM20948 IMU，支持 UART 串口命令和 Wi-Fi UDP 远程控制。使用 ESP-IDF v5.2.2 + FreeRTOS + CMake。

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

### 多任务 + 双队列架构

```
UART 输入 ──> uart_rx_task ──> [motor_queue] ──> motor_ctrl_task ──> motor_set() ──> GPIO/PWM
               (解析命令)       长度 5             (消费执行)
                                         ┌─────────────────────────────┐
Wi-Fi STA ──> udp_server_task ──────────>│ peer_addr (对端IP+端口)      │
                (接收UDP, 记录对方地址)     │ global_sock (全局socket句柄) │
                                         └──────────┬──────────────────┘
                                                     │
                                         [queue_WIFI_TXBUF]
                                         (长度 10)
                                              ▲
                                              │
              motor_ctrl_task ──> WifiUDP_printf()
              (或其他任务)          (格式化入队)
                                       │
                                       └──> udp_heartbeat_task ──> sendto()
                                            (阻塞出队，发送UDP数据)
```

**任务一览**：所有任务优先级为 10（高于空闲/定时器任务），唯 UDP 任务绑核到 core 1：

| 任务 | 栈 | 说明 |
|---|---|---|
| `uart_rx` | 4096 | 解析 UART0 行命令，非阻塞入队 motor_queue |
| `motor_ctrl` | 3072 | 阻塞等待 motor_queue，调用 motor_set()，可通过 WifiUDP_printf() 上报状态 |
| `udp_server` | 4096 | UDP 接收/绑定端口，记录对端地址（绑核 core 1） |
| `udp_heartbeat` | 4096 | 从 queue_WIFI_TXBUF 阻塞出队，向对端发送 UDP 数据（绑核 core 1） |

### 组件依赖链

```
main (REQUIRES motor UART0 wifi_udp)
  ├── motor (REQUIRES driver)
  │     └── driver (ESP-IDF 内置)
  ├── UART0 (REQUIRES driver motor)
  └── wifi_udp (REQUIRES driver esp_wifi esp_event esp_netif lwip nvs_flash)
```

## 组件详解

### 电机驱动 (`components/motor/`)

- **PWM**: LEDC 定时器 0，低速模式，1 KHz, 10 位分辨率（0-1023）
- **方向**: TB6612FNG 的 IN1/IN2 — 00=coast, 01=CCW, 10=CW, 11=brake（不会出现）
- **编码器**: A 相上升沿中断，读 A/B 相位判断方向（四倍频）
- **RPM**: gptimer 每 100ms 采样差值 → 存入 `pulse_deltas[]`，`motor_get_rpm()` 中计算浮点值（任务上下文，非 ISR）
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
- **UDP 接收**: 绑定端口 8848，收到数据后记录对端 IP+端口（`peer_addr`），实现单向命令接收
- **UDP 发送**: 通过 `WifiUDP_printf()` 格式化字符串 → 入队 `queue_WIFI_TXBUF` → `udp_heartbeat_task` 阻塞出队并 `sendto(peer_addr)`
- **心跳/重连**: Wi-Fi 断线自动重连；NVS 首次初始化失败自动擦除重试
- **发送队列**: 长度 10，满则丢弃（非阻塞入队）

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
extern QueueHandle_t queue_WIFI_TXBUF;        // 也可直接向此队列发送 wifi_udp_tx_item_t
```

## 启动流程 (`app_main`)

```
motor_init() → wifi_init_sta() → [创建 motor_queue] → motor_ctrl_task → uart0_init(motor_queue)
    ▲                                    ▲                          ▲
  初始化 LEDC                        队列长度 5                 传入队列给 UART
  GPIO/编码器/定时器                                              解析任务
```

**注意**: `wifi_init_sta()` 内部会阻塞等待 Wi-Fi 连接成功（portMAX_DELAY），之后才会进入主循环。

## 引脚分配

| 电机 | PWM | IN1 | IN2 | 编码器 A | 编码器 B |
|---|---|---|---|---|---|
| 1 | GPIO1 | GPIO2 | GPIO4 | GPIO5 | GPIO6 |
| 2 | GPIO7 | GPIO15 | GPIO16 | GPIO17 | GPIO18 |
| 3 | GPIO8 | GPIO38 | GPIO39 | GPIO40 | GPIO41 |

**其他**: ICM20948 (I2C): SDA=11, SCL=12 | UART0: TX=43, RX=44。详见 `引脚接线.md`。

## sdkconfig 关键设置

- 芯片: ESP32-S3, 160 MHz 双核, 2MB Flash (DIO, 80 MHz), 无 PSRAM
- FreeRTOS 节拍 100 Hz, 任务 WDT 5 秒超时
- 日志级别 Info, 调试 GDB Stub 已启用

## 当前状态

- ✅ **电机驱动**: 3 路 PWM 开环控制，编码器计数 + RPM 读取
- ✅ **UART 命令解析**: 串口接收 `num,speed,dir` 格式命令
- ✅ **Wi-Fi UDP**: STA 模式 + UDP 收发，`WifiUDP_printf()` 格式化发送
- ❌ **ICM20948 IMU**: 硬件已接线（SDA=11, SCL=12），无 I2C 驱动代码
- ❌ **闭环控制**: `motor_get_rpm()` 已实现但未被调用 — 当前为开环控制

## 开发注意事项

1. **串口权限**: Linux 需将用户添加到 `dialout` 组
2. **编译数据库**: `.vscode/c_cpp_properties.json` 指向 `build/compile_commands.json` — 先编译再使用 IntelliSense
3. **ISR 中禁止浮点**: 编码器采样在 ISR 执行，RPM 计算在任务上下文
4. **Wi-Fi 凭据**: 硬编码在 `wifi_udp.h` 的 `WIFI_SSID` / `WIFI_PASS` 宏，生产使用需改为 menuconfig 或 NVS 存储
5. **UDP 端口**: 本地监听端口 `8848` 在 `wifi_udp.h` 的 `UDP_PORT` 宏
6. **UDP 发送队列满**: 队列深度 10，满则静默丢弃 `WifiUDP_printf()` 数据
7. **命令队列满**: motor_queue 深度仅 5，高频命令会被丢弃 — 需精准控制需增大队列或改用覆盖式写入
8. **GPIO 冲突**: 电机 2 的编码器 A/B 使用 GPIO17/18（ESP32-S3 的 JTAG 默认引脚），调试时可能冲突 — 如遇 JTAG 异常可通过 `GPIO矩阵` 重映射或禁用 JTAG
