#ifndef WIFI_UDP_H
#define WIFI_UDP_H

#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// --- 配置区 ---
#define WIFI_SSID      "CannotConnect"
#define WIFI_PASS      "ddd775885"
#define UDP_PORT       8848             // 本地监听端口

#define WIFI_UDP_TX_BUF_SIZE    512     // 单次发送缓冲区大小
#define WIFI_UDP_TX_QUEUE_LEN   10      // 发送队列深度

#ifdef __cplusplus
extern "C" {
#endif

// UDP 发送队列数据结构
typedef struct {
    uint8_t payload[WIFI_UDP_TX_BUF_SIZE];
    uint16_t payload_len;
} wifi_udp_tx_item_t;

// 外部声明：用户通过向此队列发送 wifi_udp_tx_item_t 来实现 UDP 发送
extern QueueHandle_t queue_WIFI_TXBUF;

// 函数声明
void wifi_init_sta(void);
void udp_server_task(void *pvParameters);
void udp_heartbeat_task(void *pvParameters);

// 格式化发送：像 printf 一样使用，自动放入 UDP 发送队列
// 例如：WifiUDP_printf("temp: %.1f C, time: %d hours", temp, count);
void WifiUDP_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif