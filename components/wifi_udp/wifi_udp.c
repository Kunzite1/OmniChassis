#include "wifi_udp.h"
#include <stdarg.h>
#include "lwip/inet.h"
#include "endian.h"

static const char *TAG = "WIFI_UDP";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static struct sockaddr_in peer_addr; // 存储 ROS2 节点的地址
static bool peer_connected = false;  // 是否已获取到地址
static int global_sock = -1;         // 将 socket 提升为全局，方便发送任务使用

QueueHandle_t queue_WIFI_TXBUF = NULL; // UDP 发送队列句柄

// Wi-Fi 事件处理回调
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Wi-Fi 启动，开始连接...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi 关联成功，等待 DHCP...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *discon = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "断开连接，原因码: %d，重连中...", discon->reason);
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "获取到IP: %s", ip_str);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// 初始化 Wi-Fi (STA 模式)
void wifi_init_sta(void)
{
    // 初始化 NVS (Wi-Fi 驱动必需)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 初始化发送队列
    queue_WIFI_TXBUF = xQueueCreate(WIFI_UDP_TX_QUEUE_LEN, sizeof(wifi_udp_tx_item_t));
    if (queue_WIFI_TXBUF == NULL)
    {
        ESP_LOGE(TAG, "发送队列创建失败");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "发送队列已创建，深度: %d", WIFI_UDP_TX_QUEUE_LEN);

    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(udp_heartbeat_task, "udp_heartbeat", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Wi-Fi 初始化完成，等待连接...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi 已连接，就绪");
}

// UDP 任务：接收并回显数据
void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[32];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    while (1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);

        // 1. 创建 Socket
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "无法创建 Socket: errno %d", errno);
            break;
        }

        // 2. 绑定端口
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "绑定失败: errno %d", errno);
        }

        ESP_LOGI(TAG, "UDP 服务器已启动，监听端口: %d", UDP_PORT);

        while (1)
        {
            struct sockaddr_storage source_addr;
            socklen_t socklen = sizeof(source_addr);

            // 3. 阻塞接收数据
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0)
            {
                ESP_LOGE(TAG, "接收失败: errno %d", errno);
                break;
            }
            else
            {
                rx_buffer[len] = 0; // 字符串结束符
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "来自 %s 的数据: %s", addr_str, rx_buffer);

                // 保存对方的 IP 和端口
                peer_addr = *(struct sockaddr_in *)&source_addr;
                peer_connected = true;
                global_sock = sock; // 保存 socket 句柄

                // 原路返回（回显）或发给指定目标
                // int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                // if (err < 0)
                // {
                //     ESP_LOGE(TAG, "发送回显失败: errno %d", errno);
                //     break;
                // }
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "关闭 Socket 并重试...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void udp_heartbeat_task(void *pvParameters)
{
    wifi_udp_tx_item_t tx_item;

    while (1)
    {
        // 阻塞等待队列数据，有数据则发送
        if (xQueueReceive(queue_WIFI_TXBUF, &tx_item, portMAX_DELAY) == pdTRUE)
        {
            if (peer_connected && global_sock != -1)
            {
                int err = sendto(global_sock, tx_item.payload, tx_item.payload_len, 0,
                                 (struct sockaddr *)&peer_addr, sizeof(peer_addr));
                if (err < 0)
                    ESP_LOGE(TAG, "队列数据发送失败: errno %d", errno);
                else
                    ESP_LOGI(TAG, "已发送 %d 字节队列数据", tx_item.payload_len);
            }
            else
            {
                ESP_LOGW(TAG, "尚未获取到对端地址，无法发送队列数据");
            }
        }
    }
}

void WifiUDP_printf(const char *format, ...)
{
    if (queue_WIFI_TXBUF == NULL)
        return;

    wifi_udp_tx_item_t tx_item;

    va_list args;
    va_start(args, format);
    tx_item.payload_len = vsnprintf((char *)tx_item.payload, sizeof(tx_item.payload), format, args);
    va_end(args);

    if (tx_item.payload_len > sizeof(tx_item.payload))
        tx_item.payload_len = sizeof(tx_item.payload);

    // 非阻塞入队，满则丢弃
    xQueueSend(queue_WIFI_TXBUF, &tx_item, 0);
    ESP_LOGI(TAG, "入队: %s", (char *)tx_item.payload);
}