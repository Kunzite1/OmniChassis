idf_component_register(
    SRCS "wifi_udp.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event esp_netif lwip nvs_flash
)
