file(GLOB SRCS "*.c")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event esp_netif lwip nvs_flash
)
