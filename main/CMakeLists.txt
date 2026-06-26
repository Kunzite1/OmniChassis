idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES motor UART0 wifi_udp)
