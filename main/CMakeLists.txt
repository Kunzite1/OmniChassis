file(GLOB SRCS "*.c")
idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS "."
                    REQUIRES motor UART0 wifi_udp attitude_calculation)
