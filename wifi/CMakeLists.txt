add_library(wifi_lib
        wifi.c
)

target_include_directories(wifi_lib PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_compile_definitions(wifi_lib PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(wifi_lib
        pico_cyw43_arch_lwip_sys_freertos
        pico_lwip_mqtt
        pico_stdlib
        FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(wifi_lib 1)