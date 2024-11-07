# for accelerometer library
add_library(accelerometer_lib
        accelerometer.c
)

target_link_libraries(accelerometer_lib
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_i2c
)

pico_enable_stdio_usb(accelerometer_lib 1)

# for wifi library
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

# Dashboard executable
add_executable(dashboard
        dashboard.c
)

target_include_directories(dashboard PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/wifi
)

target_compile_definitions(dashboard PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(dashboard
        wifi_lib
)

# Remote2 executable
add_executable(remote2
        remote2.c
)

target_include_directories(remote2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/wifi
        ${CMAKE_SOURCE_DIR}/magnetometer
)

target_compile_definitions(remote2 PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(remote2
        wifi_lib
        accelerometer_lib
)

pico_add_extra_outputs(remote2)
pico_enable_stdio_usb(remote2 1)
