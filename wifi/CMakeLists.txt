add_executable(server
        server.c
        )
# target_compile_definitions(server PRIVATE
#         WIFI_SSID=\"${WIFI_SSID}\"
#         WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
#         )
target_include_directories(server PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(server
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        )

pico_add_extra_outputs(server)
pico_enable_stdio_usb(server 1)

add_executable(client
client.c
)
target_include_directories(client PRIVATE
${CMAKE_CURRENT_LIST_DIR}
${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)
target_link_libraries(client
pico_cyw43_arch_lwip_threadsafe_background
pico_stdlib
hardware_adc
)

pico_add_extra_outputs(client)
pico_enable_stdio_usb(client 1)

# clientMQTT executable
add_executable(clientMQTT
        clientMQTT.c
)

target_include_directories(clientMQTT PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
)

target_compile_definitions(clientMQTT PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(clientMQTT
        pico_cyw43_arch_lwip_sys_freertos
        pico_lwip_mqtt
        pico_stdlib
        FreeRTOS-Kernel-Heap4
        hardware_adc
        hardware_i2c
)

pico_add_extra_outputs(clientMQTT)
pico_enable_stdio_usb(clientMQTT 1)