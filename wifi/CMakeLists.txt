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
)

pico_add_extra_outputs(client)
pico_enable_stdio_usb(client 1)