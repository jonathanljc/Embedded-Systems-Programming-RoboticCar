add_executable(dashboard
        dashboard.c
        )

target_include_directories(dashboard PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(dashboard
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        )

pico_add_extra_outputs(dashboard)
pico_enable_stdio_usb(dashboard 1)

add_executable(remote
        remote.c
        )

target_include_directories(remote PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

target_compile_definitions(remote PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )

target_link_libraries(remote
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        FreeRTOS-Kernel-Heap4
        hardware_adc
        )

pico_add_extra_outputs(remote)
pico_enable_stdio_usb(remote 1)