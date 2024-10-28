# Dashboard executable
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


# Remote executable
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
        hardware_i2c
)

pico_add_extra_outputs(remote)
pico_enable_stdio_usb(remote 1)


# Car executable
add_executable(car
        car.c
)

target_include_directories(car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..   # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/ultrasonic # Include ultrasonic directory if needed
        

)

target_link_libraries(car

        ultrasonic_lib  # Link the ultrasonic library
)

pico_add_extra_outputs(car)
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 1)
