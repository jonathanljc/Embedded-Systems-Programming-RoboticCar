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
        ${CMAKE_SOURCE_DIR}/ultrasonic # Include ultrasonic directory 
        ${CMAKE_SOURCE_DIR}/encoder    # Include encoder directory
        ${CMAKE_SOURCE_DIR}/motor      # Include motor directory
        

)

target_link_libraries(car

        ultrasonic_lib  # Link the ultrasonic library
        encoder_lib     # Link the encoder library
        motor_lib       # Link the motor library
)

pico_add_extra_outputs(car)
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 1)

# Remote2 executable
add_executable(remote2
        remote2.c
)

target_include_directories(remote2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/wifi
)

target_compile_definitions(remote2 PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(remote2
        wifi_lib
)

pico_add_extra_outputs(remote2)
pico_enable_stdio_usb(remote2 1)
