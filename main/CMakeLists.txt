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

pico_add_extra_outputs(dashboard)
pico_enable_stdio_usb(dashboard 1)



# Car executable
add_executable(car
        car.c
)

target_include_directories(car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..        # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/ultrasonic      # Include ultrasonic directory 
        ${CMAKE_SOURCE_DIR}/encoder         # Include encoder directory
        ${CMAKE_SOURCE_DIR}/motor           # Include motor directory
        ${CMAKE_SOURCE_DIR}/irline          # Include irline directory
        ${CMAKE_SOURCE_DIR}/wifi            # Include wifi directory if required by irsensor_lib
        ${CMAKE_SOURCE_DIR}/irline        # Include irline directory
)

target_compile_definitions(car PRIVATE
    NO_SYS=0            # Setting NO_SYS to 0 as required by lwIP FreeRTOS
    LWIP_SOCKET=1       # Enable socket API if needed
    LWIP_MQTT=1         # Enable MQTT if required
    LWIP_TCP=1          # Enable TCP if needed
)


target_link_libraries(car
        pico_stdlib       # Link standard Pico library
        ultrasonic_lib    # Link the ultrasonic library
        encoder_lib       # Link the encoder library
        motor_lib         # Link the motor library
        irsensor_lib      # Link the irsensor library
        wifi_lib          # Link the wifi library
)



pico_add_extra_outputs(car)
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 1)


# remote executable
add_executable(remote
        remote.c
)

target_include_directories(remote PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${CMAKE_SOURCE_DIR}/wifi
        ${CMAKE_SOURCE_DIR}/magnetometer
)

target_compile_definitions(remote PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

target_link_libraries(remote
        wifi_lib
        accelerometer_lib
)

pico_add_extra_outputs(remote)
pico_enable_stdio_usb(remote 1)
