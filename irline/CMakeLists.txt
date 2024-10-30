# Add executable for your project
add_executable(irsensor_app
    irsensor.c
)

# Include FreeRTOS and necessary libraries
target_include_directories(irsensor_app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/FreeRTOS/include            # FreeRTOS core headers
    ${PICO_SDK_PATH}/lib/FreeRTOS/portable/GCC/ARM_CM0   # FreeRTOS port headers
    ${CMAKE_SOURCE_DIR}/wifi
)

target_compile_definitions(irsensor_app PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

# Link necessary libraries
target_link_libraries(irsensor_app
    pico_stdlib
    hardware_adc
    FreeRTOS-Kernel-Heap4  # FreeRTOS kernel and dynamic heap
    wifi_lib
)

# Add USB or UART support depending on your stdio setup
pico_enable_stdio_usb(irsensor_app 1)
pico_enable_stdio_uart(irsensor_app 0)

# Ensure the executable is flashed to the target device
pico_add_extra_outputs(irsensor_app)

# Add executable for your project
add_library(irsensor_lib
    irsensor.c
)

# Include FreeRTOS and necessary libraries
target_include_directories(irsensor_lib PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/FreeRTOS/include            # FreeRTOS core headers
    ${PICO_SDK_PATH}/lib/FreeRTOS/portable/GCC/ARM_CM0   # FreeRTOS port headers
    ${CMAKE_SOURCE_DIR}/wifi
)

# Link necessary libraries
target_link_libraries(irsensor_lib
    pico_stdlib
    hardware_adc
    FreeRTOS-Kernel-Heap4  # FreeRTOS kernel and dynamic heap
    wifi_lib
)

target_compile_definitions(irsensor_lib PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        LWIP_MQTT=1
        LWIP_TCP=1
)

# Add USB or UART support depending on your stdio setup
pico_enable_stdio_usb(irsensor_lib 1)
pico_enable_stdio_uart(irsensor_lib 0)

# Ensure the executable is flashed to the target device
pico_add_extra_outputs(irsensor_lib)

