# Add executable for your project
add_executable(irsensor_app
    irsensor.c
)

# Include FreeRTOS and necessary libraries
target_include_directories(irsensor_app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/FreeRTOS/include            # FreeRTOS core headers
    ${PICO_SDK_PATH}/lib/FreeRTOS/portable/GCC/ARM_CM0   # FreeRTOS port headers
)

# Link necessary libraries
target_link_libraries(irsensor_app
    pico_stdlib
    hardware_adc
    FreeRTOS-Kernel-Heap4  # FreeRTOS kernel and dynamic heap
)

# Add USB or UART support depending on your stdio setup
pico_enable_stdio_usb(irsensor_app 1)
pico_enable_stdio_uart(irsensor_app 0)

# Ensure the executable is flashed to the target device
pico_add_extra_outputs(irsensor_app)
