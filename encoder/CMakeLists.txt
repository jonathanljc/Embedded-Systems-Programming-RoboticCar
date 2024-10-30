# Define a static library for the encoder module
add_executable(encoder 
    encoder.c  # Add encoder source file here
)

# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more)
target_link_libraries(encoder
    pico_stdlib          # Standard libraries
    hardware_gpio        # GPIO control
    hardware_timer       # Timer control
    FreeRTOS-Kernel      # FreeRTOS kernel
    FreeRTOS-Kernel-Heap4  # FreeRTOS heap
)

# Enable USB and UART output for stdio (for debugging messages)
pico_enable_stdio_usb(encoder 1)
pico_enable_stdio_uart(encoder 1)

# Ensure the executable is flashed to the target device
pico_add_extra_outputs(encoder)
