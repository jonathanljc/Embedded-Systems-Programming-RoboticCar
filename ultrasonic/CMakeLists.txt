# Define a static library for the ultrasonic module
add_library(ultrasonic_lib
    ultrasonic.c  # Add your ultrasonic source file here
)

# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more)
target_link_libraries(ultrasonic_lib
    pico_stdlib          # Standard libraries
    hardware_gpio        # GPIO control
    hardware_timer       # Timer control
    FreeRTOS-Kernel      # FreeRTOS kernel
    FreeRTOS-Kernel-Heap4  # FreeRTOS heap
)

# Enable USB and UART output for stdio (so you can print debug messages to the serial console)
pico_enable_stdio_usb(ultrasonic_lib 1)
pico_enable_stdio_uart(ultrasonic_lib 1)

# (Optional) Add URL for project example (can be removed if not needed)
# example_auto_set_url(ultrasonic_lib)
