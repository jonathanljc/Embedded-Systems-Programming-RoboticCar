# Define the executable for the ultrasonic sensor program
add_executable(ultrasonic
    ultrasonic.c
)

# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more)
target_link_libraries(ultrasonic pico_stdlib hardware_gpio hardware_timer)

# Create map/bin/hex/uf2 files for the program
pico_add_extra_outputs(ultrasonic)

# Enable USB and UART output for stdio
pico_enable_stdio_usb(ultrasonic 1)
pico_enable_stdio_uart(ultrasonic 1)

# Add URL for project example (optional, can be removed if not needed)
# example_auto_set_url(ultrasonic)
