# Add executable and source files
add_executable(labtest
    main.c
    functions.c
)

# Link with the Pico standard library and hardware PWM
target_link_libraries(labtest pico_stdlib hardware_pwm)

# Enable USB serial output
pico_enable_stdio_usb(labtest 1)
# Enable UART for serial monitor output (optional)
pico_enable_stdio_uart(labtest 1)

# Create the output files (e.g., .uf2 file for flashing)
pico_add_extra_outputs(labtest)
