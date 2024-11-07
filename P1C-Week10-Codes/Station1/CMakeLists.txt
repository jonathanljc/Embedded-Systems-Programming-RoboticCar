# Car executable
add_executable(car
        car.c
)

# Include directories for car executable
target_include_directories(car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..        # for our common lwipopts
)

# for ultrasonic library
add_library(ultrasonic_lib
        ultrasonic.c
)
# for encoder library
add_library(encoder_lib
        encoder.c
)
# for motor & pid library (name as motor_lib combined)
add_library(motor_lib
        motor.c pid.c
)

# all the libraries required by the car executable
target_link_libraries(car
        pico_stdlib       # Link standard Pico library
        hardware_pwm          # Standard libraries
        hardware_gpio        # GPIO control
        hardware_timer       # Timer control
        FreeRTOS-Kernel      # FreeRTOS kernel
        FreeRTOS-Kernel-Heap4  # FreeRTOS heap
        ultrasonic_lib    # Link the ultrasonic library
        encoder_lib       # Link the encoder library
        motor_lib         # Link the motor library
        
)

# Enable USB and UART output for stdio (for debugging messages)
pico_add_extra_outputs(car)
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 1)


