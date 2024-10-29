add_library(motor_lib
        motor.c
        pid.c
        )

# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more)
target_link_libraries(motor_lib
    pico_stdlib
    hardware_pwm          # Standard libraries
    hardware_gpio        # GPIO control
    hardware_timer       # Timer control
    FreeRTOS-Kernel      # FreeRTOS kernel
    FreeRTOS-Kernel-Heap4  # FreeRTOS heap
)

# create map/bin/hex file etc.
#pico_add_extra_outputs(motor)

