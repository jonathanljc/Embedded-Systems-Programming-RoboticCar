add_executable(motor
        motor.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(motor pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(motor)

