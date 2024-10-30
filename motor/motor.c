#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pid.h"
#include "motor.h"

// Define PID controller
PIDController pid;

// Define max duty cycle
#define MAX_DUTY_CYCLE 12500

// Function to set up the PWM
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set up for left motor

    // Set the GPIO function to PWM
    gpio_set_function(gpioLeft, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint32_t slice_num_left = pwm_gpio_to_slice_num(gpioLeft);

    // Set the PWM clock divider
    pwm_set_clkdiv(slice_num_left, 100);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num_left, 12500);  

    // Enable the PWM
    pwm_set_enabled(slice_num_left, true);

    // Ensure car starts at a stop
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, 0);

    // Set up for right motor

    // Set the GPIO function to PWM
    gpio_set_function(gpioRight, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint32_t slice_num_right = pwm_gpio_to_slice_num(gpioRight);

    // Set the PWM clock divider
    pwm_set_clkdiv(slice_num_right, 100);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num_right, 12500);  

    // Enable the PWM
    pwm_set_enabled(slice_num_right, true);

    // Ensure car starts at a stop
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);
}

void init_motor_pins() {
    // Initialize the GPIO pins for directional control
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);

    // Set the GPIO pins to output
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN2, GPIO_OUT);
}

void set_speed30(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 0.3 * MAX_DUTY_CYCLE);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 0.3 * MAX_DUTY_CYCLE);
}

void set_speed50(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 0.5 * MAX_DUTY_CYCLE);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 0.5 * MAX_DUTY_CYCLE);
}

void set_speed100(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, MAX_DUTY_CYCLE);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, MAX_DUTY_CYCLE);
}

void move_forward(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);

    // Set speed   
    set_speed50(gpioLeft, gpioRight);
}

void move_backward(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);

    // Set speed
    set_speed50(gpioLeft, gpioRight);
}

void rotate_left(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);

    // Set speed
    set_speed30(gpioLeft, gpioRight);

    // Rotate for 1 second
    sleep_ms(1000);

    // Stop the car
    stop_car(gpioLeft, gpioRight);
}

void rotate_right(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);

    // Set speed
    set_speed30(gpioLeft, gpioRight);

    // Rotate for 1 second
    sleep_ms(1000);

    // Stop the car
    stop_car(gpioLeft, gpioRight);
}

void stop_car(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 0);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 0);
}




