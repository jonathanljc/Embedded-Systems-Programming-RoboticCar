#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pid.h"

// Define GPIO pins
#define L_MOTOR_PWM_PIN 0     // GP2 for PWM
#define L_MOTOR_DIR_PIN1 2    // GP0 for direction
#define L_MOTOR_DIR_PIN2 3    // GP1 for direction

#define R_MOTOR_PWM_PIN 1     // GP2 for PWM
#define R_MOTOR_DIR_PIN1 4    // GP0 for direction
#define R_MOTOR_DIR_PIN2 5    // GP1 for direction

// Define PID controller
PIDController pid;

// Function to set up the PWM
void setup_pwm(uint gpioLeft, uint gpioRight) {
    // Set up for left motor

    // Set the GPIO function to PWM
    gpio_set_function(gpioLeft, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num_left = pwm_gpio_to_slice_num(gpioLeft);

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
    uint slice_num_right = pwm_gpio_to_slice_num(gpioRight);

    // Set the PWM clock divider
    pwm_set_clkdiv(slice_num_right, 100);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num_right, 12500);  

    // Enable the PWM
    pwm_set_enabled(slice_num_right, true);

    // Ensure car starts at a stop
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);
}

void move_forward(uint gpioLeft, uint gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);

    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 12500/2);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 12500/2);
}

void move_backward(uint gpioLeft, uint gpioRight) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);

    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 12500/2);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 12500/2);
}

void stop_car(uint gpioLeft, uint gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 0);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 0);
}

void set_speed1(uint gpioLeft, uint gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 12500/4);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 12500/4);
}

void set_speed2(uint gpioLeft, uint gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 12500/2);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 12500/2);
}

void set_speed3(uint gpioLeft, uint gpioRight) {
    // Set the PWM level for the left motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioLeft), PWM_CHAN_A, 12500);

    // Set the PWM level for the right motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpioRight), PWM_CHAN_B, 12500);
}

int main (){
    // Initialize the GPIO pins for diretional control
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);

    // Set the GPIO pins to output
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN2, GPIO_OUT);

    // Set up PWM
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    // Initialize PID controller
    pid_init(&pid, 1.0, 0.1, 0.01);

    // Function Calls
    move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(5000);
    set_speed1(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(5000);
    set_speed3(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(5000);
    move_backward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(5000);
    stop_car(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    return 0;
}