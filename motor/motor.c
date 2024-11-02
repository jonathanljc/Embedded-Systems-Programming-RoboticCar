#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>

// Define PID controller for each motor
PIDController left_pid;
PIDController right_pid;

// Define max duty cycle
#define MAX_DUTY_CYCLE 12500

// Define max speed in m/s
#define MAX_SPEED 0.48

// Define the task handle
TaskHandle_t controlSpeedTaskHandle;

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
    pwm_set_chan_level(slice_num_left, PWM_CHAN_B, 0);

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
    pwm_set_chan_level(slice_num_right, PWM_CHAN_A, 0);
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

void init_pid() {
    // Initialize PID controller for each motor
    pid_init(&left_pid, 0.5, 0.05, 0.005);
    pid_init(&right_pid, 0.5, 0.05, 0.005);
}

void set_left_motor_speed(uint32_t gpio, float speed) {
    // Calculate the duty cycle
    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    
    // Set the PWM level for the motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), PWM_CHAN_B, duty_cycle);
}

void set_right_motor_speed(uint32_t gpio, float speed) {
    // Calculate the duty cycle
    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    
    // Set the PWM level for the motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), PWM_CHAN_A, duty_cycle);
}

void set_speed30(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set speed for each motor
    set_left_motor_speed(gpioLeft, 0.3);
    set_right_motor_speed(gpioRight, 0.3);
}

void set_speed50(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set speed for each motor
    set_left_motor_speed(gpioLeft, 0.5);
    set_right_motor_speed(gpioRight, 0.5);
}

void set_speed100(uint32_t gpioLeft, uint32_t gpioRight) {
    // Set speed for each motor
    set_left_motor_speed(gpioLeft, 1);
    set_right_motor_speed(gpioRight, 1);
}

void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);

    // Set speed for each motor
    set_left_motor_speed(gpioLeft, speed - 0.148);
    set_right_motor_speed(gpioRight, speed);
}

void move_backward(uint32_t gpioLeft, uint32_t gpioRight, float speed) {
    // Set the direction of the left motor
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);

    // Set the direction of the right motor
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);

    // Set speed for each motor
    set_left_motor_speed(gpioLeft, speed - 0.148);
    set_right_motor_speed(gpioRight, speed);
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
    sleep_ms(750);

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
    set_speed50(gpioLeft, gpioRight);

    // Rotate for 1 second
    sleep_ms(1000);

    // Stop the car
    stop_car(gpioLeft, gpioRight);
}

void stop_car(uint32_t gpioLeft, uint32_t gpioRight) {

    // Set the speed for each motor
    set_left_motor_speed(gpioLeft, 0);
    set_right_motor_speed(gpioRight, 0);
}

float speed_to_fraction(float speed) {
    return speed / MAX_SPEED;
}

void control_speed_task(void *pvParameters) {
    // Cast the parameters to the SpeedControlParams struct
    SpeedControlParams *params = (SpeedControlParams *)pvParameters;
    float setpoint = params->setpoint;
    printf("Setpoint: %f\n", setpoint);
    float setpoint_fraction = speed_to_fraction(setpoint);

    // Move car forward at setpoint speed
    move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, setpoint);

    PulseData_t left_data;
    PulseData_t right_data;

    while (true) {
        /*if (obstacleDetected){
            stop_car(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
        }
        else{ */
            // Read current speed from each encoder
            if(xMessageBufferReceive(leftMotorControlBuffer, &left_data, sizeof(left_data), pdMS_TO_TICKS(10)) == sizeof(left_data) &&
            xMessageBufferReceive(rightMotorControlBuffer, &right_data, sizeof(right_data), pdMS_TO_TICKS(10)) == sizeof(right_data)){

                // Convert speed to fraction of max speed
                float left_speed_fraction = speed_to_fraction(left_data.speed);
                float right_speed_fraction = speed_to_fraction(right_data.speed);
                printf("Left speed fraction: %f\n", left_speed_fraction);
                printf("Right speed fraction: %f\n", right_speed_fraction);

                // Compute PID output for each motor
                float left_pid_output = pid_compute(&left_pid, setpoint_fraction, left_speed_fraction); 
                float right_pid_output = pid_compute(&right_pid, setpoint_fraction, right_speed_fraction);
                printf("Left PID output: %f\n", left_pid_output);
                printf("Right PID output: %f\n", right_pid_output);

                // Adjust motor speed
                set_left_motor_speed(L_MOTOR_PWM_PIN, left_pid_output);
                set_right_motor_speed(R_MOTOR_PWM_PIN, right_pid_output);
            }
        
            // Delay for 100 ms
            vTaskDelay(pdMS_TO_TICKS(100)); 
        
    }
}


