
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"
#include <string.h>

// Define and initialize the message buffer
MessageBufferHandle_t motorMessageBuffer;

// Define the PID controllers for the left and right motors
PIDController left_pid_controller;
PIDController right_pid_controller;


// Function to set up the PWM
// Motor Control Code
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_set_function(gpioLeft, GPIO_FUNC_PWM);
    uint32_t slice_num_left = pwm_gpio_to_slice_num(gpioLeft);
    pwm_set_clkdiv(slice_num_left, 100);
    pwm_set_wrap(slice_num_left, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_left, true);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, 0);

    gpio_set_function(gpioRight, GPIO_FUNC_PWM);
    uint32_t slice_num_right = pwm_gpio_to_slice_num(gpioRight);
    pwm_set_clkdiv(slice_num_right, 100);
    pwm_set_wrap(slice_num_right, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_right, true);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);
}

void init_motor_pins() {
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN2, GPIO_OUT);
}

void set_motor_speed(uint32_t gpio, float speed, bool is_left) {
    if (speed > 1.0) speed = 1.0;
    if (speed < 0.0) speed = 0.0;

    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), is_left ? PWM_CHAN_A : PWM_CHAN_B, duty_cycle);
}

void set_speed40(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 0.45, true);
    set_motor_speed(gpioRight, 0.45, false);
}

void set_speed50(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 0.6, true);
    set_motor_speed(gpioRight, 0.6*0.95, false);
}

void speed50(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 0.5, true);
    set_motor_speed(gpioRight, 0.5 * 0.93, false);
}

void speed100(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 1.0, true);
    set_motor_speed(gpioRight, 1.0 * 0.95, false);
}

void set_speed70(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 0.7, true);
    set_motor_speed(gpioRight, 0.7 * 0.90, false);
}


void set_speed100(uint32_t gpioLeft, uint32_t gpioRight) {
    set_motor_speed(gpioLeft, 1.0, true);
    set_motor_speed(gpioRight, 1.0, false);
}

void move_forward(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    //printf("FORWARD DONE\n");
}

void move_backward(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);
    //printf("BACKWARD DONE\n");
}

void stop_motors() {
    set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
    set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
}

// Rotate left function
void rotate_left(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    //printf("LEFT DONE\n");
}

// Rotate right function 
void rotate_right(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);
    //printf("RIGHT DONE\n");
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

void init_pid(){
    pid_init(&left_pid_controller, 1.0, 0.1, 0.1);
    pid_init(&right_pid_controller, 1.0, 0.1, 0.1);
}

void adjust_motor_speed(float left_speed, float right_speed,float setpoint, float dt) {
    float left_pid = pid_compute(&left_pid_controller, setpoint, left_speed, dt);
    float right_pid = pid_compute(&right_pid_controller, setpoint, right_speed, dt);
    printf("Left PID: %.2f, Right PID: %.2f\n", left_pid, right_pid);
    set_left_motor_speed(L_MOTOR_PWM_PIN, left_pid);
    set_right_motor_speed(R_MOTOR_PWM_PIN, right_pid);
}

void motor_init_buffers() {
    motorMessageBuffer = xMessageBufferCreate(256);
    if (motorMessageBuffer == NULL) {
        printf("Failed to create motor message buffers\n");
        while (true);  // Halt if buffer creation fails
    }
}

void motor_control_task(void *pvParameters) {
    absolute_time_t previous_time = get_absolute_time();
    absolute_time_t current_time;
    float dt = 0.0;
    float setpoint = 0.0;
    float left_average_speed = 0.0;
    float left_percentage_speed = 0.0;
    float right_average_speed = 0.0;
    float right_percentage_speed = 0.0;
    char speed[100];

    init_encoder_gpio();
    init_pid();
    init_motor_pins();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    motor_init_buffers();

    while (true) {
        // Poll the encoders
        poll_encoder(&left_encoder, LEFT_WHEEL_ENCODER_PIN);  // Poll the left encoder
        poll_encoder(&right_encoder, RIGHT_WHEEL_ENCODER_PIN);

        if (xMessageBufferReceive(motorMessageBuffer, &speed, sizeof(speed), 0) > 0) {
            setpoint = atof(speed);
        }

        current_time = get_absolute_time();
        dt = absolute_time_diff_us(previous_time, current_time) / 1000000;  // in seconds
        previous_time = current_time;

        left_percentage_speed = left_average_speed / MAX_SPEED;
        right_percentage_speed = right_average_speed / MAX_SPEED;

        adjust_motor_speed(left_percentage_speed, right_percentage_speed, setpoint, dt);

        // Delay for a short time to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust the delay as necessary
    }
}
    
