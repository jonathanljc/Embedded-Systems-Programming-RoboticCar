
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"

// Define and initialize the message buffer
MessageBufferHandle_t motorMessageBuffer;


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

void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed) {
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(gpioLeft, speed - 0.08, true); // 95% speed for left motor
    set_motor_speed(gpioRight, speed, false);      // 100% speed for right motor
}

void stop_motors() {
    set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
    set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
}

// Rotate left function (small rotation)
void rotate_left(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(gpioLeft, 0.5, true);
    set_motor_speed(gpioRight, 0.7, false);
    sleep_ms(250); // Small angle rotation
    stop_motors();
}

// Rotate right function (small rotation)
void rotate_right(uint32_t gpioLeft, uint32_t gpioRight) {
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);
    set_motor_speed(gpioLeft, 0.5, true);
    set_motor_speed(gpioRight, 0.7, false);
    sleep_ms(100); // Small angle rotation
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


void motor_init_buffers() {
    motorMessageBuffer = xMessageBufferCreate(256);
    if (motorMessageBuffer == NULL) {
        printf("Failed to create motor message buffers\n");
        while (true);  // Halt if buffer creation fails
    }
}




    