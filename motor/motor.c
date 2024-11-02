#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"

// Define and initialize the message buffer
MessageBufferHandle_t motorMessageBuffer;

// Define PID controller for each motor
PIDController left_pid;
PIDController right_pid;

// Define max duty cycle
#define MAX_DUTY_CYCLE 12500

// Define max speed in m/s
#define MAX_SPEED 2.0

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

void init_pid() {
    // Initialize PID controller for each motor
    pid_init(&left_pid, 0.1, 0.01, 0.01);
    pid_init(&right_pid, 0.1, 0.01, 0.01);
}

void set_left_motor_speed(uint32_t gpio, float speed) {
    // Calculate the duty cycle
    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    
    // Set the PWM level for the motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), PWM_CHAN_A, duty_cycle);
}

void set_right_motor_speed(uint32_t gpio, float speed) {
    // Calculate the duty cycle
    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    
    // Set the PWM level for the motor
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), PWM_CHAN_B, duty_cycle);
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
    set_left_motor_speed(gpioLeft, speed);
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
    set_left_motor_speed(gpioLeft, speed);
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

    PulseData_t left_data;
    PulseData_t right_data;

    while (true) {
        // Read current speed from each encoder
        //xMessageBufferReceive(leftMotorControlBuffer, &left_data, sizeof(left_data), portMAX_DELAY);
        //xMessageBufferReceive(rightMotorControlBuffer, &right_data, sizeof(right_data), portMAX_DELAY);

        // Convert speed to fraction of max speed
        float left_speed_fraction = speed_to_fraction(left_data.speed);
        float right_speed_fraction = speed_to_fraction(right_data.speed);
        float setpoint_fraction = speed_to_fraction(setpoint);

        // Compute PID output for each motor
        float left_pid_output = pid_compute(&left_pid, setpoint_fraction, left_speed_fraction); 
        float right_pid_output = pid_compute(&right_pid, setpoint_fraction, right_speed_fraction);

        // Adjust motor speed
        set_left_motor_speed(L_MOTOR_PWM_PIN, left_pid_output);
        set_right_motor_speed(R_MOTOR_PWM_PIN, right_pid_output);

        // Delay for 100 ms
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}
void motor_init_buffers() {
    motorMessageBuffer = xMessageBufferCreate(256);
    if (motorMessageBuffer == NULL) {
        printf("Failed to create motor message buffer\n");
        while (true);  // Halt if buffer creation fails
    }
}

// Task to control the motor based on obstacle detection
void motor_control_task(void *pvParameters) {
    DistanceMessage receivedMessage;

    while (true) {
        // Receive obstacle detection message (refer to ultraonic.c void ultrasonic_task(void *pvParameters))
        if (xMessageBufferReceive(motorMessageBuffer, &receivedMessage, sizeof(receivedMessage), portMAX_DELAY) > 0) {
            // Check if an obstacle is detected and take action
            if (receivedMessage.obstacleDetected) {
                printf("Obstacle detected within 10 cm! Taking action.\n");
                stop_car(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);  // Stop the car
            } else {
                // No obstacle detected, keep moving forward
                move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, 1);
            }
        }

        // Small delay to prevent excessive loop execution
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



