#include "pico/stdlib.h"
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
MessageBufferHandle_t leftMotorControlBuffer;
MessageBufferHandle_t rightMotorControlBuffer;

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
    // Initialize PID controllers for each motor
    pid_init(&left_pid, LEFT_KP, LEFT_KI, LEFT_KD);
    pid_init(&right_pid, RIGHT_KP, RIGHT_KI, RIGHT_KD);
}

void update_pid_constants(float left_kp, float left_ki, float left_kd, float right_kp, float right_ki, float right_kd) {
    // Update PID constants for the left motor
    left_pid.kp = left_kp;
    left_pid.ki = left_ki;
    left_pid.kd = left_kd;

    // Update PID constants for the right motor
    right_pid.kp = right_kp;
    right_pid.ki = right_ki;
    right_pid.kd = right_kd;
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
    SpeedControlParams *params = (SpeedControlParams *)pvParameters;
    float setpoint = params->setpoint;  // Desired speed in m/s

    PulseData_t left_data;
    PulseData_t right_data;
    int counter = 0; // Counter for rate-limiting speed updates

    while (true) {
        // Receive current speed data from each encoder buffer
        if (xMessageBufferReceive(leftMotorControlBuffer, &left_data, sizeof(left_data), portMAX_DELAY) > 0 &&
            xMessageBufferReceive(rightMotorControlBuffer, &right_data, sizeof(right_data), portMAX_DELAY) > 0) {
            
            counter++;
            if (counter % 5 == 0) {  // Only process every 5th message, adjust as needed
                // Convert speeds to fractions of max speed
                float left_speed_fraction = speed_to_fraction(left_data.speed);
                float right_speed_fraction = speed_to_fraction(right_data.speed);
                float setpoint_fraction = speed_to_fraction(setpoint);

                // Debugging print to verify inputs to PID
                printf("Setpoint Fraction: %.3f, Left Speed Fraction: %.3f, Right Speed Fraction: %.3f\n", 
                       setpoint_fraction, left_speed_fraction, right_speed_fraction);

                // Compute PID output based on setpoint vs. actual speed
                float left_pid_output = pid_compute(&left_pid, setpoint_fraction, left_speed_fraction);
                float right_pid_output = pid_compute(&right_pid, setpoint_fraction, right_speed_fraction);

                // Clamp PID output to 0 - 1 range for PWM
                float left_duty_cycle = fminf(fmaxf(left_pid_output, 0), 1);
                float right_duty_cycle = fminf(fmaxf(right_pid_output, 0), 1);

                // Adjust motor speeds based on PID output
                set_left_motor_speed(L_MOTOR_PWM_PIN, left_duty_cycle);
                set_right_motor_speed(R_MOTOR_PWM_PIN, right_duty_cycle);

                // Print clamped PID outputs and actual speeds for debugging
                printf("Left Speed: %.3f, Right Speed: %.3f\n", left_data.speed, right_data.speed);
                printf("Left PID Output (Duty Cycle): %.3f, Right PID Output (Duty Cycle): %.3f\n", left_duty_cycle, right_duty_cycle);

                counter = 0;  // Reset the counter
            }
        }

        // Delay between control updates
        vTaskDelay(pdMS_TO_TICKS(100));  // Adjust delay for appropriate control frequency
    }
}

void motor_init_buffers() {
    motorMessageBuffer = xMessageBufferCreate(256);
    leftMotorControlBuffer = xMessageBufferCreate(256);
    rightMotorControlBuffer = xMessageBufferCreate(256);
    if (motorMessageBuffer == NULL || leftMotorControlBuffer == NULL || rightMotorControlBuffer == NULL) {
        printf("Failed to create motor message buffers\n");
        while (true);  // Halt if buffer creation fails
    }
}


void motor_control_task(void *pvParameters) {
    DistanceMessage receivedMessage;
    bool is_turning = false;  // Flag to track if a turn is in progress
    TickType_t last_turn_time = 0;  // Timestamp for the last turn
    const TickType_t turn_cooldown = pdMS_TO_TICKS(2000); // 2-second cooldown between turns

    while (true) {
        // Receive obstacle detection message
        if (xMessageBufferReceive(motorMessageBuffer, &receivedMessage, sizeof(receivedMessage), portMAX_DELAY) > 0) {
            TickType_t current_time = xTaskGetTickCount();
            if (receivedMessage.obstacleDetected && !is_turning && (current_time - last_turn_time > turn_cooldown)) {
                printf("Obstacle detected within 10 cm! Taking action.\n");
                
                // Rotate 90 degrees to the right when an obstacle is detected
                rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                last_turn_time = current_time;
                is_turning = true;
                
                // Optionally, move forward after turning with a reduced speed
                move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, 0.5);  // Adjust speed as needed
            } else if (!receivedMessage.obstacleDetected) {
                // No obstacle detected, resume moving forward
                is_turning = false;
                move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, 1);
            }
        }

        // Small delay to prevent excessive loop execution
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



