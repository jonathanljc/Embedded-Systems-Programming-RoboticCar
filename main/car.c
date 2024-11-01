#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h" // Include motor control functions

// Define speed in fraction of max speed (0.5 represents 50% of MAX_SPEED)
#define FORWARD_SPEED 0.5

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Wait for UART to start

    // Setup motor pins and initialize PWM for motor control
    init_motor_pins();
    sleep_ms(2000);
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(2000);
    init_pid();  // Initialize PID controller if required
    sleep_ms(2000);
    // Move the car forward at specified speed
    move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, FORWARD_SPEED);
    sleep_ms(2000);
    // Setup pins for ultrasonic and encoder
    setup_pins_with_unified_callback();
    sleep_ms(2000);
    
    // Initialize the Kalman filter state for ultrasonic
    kalman_state *state = kalman_init(2, 50, 5, 0);

    // Initialize encoder tasks (called from encoder.c)
    encoder_init();

    // Create FreeRTOS tasks for ultrasonic and printing
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, state, 1, NULL);
    xTaskCreate(print_task, "Print Task", 256, NULL, 1, NULL);

    // Start FreeRTOS scheduler to handle the tasks
    vTaskStartScheduler();

    while (true);  // This should never be reached
}
