#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h" // Include motor control functions

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Wait for UART to start

    // Initialize message buffers
    ultrasonic_init_buffers();
    motor_init_buffers();

    // Setup motor pins and initialize PWM for motor control
    init_motor_pins();
    sleep_ms(2000);
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    sleep_ms(2000);

    init_pid();  // Initialize PID controller if required
    sleep_ms(2000);

    // Setup pins for ultrasonic and encoder
    setup_pins_with_unified_callback();
    sleep_ms(2000);
    
    // Initialize encoder tasks (called from encoder.c)
    encoder_init();
    sleep_ms(2000);

    // Initialize the Kalman filter state for ultrasonic
    kalman_state *state = kalman_init(2, 50, 5, 0);

    // Create FreeRTOS tasks for ultrasonic, printing, and motor control
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, state, 2, NULL);
    xTaskCreate(print_task, "Print Task", 256, NULL, 3, NULL);
    xTaskCreate(motor_control_task, "Motor Control Task", 256, NULL, 3, NULL);

    // Start FreeRTOS scheduler to handle the tasks
    vTaskStartScheduler();

    while(true);

}
