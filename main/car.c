#include "motor.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"

void motor_control_task(void *pvParameters) {
    move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, 0.3);

   while (true) {
        if (obstacleDetected) {
            stop_car(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            obstacleDetected = false;
        } else {
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
   }
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    init_motor_pins();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    setupUltrasonicPins();
    // encoder_init();
    init_pid();

    SpeedControlParams speed = {
        .setpoint = 0.48
    };

    // Initialize the Kalman filter state
    kalman_state *state = kalman_init(2, 50, 5, 0);

    // Create the message buffer for printing data
    printMessageBuffer = xMessageBufferCreate(256);

    // Initialize the encoder system
    //encoder_init();

    // Create the ultrasonic task
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, state, 2, NULL);
    // Create the motor control task
    xTaskCreate(motor_control_task, "Motor Control Task", 256, NULL, 1, NULL);
    //xTaskCreate(control_speed_task, "Motor Control Task", 256, &speed, 1, NULL);
    // Create the print task
    xTaskCreate(print_task, "Print Task", 256, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true);  // This should never be reached
}

