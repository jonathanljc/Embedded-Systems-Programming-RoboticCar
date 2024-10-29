#include "motor.h"    // Include motor header to access motor functions
#include "pico/stdlib.h"

int main() {
    // Initialize standard I/O for debugging
    stdio_init_all();
    
    init_motor_pins();

    // Set up PWM for both motors
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    // Move the car forward
    move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    // Keep moving forward indefinitely
    while (true) {
        // Add a small delay to keep the loop running
        sleep_ms(100);
    }

    return 0;
}




