#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "ultrasonic.h"
#include "encoder.h"


int main() {
    stdio_init_all();

    // Setup pins for ultrasonic and encoder
    setup_pins_with_unified_callback();
    

    // Initialize the Kalman filter state for ultrasonic
    kalman_state *state = kalman_init(2, 50, 5, 0);

    // Initialize encoder tasks (called from encoder.c)
    encoder_init();

    // Create remaining FreeRTOS tasks
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, state, 1, NULL);
    xTaskCreate(print_task, "Print Task", 256, NULL, 1, NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    while (true);  // This should never be reached
}



