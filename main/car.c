#include "encoder.h" 
#include "ultrasonic.h"  // Include ultrasonic header 
#include "motor.h"       // Include motor header for motor control functions 
#include <stdio.h> 
#include "pico/stdlib.h" 
#include "FreeRTOS.h" 
#include "task.h" 
#include "message_buffer.h" 
 
// Define the task handle for ultrasonic task 
TaskHandle_t ultrasonicTaskHandle = NULL; 
 
int main() { 
    // Initialize standard I/O for serial output 
    stdio_init_all(); 
    sleep_ms(2000); 
 
    // Initialize Kalman filter state 
    kalman_state *kalman = kalman_init(0.1, 1.0, 1.0, 50);  // You can adjust parameters as needed 
 
    // Create tasks for ultrasonic and logging 
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, kalman, 1, &ultrasonicTaskHandle); 
 
    // Start the FreeRTOS scheduler 
    vTaskStartScheduler(); 
 
    // The main function should not reach here as FreeRTOS tasks run indefinitely 
    while (1) { 
        // Idle loop; FreeRTOS takes over task management 
    } 
 
    return 0; 
}