#include "pico/stdlib.h" 
#include <stdio.h> 
#include "encoder.h"    // Include encoder header for encoder functions
#include "ultrasonic.h"  // Include ultrasonic header 
#include "motor.h"       // Include motor header for motor control functions 


 
// Define the task handle for ultrasonic task 
TaskHandle_t ultrasonicTaskHandle = NULL; 
 
int main() { 
    // Initialize standard I/O for serial output 
    stdio_init_all(); 
    sleep_ms(2000); 
 
    // Initialize Kalman filter state 
    kalman_state *kalman = kalman_init(0.1, 1.0, 1.0, 50);  // kalman_init(q, r, p, initial_value)
 
    // Create tasks for station 1 (logic located in ultrasonic.c)
    // make use of pid calibration via encoder feedback for motor control 
    // then use ultrasonic sensor to detect obstacles
    // rotate right and move 90cm based on notches detected (88 notches in this case equals 90cm)
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, kalman, 1, &ultrasonicTaskHandle); 
 
    // Start the FreeRTOS scheduler 
    vTaskStartScheduler(); 
 
    // The main function should not reach here as FreeRTOS tasks run indefinitely 
    while (1) { 
        // Idle loop; FreeRTOS takes over task management 
    } 
 
    return 0; 
}