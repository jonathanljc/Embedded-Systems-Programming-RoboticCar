// #include "encoder.h" 
#include "ultrasonic.h"  // Include ultrasonic header 
#include "motor.h"       // Include motor header for motor control functions 
#include <stdio.h> 
#include "pico/stdlib.h" 
#include "FreeRTOS.h" 
#include "task.h" 
#include "message_buffer.h" 
#include "wifi.h"
#include "function.h"
 
// Define the task handle for ultrasonic task 
TaskHandle_t ultrasonicTaskHandle = NULL; 
TaskHandle_t wifiTaskHandle = NULL;

// Message buffer for wifi receive
MessageBufferHandle_t wifiMessageBuffer;
MessageBufferHandle_t wifiReceiveBuffer;

bool disconnectRemote = false;
char telemetryInstruction;
char telemetrySpeed;
 
int main() { 
    // Initialize standard I/O for serial output 
    stdio_init_all(); 
    sleep_ms(2000); 
 
    wifiReceiveBuffer = xMessageBufferCreate(512);
    wifiMessageBuffer = xMessageBufferCreate(512);
    
    // Initialize Kalman filter state q,r,p,x
    kalman_state *kalman = kalman_init(0.1, 1.0, 1.0, 50);  // You can adjust parameters as needed 
 
    // Create tasks for ultrasonic and logging 
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 256, kalman, 2, &ultrasonicTaskHandle); 

    // Create a task for wifi
    xTaskCreate(main_task, "Wifi Task", 256, "car", 3, &wifiTaskHandle);

     // Create Unified Task
    xTaskCreate(unified_task, "Sensor Task", 1024, NULL, 1, NULL);

    // Create a task for motor control
    // xTaskCreate(motor_control_task, "Motor Control Task", 256, NULL, 1, NULL);
 
    // Start the FreeRTOS scheduler 
    vTaskStartScheduler(); 
 
    // The main function should not reach here as FreeRTOS tasks run indefinitely 
    while (1) { 
        // Idle loop; FreeRTOS takes over task management 
    } 
 
    return 0; 
}
