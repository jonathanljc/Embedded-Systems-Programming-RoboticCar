#include "ultrasonic.h"

// Define the global message buffer
MessageBufferHandle_t printMessageBuffer;

int main()
{
    // Initialize standard I/O for debugging output
    stdio_init_all();
    sleep_ms(1000);  // Delay to ensure USB console is ready

    // Initialize the message buffer for inter-task communication
    printMessageBuffer = xMessageBufferCreate(256);  // Adjust size as needed
    if (printMessageBuffer == NULL)
    {
        printf("Failed to create message buffer\n");
        return 1;
    }

    // Initialize the ultrasonic sensor pins
    setupUltrasonicPins();

    // Initialize the Kalman filter state and create tasks
    kalman_state *state = kalman_init(2, 50, 5, 0);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, (void *)state, 1, NULL);
    xTaskCreate(print_task, "Print Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {}

    return 0;
}
