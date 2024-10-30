#include "wifi.h"

// Define the global message buffer
MessageBufferHandle_t wifiMessageBuffer;

int main()
{
    // Initialize standard I/O for debugging output
    stdio_init_all();
    sleep_ms(5000); // Delay to ensure USB console is ready

    wifiMessageBuffer = xMessageBufferCreate(1024);

    xTaskCreate(main_task, "TestMainThread", 256, "dashboard", 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {}

    return 0;
}