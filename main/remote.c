#include "wifi.h"
#include "accelerometer.h"

// Define the global message buffer
MessageBufferHandle_t wifiMessageBuffer;
MessageBufferHandle_t wifiReceiveBuffer;

bool disconnectRemote = false;

int main()
{
    // Initialize standard I/O for debugging output
    stdio_init_all();
    sleep_ms(3000); // Delay to ensure USB console is ready
    
    wifiMessageBuffer = xMessageBufferCreate(512);

    xTaskCreate(magnetometer_task, "MagnetometerTask", 256, NULL, 1, NULL);
    xTaskCreate(main_task, "WifiTask", 256, "remote", 2, NULL);
    if (wifiMessageBuffer == NULL)
    {
        printf("Failed to create message buffer\n");
        return 1;
    }

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true)
    {
    }

    return 0;
}