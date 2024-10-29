#include "MQTTClient.h"

int main()
{
    // Initialize standard I/O for debugging output
    stdio_init_all();
    sleep_ms(5000); // Delay to ensure USB console is ready

    xTaskCreate(main_task, "TestMainThread", 256, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {}

    return 0;
}