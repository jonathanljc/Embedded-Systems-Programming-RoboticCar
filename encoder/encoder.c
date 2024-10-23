#include <stdio.h> // For printf()
#include "pico/stdlib.h" // For stdio_init_all() and sleep_ms()
#include "hardware/gpio.h" // For GPIO functions
#include "FreeRTOS.h" // For FreeRTOS API
#include "task.h"    // For FreeRTOS tasks
#include "semphr.h" // For FreeRTOS semaphores
#include "pico/time.h"  // For time functions
#include "message_buffer.h" // For Message Buffer

#define WHEEL_ENCODER_PIN 2         // GPIO pin for the wheel encoder   
#define ENCODER_NOTCHES_PER_REV 20  // Number of notches (slots) on the encoder disk
#define WHEEL_DIAMETER 0.065        // Diameter of the wheel in meters (65mm) according to datasheet
#define WHEEL_CIRCUMFERENCE 0.2042  // Circumference of the wheel in meters (calculated as π * Diameter)
#define DISTANCE_PER_NOTCH 0.01021  // Distance traveled per notch (Circumference / ENCODER_NOTCHES_PER_REV)
#define MICROSECONDS_IN_A_SECOND 1000000.0  // Number of microseconds in one second

// Struct to hold pulse data to be passed between tasks
typedef struct {
    uint32_t pulse_width;
    float speed;
    float total_distance;
    uint32_t notch_count;
} PulseData_t;

static volatile uint32_t notch_count = 0; // Notch count to track the number of notches
static volatile uint32_t last_rising_time = 0; // Capture the time of the rising edge
static volatile uint32_t last_falling_time = 0;  // Capture the time of the falling edge
static uint32_t pulse_width = 0;  // Pulse width calculated in the task

TaskHandle_t pulseTaskHandle = NULL; // Handle for the pulse width task
TaskHandle_t speedTaskHandle;      // Handle for the speed task
MessageBufferHandle_t messageBuffer;  // Message buffer handle

// Interrupt callback function
void gpio_callback(uint gpio, uint32_t events) { // gpio is the pin number, events is the GPIO IRQ event
    uint32_t current_time = to_us_since_boot(get_absolute_time());  // Capture current time in microseconds

    if (events & GPIO_IRQ_EDGE_RISE) { // Check if the rising edge event occurred
        last_rising_time = current_time;  // Capture rising edge timestamp
    }

    if (events & GPIO_IRQ_EDGE_FALL) { // Check if the falling edge event occurred
        last_falling_time = current_time;  // Capture falling edge timestamp

        // Increment notch count
        notch_count++; 

        // Signal task to process the pulse data
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Initial value of the flag
        vTaskNotifyGiveFromISR(pulseTaskHandle, &xHigherPriorityTaskWoken); // Notify the task from ISR
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // Yield if a higher-priority task is unblocked
    }
}

// Task 1: Pulse Width Calculation Task
void pulse_width_task(void *pvParameters) {
    while (1) {
        // Wait for notification from the ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Block until notified

        // Process the pulse data (calculate pulse width)
        pulse_width = last_falling_time - last_rising_time;

        // Notify the speed calculation task if needed
        xTaskNotifyGive(speedTaskHandle);
    }
}

// Task 2: Speed and Distance Calculation
void speed_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (pulse_width > 0) {
            float pulse_width_sec = pulse_width / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;
            float total_distance = notch_count * DISTANCE_PER_NOTCH;

            // Prepare data to send to the logging task
            PulseData_t data;
            data.pulse_width = pulse_width;
            data.speed = speed;
            data.total_distance = total_distance;
            data.notch_count = notch_count;

            // Send the structured data to the logging task via message buffer
            xMessageBufferSend(messageBuffer, &data, sizeof(data), portMAX_DELAY);
        }
    }
}

// Task 3: Logging Task
void log_task(void *pvParameters) {
    while (1) {
        // Receive data from Task 2 via message buffer
        PulseData_t received_data;
        xMessageBufferReceive(messageBuffer, &received_data, sizeof(received_data), portMAX_DELAY);

        // Log the data
        printf("Notch Count: %u, Pulse Width: %u us, Speed: %.3f meters/second, Total Distance: %.3f meters\n",
               received_data.notch_count, received_data.pulse_width, received_data.speed, received_data.total_distance);
    }
}

int main() {
    stdio_init_all();
    // Initialize the GPIO pin for the wheel encoder
    gpio_init(WHEEL_ENCODER_PIN);
    gpio_set_dir(WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(WHEEL_ENCODER_PIN);  // Use pull-up resistor
    gpio_set_irq_enabled_with_callback(WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Create the message buffer for synchronization
    messageBuffer = xMessageBufferCreate(256);  // Create a message buffer of size 256 bytes

    // Create the FreeRTOS tasks
    xTaskCreate(pulse_width_task, "Pulse Width Task", 1024, NULL, 1, &pulseTaskHandle);
    xTaskCreate(speed_task, "Speed Task", 1024, NULL, 1, &speedTaskHandle);
    xTaskCreate(log_task, "Log Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // Main loop is never reached as the FreeRTOS scheduler takes over
    }

    return 0;
}
