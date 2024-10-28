#include <stdio.h> // For printf()
#include "pico/stdlib.h" // For stdio_init_all() and sleep_ms()
#include "hardware/gpio.h" // For GPIO functions
#include "FreeRTOS.h" // For FreeRTOS API
#include "task.h"    // For FreeRTOS tasks
#include "semphr.h" // For FreeRTOS semaphores
#include "pico/time.h"  // For time functions
#include "message_buffer.h" // For Message Buffer

#define LEFT_WHEEL_ENCODER_PIN 2    // GPIO pin for the left wheel encoder
#define RIGHT_WHEEL_ENCODER_PIN 3   // GPIO pin for the right wheel encoder
#define ENCODER_NOTCHES_PER_REV 20  // Number of notches (slots) on the encoder disk
#define WHEEL_DIAMETER 0.065        // Diameter of the wheel in meters (65mm)
#define WHEEL_CIRCUMFERENCE 0.2042  // Circumference of the wheel in meters (calculated as π * Diameter)
#define DISTANCE_PER_NOTCH 0.01021  // Distance traveled per notch (Circumference / ENCODER_NOTCHES_PER_REV)
#define MICROSECONDS_IN_A_SECOND 1000000.0  // Number of microseconds in one second

// Struct to hold pulse data for each wheel
typedef struct {
    uint32_t pulse_width;
    float speed;
    float total_distance;
    uint32_t notch_count;
} PulseData_t;

// Separate data for left and right wheels
static volatile uint32_t left_notch_count = 0;
static volatile uint32_t right_notch_count = 0;
static volatile uint32_t left_last_rising_time = 0;
static volatile uint32_t right_last_rising_time = 0;
static volatile uint32_t left_last_falling_time = 0;
static volatile uint32_t right_last_falling_time = 0;
static uint32_t left_pulse_width = 0;
static uint32_t right_pulse_width = 0;

// Global flags to indicate data readiness
volatile bool left_data_ready = false;
volatile bool right_data_ready = false;

// Task handles and message buffers
TaskHandle_t leftPulseTaskHandle = NULL;
TaskHandle_t rightPulseTaskHandle = NULL;
TaskHandle_t leftSpeedTaskHandle;
TaskHandle_t rightSpeedTaskHandle;
MessageBufferHandle_t leftMessageBuffer;
MessageBufferHandle_t rightMessageBuffer;

// Interrupt callback function for left and right wheels (optimized)
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());  // Capture current time in microseconds

    if (gpio == LEFT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            left_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            left_last_falling_time = current_time;
            left_notch_count++;
            left_data_ready = true;  // Set flag to indicate data is ready for the left wheel
        }
    } else if (gpio == RIGHT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            right_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            right_last_falling_time = current_time;
            right_notch_count++;
            right_data_ready = true;  // Set flag to indicate data is ready for the right wheel
        }
    }
}

// Pulse Width Calculation Task for each wheel
void pulse_width_task(void *pvParameters) {
    uint32_t *pulse_width_ptr = (uint32_t *)pvParameters;

    while (1) {
        // Check if the left data is ready
        if (left_data_ready && pulse_width_ptr == &left_pulse_width) {
            left_data_ready = false;  // Reset the flag
            left_pulse_width = left_last_falling_time - left_last_rising_time;

            // Notify the left speed calculation task
            xTaskNotifyGive(leftSpeedTaskHandle);
        }

        // Check if the right data is ready
        if (right_data_ready && pulse_width_ptr == &right_pulse_width) {
            right_data_ready = false;  // Reset the flag
            right_pulse_width = right_last_falling_time - right_last_rising_time;

            // Notify the right speed calculation task
            xTaskNotifyGive(rightSpeedTaskHandle);
        }

        // Add a small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));  // Adjust the delay as needed
    }
}

// Speed and Distance Calculation Task for each wheel
void speed_task(void *pvParameters) {
    uint32_t *pulse_width_ptr = (uint32_t *)pvParameters;
    MessageBufferHandle_t messageBuffer = (MessageBufferHandle_t)pvParameters;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

        if (*pulse_width_ptr > 0) {
            float pulse_width_sec = *pulse_width_ptr / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;
            float total_distance;

            // Determine if it's the left or right wheel
            PulseData_t data;
            if (pulse_width_ptr == &left_pulse_width) {
                total_distance = left_notch_count * DISTANCE_PER_NOTCH;
                data.notch_count = left_notch_count;
                data.pulse_width = left_pulse_width;
            } else {
                total_distance = right_notch_count * DISTANCE_PER_NOTCH;
                data.notch_count = right_notch_count;
                data.pulse_width = right_pulse_width;
            }

            // Populate the data
            data.speed = speed;
            data.total_distance = total_distance;

            // Send data to the respective logging task via message buffer
            xMessageBufferSend(messageBuffer, &data, sizeof(data), portMAX_DELAY);
        }
    }
}

// Logging Task for each wheel
void log_task(void *pvParameters) {
    MessageBufferHandle_t messageBuffer = (MessageBufferHandle_t)pvParameters;

    while (1) {
        PulseData_t received_data;
        xMessageBufferReceive(messageBuffer, &received_data, sizeof(received_data), portMAX_DELAY);

        // Log the data
        printf("Notch Count: %u, Pulse Width: %u us, Speed: %.3f meters/second, Total Distance: %.3f meters\n",
               received_data.notch_count, received_data.pulse_width, received_data.speed, received_data.total_distance);
    }
}

int main() {
    stdio_init_all();

    // Initialize GPIO pins for left and right wheel encoders
    gpio_init(LEFT_WHEEL_ENCODER_PIN);
    gpio_set_dir(LEFT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_WHEEL_ENCODER_PIN);

    gpio_init(RIGHT_WHEEL_ENCODER_PIN);
    gpio_set_dir(RIGHT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_WHEEL_ENCODER_PIN);

    // Setup GPIO interrupts for both wheels
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Create message buffers for left and right wheels
    leftMessageBuffer = xMessageBufferCreate(256);
    rightMessageBuffer = xMessageBufferCreate(256);

    // Create tasks for left wheel
    xTaskCreate(pulse_width_task, "Left Pulse Width Task", 1024, &left_pulse_width, 1, &leftPulseTaskHandle);
    xTaskCreate(speed_task, "Left Speed Task", 1024, (void *)&leftMessageBuffer, 1, &leftSpeedTaskHandle);
    xTaskCreate(log_task, "Left Log Task", 1024, (void *)&leftMessageBuffer, 1, NULL);

    // Create tasks for right wheel
    xTaskCreate(pulse_width_task, "Right Pulse Width Task", 1024, &right_pulse_width, 1, &rightPulseTaskHandle);
    xTaskCreate(speed_task, "Right Speed Task", 1024, (void *)&rightMessageBuffer, 1, &rightSpeedTaskHandle);
    xTaskCreate(log_task, "Right Log Task", 1024, (void *)&rightMessageBuffer, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // Main loop is never reached as the FreeRTOS scheduler takes over
    }

    return 0;
}
