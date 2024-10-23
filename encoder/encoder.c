#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/time.h"  // For time functions

#define WHEEL_ENCODER_PIN 2
#define ENCODER_NOTCHES_PER_REV 20  // Number of notches (slots) on the encoder disk
#define WHEEL_DIAMETER 0.065        // Diameter of the wheel in meters (65mm) according to datasheet
#define WHEEL_CIRCUMFERENCE 0.2042  // Circumference of the wheel in meters (calculated as π * Diameter)
#define DISTANCE_PER_NOTCH 0.01021  // Distance traveled per notch (Circumference / ENCODER_NOTCHES_PER_REV)
#define NO_PULSE_TIMEOUT_MS 1000    // Timeout for detecting zero speed

static volatile uint32_t notch_count = 0;
static volatile bool pulse_width_ready = false;  // Flag to indicate new pulse width data
static float speed = 0.0;
static float total_distance = 0.0;  // To keep track of total distance traveled
static volatile uint32_t last_rising_time = 0;
static volatile uint32_t pulse_width = 0;  // Pulse width in microseconds
static uint32_t last_speed_time = 0;  // To track the last time speed was updated

SemaphoreHandle_t pulseSemaphore;  // Semaphore to sync interrupt and task

// Interrupt callback function with diagnostics
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());  // Capture the current time in microseconds

    if (events & GPIO_IRQ_EDGE_RISE) {
        // Rising edge: notch starting to pass
        last_rising_time = current_time;
    }

    if (events & GPIO_IRQ_EDGE_FALL) {
        // Falling edge: notch has passed, calculate pulse width
        pulse_width = current_time - last_rising_time;
        pulse_width_ready = true;  // Set flag to handle in the task

        // Increment notch count
        notch_count++;
        total_distance += DISTANCE_PER_NOTCH;

        // Signal task to process the pulse
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(pulseSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  // Yield if a higher-priority task is unblocked
    }
}

// Function to calculate speed based on pulse width
void calculate_speed_from_pulse_width() {
    if (pulse_width > 0) {
        // Convert pulse width from microseconds to seconds
        float pulse_width_sec = pulse_width / 1000000.0;  // Convert microseconds to seconds

        // Calculate speed based on pulse width
        speed = DISTANCE_PER_NOTCH / pulse_width_sec;

        // Print speed
        printf("Calculated Speed: %.3f meters/second\n", speed);
    }
}

// Task to handle the pulse width calculations
void pulse_task(void *pvParameters) {
    while (1) {
        // Wait until the pulse width is ready
        if (xSemaphoreTake(pulseSemaphore, portMAX_DELAY) == pdTRUE) {
            if (pulse_width_ready) {
                pulse_width_ready = false;  // Reset the flag

                // Calculate speed based on pulse width
                calculate_speed_from_pulse_width();

                // Print pulse width, notch count, speed, and total distance traveled
                printf("Pulse Width: %u us, Notch Count: %u, Speed: %.3f meters/second, Total Distance: %.3f meters\n",
                       pulse_width, notch_count, speed, total_distance);
            }
        }

        //vTaskDelay(pdMS_TO_TICKS(50));  // Delay for 50ms between checks
    }
}

int main() {
    stdio_init_all();
    printf("Wheel Encoder with Distance, Speed, and Pulse Width Calculation\n");

    // Initialize the GPIO pin for the wheel encoder
    gpio_init(WHEEL_ENCODER_PIN);
    gpio_set_dir(WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(WHEEL_ENCODER_PIN);  // Use pull-up resistor
    gpio_set_irq_enabled_with_callback(WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Create the semaphore for synchronization between ISR and task
    pulseSemaphore = xSemaphoreCreateBinary();
    if (pulseSemaphore == NULL) {
        printf("Failed to create the semaphore.\n");
        return 1;  // Exit if semaphore creation failed
    }

    // Create the FreeRTOS task for processing encoder pulses
    xTaskCreate(pulse_task, "Pulse Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1) {
        // Main loop is never reached as the FreeRTOS scheduler takes over
    }

    return 0;
}
