#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h" // For time functions

#define WHEEL_ENCODER_PIN 2
#define ENCODER_NOTCHES_PER_REV 20        // Number of notches (slots) on the encoder disk
#define WHEEL_DIAMETER 0.065              // Diameter of the wheel in meters (65mm) according to datasheet
#define WHEEL_CIRCUMFERENCE 0.2042        // Circumference of the wheel in meters (calculated as π * Diameter)
#define DISTANCE_PER_NOTCH 0.01021        // Distance traveled per notch (Circumference / ENCODER_NOTCHES_PER_REV)
#define MIN_PULSE_TIME_US 500             // Minimum pulse width in microseconds to filter noise (debouncing)
#define NO_PULSE_TIMEOUT_MS 1000          // Timeout for detecting zero speed

static volatile uint32_t notch_count = 0;
static volatile bool pulse_width_ready = false;  // Flag to indicate new pulse width data
static float speed = 0.0;
static float total_distance = 0.0;  // To keep track of total distance traveled
static volatile uint32_t last_rising_time = 0;
static volatile uint32_t pulse_width = 0;  // Pulse width in microseconds
static uint32_t last_speed_time = 0;  // To track the last time speed was updated

// Interrupt callback function (minimized work)
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());  // Capture the current time in microseconds

    if (events & GPIO_IRQ_EDGE_RISE) {
        notch_count++;  // Increment the notch count

        // Calculate pulse width only if last rising edge time is available and debounce the noise
        if (last_rising_time != 0 && (current_time - last_rising_time) > MIN_PULSE_TIME_US) {
            pulse_width = current_time - last_rising_time;
            pulse_width_ready = true;  // Set flag to handle in main loop
        }

        last_rising_time = current_time;  // Update the last rising time
        total_distance += DISTANCE_PER_NOTCH;  // Update distance immediately after each pulse
        last_speed_time = to_ms_since_boot(get_absolute_time());  // Track the last time speed was updated
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

// Function to check for no new pulses and set speed to 0 if no pulses are detected for a while
void check_for_no_pulse() {
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

    // If more than NO_PULSE_TIMEOUT_MS has passed since the last speed update, set speed to 0
    if (current_time_ms - last_speed_time > NO_PULSE_TIMEOUT_MS) {
        if (speed != 0.0) {  // Only print if speed was not already zero
            speed = 0.0;
            printf("No pulse detected for a while, setting speed to 0.\n");
        }
    }
}

int main() {
    stdio_init_all();

    printf("Wheel Encoder with Distance, Speed, and Pulse Width Calculation\n");

    // Initialize the GPIO pin for the wheel encoder
    gpio_init(WHEEL_ENCODER_PIN);
    gpio_set_dir(WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(WHEEL_ENCODER_PIN);  // or gpio_pull_down(WHEEL_ENCODER_PIN) depending on your setup
    gpio_set_irq_enabled_with_callback(WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);  // Only use rising edge

    // Main loop
    while (1) {
        // Check if new pulse width data is ready
        if (pulse_width_ready) {
            printf("Pulse Width: %u us\n", pulse_width);
            pulse_width_ready = false;  // Reset the flag after printing

            // Calculate speed based on pulse width
            calculate_speed_from_pulse_width();
        }

        // Check for no pulses and update speed to 0 if necessary
        check_for_no_pulse();

        // Print notch count, speed, and total distance traveled
        printf("Notch Count: %u, Speed: %.3f meters/second, Total Distance: %.3f meters\n", 
               notch_count, speed, total_distance);

        sleep_ms(100);  // Small delay for CPU to handle other tasks
    }

    return 0;
}
