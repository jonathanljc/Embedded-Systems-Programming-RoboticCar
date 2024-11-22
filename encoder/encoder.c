#include "encoder.h"
#include <string.h>

// Encoder variables for the left encoder
static volatile uint32_t notch_count = 0;
static uint32_t last_rising_time = 0;
static uint32_t last_falling_time = 0;
static uint32_t pulse_width = 0;

// Speed buffer for moving average
static float speed_buffer[AVERAGE_FILTER_SIZE] = {0};
static int buffer_index = 0;


// Helper function to calculate moving average
float calculate_moving_average(float buffer[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}
// Function to initialize the GPIO pin for the encoder
void init_encoder_gpio() {
    gpio_init(LEFT_WHEEL_ENCODER_PIN);       // Initialize GPIO for the encoder
    gpio_set_dir(LEFT_WHEEL_ENCODER_PIN, GPIO_IN);  // Set the pin as input
    gpio_pull_up(LEFT_WHEEL_ENCODER_PIN);    // Enable pull-up resistor
    //printf("Left encoder GPIO initialized on pin %d\n", LEFT_WHEEL_ENCODER_PIN);
}

// Polling loop for left encoder
void poll_encoder() {
    static uint32_t prev_state = 0; // Store the previous state of the pin
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    uint32_t current_state = gpio_get(LEFT_WHEEL_ENCODER_PIN); // Read current state of the encoder pin

    // Detect edge changes (rising or falling)
    if (current_state != prev_state) {
        if (current_state == 1) {  // Rising edge detected
            last_rising_time = current_time;
        } else {  // Falling edge detected
            last_falling_time = current_time;
            notch_count++; // Increment notch count
            pulse_width = last_falling_time - last_rising_time;

            if (pulse_width > MIN_PULSE_WIDTH) {
                // Calculate speed and update moving average
                float pulse_width_sec = pulse_width / MICROSECONDS_IN_A_SECOND;
                float speed = DISTANCE_PER_NOTCH / pulse_width_sec;

                speed_buffer[buffer_index] = speed;
                buffer_index = (buffer_index + 1) % AVERAGE_FILTER_SIZE;

                float average_speed = calculate_moving_average(speed_buffer, AVERAGE_FILTER_SIZE);
                float total_distance = notch_count * DISTANCE_PER_NOTCH;

                // Print speed and distance
                //printf("Left Speed: %.2f m/s, Distance: %.2f m\n", average_speed, total_distance);
                char message[50];
                sprintf(message, "Left Speed: %.2f m/s, Distance: %.2f m\n", average_speed, total_distance);
                xMessageBufferSend(wifiMessageBuffer, message, strlen(message), portMAX_DELAY);
            }
        }
        prev_state = current_state; // Update previous state
    }
}
