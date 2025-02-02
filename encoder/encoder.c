#include "encoder.h"
#include <string.h>

// Define encoder states
Encoder left_encoder = {0};  // Left encoder instance
Encoder right_encoder = {0}; // Right encoder instance
float left_average_speed = 0.0;
float left_total_distance = 0.0;
float right_average_speed = 0.0;
float right_total_distance = 0.0;


// Helper function to calculate moving average
float calculate_moving_average(float buffer[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

// Function to initialize GPIO pins for both encoders
void init_encoder_gpio() {
    // Initialize left encoder GPIO
    gpio_init(LEFT_WHEEL_ENCODER_PIN);
    gpio_set_dir(LEFT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_WHEEL_ENCODER_PIN);

    // Initialize right encoder GPIO
    gpio_init(RIGHT_WHEEL_ENCODER_PIN);
    gpio_set_dir(RIGHT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_WHEEL_ENCODER_PIN);
}

// Generalized polling function for an encoder
void poll_encoder(Encoder *encoder, uint32_t gpio_pin) {
    static uint32_t last_edge_time[2] = {0};  // Separate debounce time for each encoder
    static uint32_t prev_state[2] = {0};     // Store previous state for each encoder

    uint32_t current_time = to_us_since_boot(get_absolute_time());
    uint32_t current_state = gpio_get(gpio_pin);  // Read the GPIO state

    // Determine index based on the GPIO pin
    int index = (gpio_pin == LEFT_WHEEL_ENCODER_PIN) ? 0 : 1;

    // Check for inactivity timeout
    if (current_time - last_edge_time[index] > ENCODER_INACTIVITY_TIMEOUT) {
        // Reset encoder readings
        encoder->notch_count = 0;
        memset(encoder->speed_buffer, 0, sizeof(encoder->speed_buffer));
        encoder->buffer_index = 0;

        // Update global variables
        if (gpio_pin == LEFT_WHEEL_ENCODER_PIN) {
            left_average_speed = 0.0;
            left_total_distance = 0.0;
        } else if (gpio_pin == RIGHT_WHEEL_ENCODER_PIN) {
            right_average_speed = 0.0;
            right_total_distance = 0.0;
        }

        // Send reset message to Wi-Fi
        char reset_message[100];
        snprintf(reset_message, 100, "Idle | Left: 0.00 m/s, %.2f m | Right: 0.00 m/s, %.2f m\n",
                left_total_distance, right_total_distance);
        xMessageBufferSend(wifiMessageBuffer, reset_message, strlen(reset_message) + 1, portMAX_DELAY);
        

        // Update last edge time to prevent repetitive resets
        last_edge_time[index] = current_time;
        return; // Skip further processing for this cycle
    }

    // Detect edge changes (rising or falling)
    if (current_state != prev_state[index]) {
        uint32_t time_since_last_edge = current_time - last_edge_time[index];

        // Ignore edges occurring within debounce threshold
        if (time_since_last_edge < DEBOUNCE_THRESHOLD) {
            prev_state[index] = current_state;  // Update the state, but ignore the edge
            return;
        }

        if (current_state == 1) {  // Rising edge detected
            encoder->last_rising_time = current_time;
        } else {  // Falling edge detected
            encoder->last_falling_time = current_time;
            encoder->pulse_width = encoder->last_falling_time - encoder->last_rising_time;

            if (encoder->pulse_width > MIN_PULSE_WIDTH) {
                float pulse_width_sec = encoder->pulse_width / MICROSECONDS_IN_A_SECOND;

                // Only calculate speed if pulse_width_sec > 0
                if (pulse_width_sec > 0) {
                    encoder->notch_count++;  // Increment notch count

                    // Calculate speed and update moving average
                    float speed = DISTANCE_PER_NOTCH / pulse_width_sec;

                    encoder->speed_buffer[encoder->buffer_index] = speed;
                    encoder->buffer_index = (encoder->buffer_index + 1) % AVERAGE_FILTER_SIZE;

                    float average_speed = calculate_moving_average(encoder->speed_buffer, AVERAGE_FILTER_SIZE);
                    float total_distance = encoder->notch_count * DISTANCE_PER_NOTCH;

                    // Update global variables
                    if (gpio_pin == LEFT_WHEEL_ENCODER_PIN) {
                        left_average_speed = average_speed;
                        left_total_distance = total_distance;
                    } else if (gpio_pin == RIGHT_WHEEL_ENCODER_PIN) {
                        right_average_speed = average_speed;
                        right_total_distance = total_distance;
                    }

                    // Send the message to Wi-Fi
                    char combined_message[100];
                    if(telemetryInstruction == 'f' && telemetrySpeed == '0'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Forward at 100%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'f'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Forward at %c0%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            telemetrySpeed, left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'b' && telemetrySpeed == '0'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Backward at 100%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'b'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Backward at %c0%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            telemetrySpeed, left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'l' && telemetrySpeed == '0'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Left at 100%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'l'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Left at %c0%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            telemetrySpeed, left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'r' && telemetrySpeed == '0'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Right at 100%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'r'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Right at %c0%% | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            telemetrySpeed, left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 's'){
                        snprintf(combined_message, 100, "Mode: Remote | Ins: Stop | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                            left_average_speed, left_total_distance,
                            right_average_speed, right_total_distance);
                    }else if(telemetryInstruction == 'a'){
                        snprintf(combined_message, 100, "Mode: Auto | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                                left_average_speed, left_total_distance,
                                right_average_speed, right_total_distance);
                    }else{
                        snprintf(combined_message, 100, "Receiving data... | Left: %.2f m/s, %.2f m | Right: %.2f m/s, %.2f m\n",
                                left_average_speed, left_total_distance,
                                right_average_speed, right_total_distance);
                    }
                    xMessageBufferSend(wifiMessageBuffer, combined_message, strlen(combined_message) + 1, portMAX_DELAY);
                }
            }
        }

        last_edge_time[index] = current_time;  // Update the last valid edge time
        prev_state[index] = current_state;    // Update the previous state
    }
}
