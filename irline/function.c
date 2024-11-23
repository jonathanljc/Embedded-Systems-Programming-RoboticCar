#include "function.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h> // For INT_MAX
#include <unistd.h>
#include "pico/time.h" // Include this for sleep_us and sleep_ms
#include "motor.h"



// Motor GPIO Pins
#define L_MOTOR_PWM_PIN 11
#define L_MOTOR_DIR_PIN1 12
#define L_MOTOR_DIR_PIN2 13
#define R_MOTOR_PWM_PIN 10
#define R_MOTOR_DIR_PIN1 14
#define R_MOTOR_DIR_PIN2 15

// PWM Settings
#define MAX_DUTY_CYCLE 12500
#include "FreeRTOS.h"
#include "task.h"


// Barcode Detection
#define IR_SENSOR_PIN 26
#define THRESHOLD 2480
#define MAX_BARS 29
#define NOISE_THRESHOLD 500
#define DEBOUNCE_DELAY 500

volatile uint64_t last_white_time = 0;
volatile uint64_t last_transition_time = 0;
bool decoding_active = false;


uint64_t bar_widths[MAX_BARS];
const char *bar_colors[MAX_BARS];
int bar_count = 0;
uint64_t max_width = 0;
const char *previous_color = "White";
#define MAX_WHITE_TIME 5000000 // Fixed definition

bool initial_black_detected = false;
bool initial_blackline_detected = false;

// Code 39 Encoding (Narrow = 1, Wide = 2)
const char *code39_patterns[] = {
    "12112121111112212111121121211", "12112121112112111121121121211", "12112121111122111121121121211", "12112121112122111111121121211", "12112121111112211121121121211", // 0-4
    "12112121112112211111121121211", "12112121111122211111121121211", "12112121111112112121121121211", "12112121112112112111121121211", "12112121111122112111121121211", // 5-9
    "12112121112111121121121121211", "12112121111121121121121121211", "12112121112121121111121121211", "12112121111111221121121121211", "12112121112111221111121121211", // A-E
    "12112121111121221111121121211", "12112121111111122121121121211", "12112121112111122111121121211", "12112121111121122111121121211", "12112121111111222111121121211", // F-J
    "12112121112111111221121121211", "12112121111121111221121121211", "12112121112121111211121121211", "12112121111111211221121121211", "12112121112111211211121121211", // K-O
    "12112121111121211211121121211", "12112121111111112221121121211", "12112121112111112211121121211", "12112121111121112211121121211", "12112121111111212211121121211", // P-T
    "12112121112211111121121121211", "12112121111221111121121121211", "12112121112221111111121121211", "12112121111211211121121121211", "12112121112211211111121121211", // U-Y
    "12112121111221211111121121211", "12112121111211112121121121211", "12112121112211112111121121211", "12112121112222222211121121211"                                   // Z-*
};

const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";




uint16_t read_adc() {
    return adc_read();
}

void reset_bar_data() {
    for (int i = 0; i < MAX_BARS; i++) {
        bar_widths[i] = 0;
        bar_colors[i] = "Unknown";
    }
    bar_count = 0;
    max_width = 0;
    initial_black_detected = false;
    previous_color = "White";
    last_white_time = 0;
}



void detect_surface_contrast_task(void *pvParameters) {
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (true) {
        uint16_t adc_value = read_adc();
        const char *current_color = (adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        // Start decoding when first black is detected
        if (!decoding_active && strcmp(current_color, "Black") == 0) {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = current_time;
            previous_color = "Black";
            printf("Starting barcode detection\n");
        }

        // Continue decoding once active
        if (decoding_active) {
            if (strcmp(current_color, previous_color) != 0) {
                uint64_t pulse_duration_us = current_time - last_transition_time;

                // Capture bar width if above noise threshold
                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS) {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar #%d: Width = %llu us, Color = %s\n", bar_count, pulse_duration_us, previous_color);

                    // Update max width dynamically
                    if (pulse_duration_us > max_width) {
                        max_width = pulse_duration_us;
                    }

                    bar_count++;
                }

                // Check if maximum bars have been captured
                if (bar_count >= MAX_BARS) {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period (end of barcode)
            if (strcmp(current_color, "White") == 0) {
                if (last_white_time == 0) {
                    last_white_time = current_time; // Start timing white period
                } else if (current_time - last_white_time > MAX_WHITE_TIME) {
                    printf("Detected extended white space - triggering decode.\n");
                    decode_barcode();
                    last_white_time = 0; // Reset white timing
                }
            } else {
                last_white_time = 0; // Reset white timing if black is detected
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for next ADC reading
    }
}

void decode_barcode() {
    printf("Decoding barcode...\n");
    display_captured_bars();

    // Decode captured pattern
    char decoded_char = decode_character();
    if (decoded_char != '?') {
        printf("Decoded Character: %c\n", decoded_char);
    } else {
        printf("Failed to decode barcode.\n");
    }

    // Reset data for next barcode
    reset_bar_data();
    decoding_active = false;
}

char decode_character() {
    // Convert bar widths into a pattern string (Narrow = '1', Wide = '2')
    char pattern[MAX_BARS + 1] = {0};

    for (int i = 0; i < bar_count; i++) {
        pattern[i] = classify_bar_width(bar_widths[i], max_width);
    }
    pattern[bar_count] = '\0'; // Null-terminate pattern

    printf("Captured Pattern: %s\n", pattern);

    // Check pattern against Code 39 reference patterns
    int min_distance = INT_MAX;
    char best_match = '?';

    int num_patterns = sizeof(code39_patterns) / sizeof(code39_patterns[0]);
    for (int i = 0; i < num_patterns; i++) {
        const char *reference_pattern = code39_patterns[i];
        int distance = levenshtein_distance(pattern, reference_pattern);

        if (distance < min_distance) {
            min_distance = distance;
            best_match = code39_chars[i];
        }
    }

    // Validate decoding result
    int max_acceptable_distance = 10; // Threshold for Levenshtein distance
    if (min_distance <= max_acceptable_distance) {
        printf("Best Match: %c with Distance: %d\n", best_match, min_distance);
        return best_match;
    } else {
        printf("No acceptable match found (Min Distance: %d)\n", min_distance);
        return '?';
    }
}

char classify_bar_width(uint64_t width, uint64_t max_width) {
    // Use a ratio to classify bar width
    if (width <= max_width * 0.6) {
        return '1'; // Narrow
    } else {
        return '2'; // Wide
    }
}

void display_captured_bars() {
    printf("Captured Bar Data:\n");
    for (int i = 0; i < bar_count; i++) {
        printf("Bar #%d: Width = %llu us, Color = %s\n", i, bar_widths[i], bar_colors[i]);
    }
}

int levenshtein_distance(const char *s, const char *t) {
    int len_s = strlen(s);
    int len_t = strlen(t);

    int matrix[len_s + 1][len_t + 1];

    // Initialize matrix
    for (int i = 0; i <= len_s; i++) matrix[i][0] = i;
    for (int j = 0; j <= len_t; j++) matrix[0][j] = j;

    // Fill matrix with distances
    for (int i = 1; i <= len_s; i++) {
        for (int j = 1; j <= len_t; j++) {
            int cost = (s[i - 1] == t[j - 1]) ? 0 : 1;
            int deletion = matrix[i - 1][j] + 1;
            int insertion = matrix[i][j - 1] + 1;
            int substitution = matrix[i - 1][j - 1] + cost;

            int min = deletion < insertion ? deletion : insertion;
            matrix[i][j] = min < substitution ? min : substitution;
        }
    }

    return matrix[len_s][len_t];
}




/*
void unified_task(void *pvParameters) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);    // Line sensor
    adc_gpio_init(BARCODE_SENSOR_PIN); // Barcode sensor

    adc_select_input(0); // Select initial ADC channel

    uint64_t last_white_time = 0;
    bool line_black_detected = false;
    bool decoding_active = false;
    float move_speed = 0.5; // Adjusted speed for better performance

    while (true) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        uint16_t line_adc_value = adc_read();
        line_black_detected = (line_adc_value <= LINE_SENSOR_THRESHOLD);

        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, move_speed);
           
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);

        }

        // **Barcode Sensor Logic** (Character Detection)
        adc_select_input(0); // Switch to barcode sensor (GPIO 26)
        uint16_t barcode_adc_value = adc_read();
        const char *current_color = (barcode_adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        if (!decoding_active && strcmp(current_color, "Black") == 0)
        {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = current_time;
            previous_color = "Black"; // Initialize previous_color
            printf("Starting barcode detection\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0)
        {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected)
        {
            if (strcmp(current_color, previous_color) != 0)
            {
                uint64_t pulse_duration_us = current_time - last_transition_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS)
                {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar Color: %s\n", previous_color);
                    printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                    if (pulse_duration_us > max_width)
                    {
                        max_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", max_width);
                    }
                    bar_count++;
                }

                if (bar_count >= MAX_BARS)
                {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period to force decode
            if (strcmp(current_color, "White") == 0)
            {
                if (last_white_time == 0)
                {
                    last_white_time = current_time; // Start timing white period
                }
                else if (current_time - last_white_time > MAX_WHITE_TIME)
                {
                    printf("Detected extended white space - triggering decode.\n");
                    decode_barcode();
                    last_white_time = 0; // Reset white timing
                }
            }
            else
            {
                last_white_time = 0; // Reset white timing if black is detected
            }
        }
        else
        {
            // If not decoding, ensure last_white_time is reset
            last_white_time = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for next ADC reading
    }
}
*/


/*
void unified_function() {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);    // Line sensor
    adc_gpio_init(BARCODE_SENSOR_PIN); // Barcode sensor

    adc_select_input(0); // Select initial ADC channel

    uint64_t last_white_time = 0;
    bool line_black_detected = false;
    bool decoding_active = false;
    //float move_speed = 0.5; // Adjusted speed for better performance

    while (true) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        uint16_t line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);


        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN,R_MOTOR_PWM_PIN);
            printf("Line ADC Value: %u\n", line_adc_value);

        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Line ADC Value: %u\n", line_adc_value);
        }

        // **Barcode Sensor Logic** (Character Detection)
        adc_select_input(0); // Switch to barcode sensor (GPIO 26)
        uint16_t barcode_adc_value = adc_read();
        const char *current_color = (barcode_adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        if (!decoding_active && strcmp(current_color, "Black") == 0) {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = current_time;
            previous_color = "Black"; // Initialize previous_color
            printf("Starting barcode detection\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0) {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected) {
            if (strcmp(current_color, previous_color) != 0) {
                uint64_t pulse_duration_us = current_time - last_transition_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS) {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar Color: %s\n", previous_color);
                    printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                    if (pulse_duration_us > max_width) {
                        max_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", max_width);
                    }
                    bar_count++;
                }

                if (bar_count >= MAX_BARS) {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period to force decode
            if (strcmp(current_color, "White") == 0) {
                if (last_white_time == 0) {
                    last_white_time = current_time; // Start timing white period
                } else if (current_time - last_white_time > MAX_WHITE_TIME) {
                    printf("Detected extended white space - triggering decode.\n");
                    decode_barcode();
                    last_white_time = 0; // Reset white timing
                }
            } else {
                last_white_time = 0; // Reset white timing if black is detected
            }
        } else {
            // If not decoding, ensure last_white_time is reset
            last_white_time = 0;
        }

        // Add a delay to simulate task periodicity
        sleep_us(1000); // 1 ms delay

    }
}
*/

/*
void unified_function() {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN); // Line sensor

    adc_select_input(0); // Select initial ADC channel

    uint64_t last_white_time = 0;
    bool line_black_detected = false;

    while (true) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        uint16_t line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            printf("Line ADC Value: %u\n", line_adc_value);
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Line ADC Value: %u\n", line_adc_value);
        }

        // Add a delay to simulate task periodicity
        sleep_us(1000); // 1 ms delay
    }
}


bool is_inital_black_line_detected(uint16_t adc_value, uint16_t threshold) {
    return adc_value <= threshold;
}

void blackline_function() {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN); // Initialize line sensor pin

    adc_select_input(0); // Select ADC channel 0 for the line sensor

    while (true) {
        // Read ADC value from the line sensor
        uint16_t line_adc_value = adc_read();

        // Determine whether the surface is black or white based on the threshold
        if (line_adc_value >= LINE_SENSOR_THRESHOLD) {
            initial_blackline_detected=true;
            printf("Detected: Black (ADC Value: %u)\n", line_adc_value);
        } else {
            printf("Detected: White (ADC Value: %u)\n", line_adc_value);
        }

        // Add a delay to simulate task periodicity
        sleep_us(1000); // 1 ms delay
    }
}
*/

/*
void unified_task(void *pvParameters) {
    // Initialize ADC for line sensor
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    // Select initial ADC input
    adc_select_input(1); // Ensure correct initial channel

    // Variables
    uint16_t line_adc_value = 0;
    bool line_black_detected = false;

    // Wait until black surface is detected
    printf("Waiting for black surface to start...\n");
    do {
        adc_select_input(1); // Read from line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (!line_black_detected) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10 ms
        }
    } while (!line_black_detected);

    printf("Black surface detected! Starting task...\n");

    while (1) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            printf("Line ADC Value (Black Detected): %u\n", line_adc_value);
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Line ADC Value (White Detected): %u\n", line_adc_value);

            // Rotate left until black is detected
            while (!line_black_detected) {
                rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

                adc_select_input(1); // Read sensor value again
                line_adc_value = adc_read();
                line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);
            }

            // Stop briefly when black is detected again
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Black Detected - Stopping Briefly\n");
            vTaskDelay(pdMS_TO_TICKS(500)); // Stop for 500 milliseconds

            // Resume forward motion
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            printf("Resuming Forward Motion\n");
        }

        // Task periodicity
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay of 10 ms
    }
}
*/

/*Working
void unified_task(void *pvParameters) {
    // Initialize ADC for line sensor
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);

    // Select initial ADC input
    adc_select_input(1); // Ensure correct initial channel

    // Variables
    uint16_t line_adc_value = 0;
    bool line_black_detected = false;

    // Wait until black surface is detected
    printf("Waiting for black surface to start...\n");
    do {
        adc_select_input(1); // Read from line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (!line_black_detected) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10 ms
        }
    } while (!line_black_detected);

    printf("Black surface detected! Starting task...\n");

    while (1) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            printf("Line ADC Value (Black Detected): %u\n", line_adc_value);
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Line ADC Value (White Detected): %u\n", line_adc_value);

            // Rotate left, then right if no black is detected
            while (!line_black_detected) {
                // Rotate left
                printf("Rotating left...\n");
                rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                vTaskDelay(pdMS_TO_TICKS(200)); // Rotate for a short duration

                // Check for black surface
                adc_select_input(1); // Read sensor value again
                line_adc_value = adc_read();
                line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

                if (!line_black_detected) {
                    // Rotate right if still no black detected
                    printf("No black detected, rotating right...\n");
                    rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                    set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                    vTaskDelay(pdMS_TO_TICKS(200)); // Rotate for a short duration

                    // Check again for black surface
                    adc_select_input(1);
                    line_adc_value = adc_read();
                    line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);
                }
            }

            // Stop briefly when black is detected again
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            printf("Black Detected - Stopping Briefly\n");
            vTaskDelay(pdMS_TO_TICKS(500)); // Stop for 500 milliseconds

            // Resume forward motion
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            printf("Resuming Forward Motion\n");
        }

        // Task periodicity
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay of 10 ms
    }
}
*/

void unified_task(void *pvParameters) {
    // Initialize ADC for line and barcode sensors
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);    // Line sensor
    adc_gpio_init(BARCODE_SENSOR_PIN); // Barcode sensor

    // Select initial ADC input
    adc_select_input(0);

    // Variables for line following
    uint16_t line_adc_value = 0;
    bool line_black_detected = false;

    // Variables for barcode detection
    uint16_t barcode_adc_value = 0;
    const char *current_color = NULL;
    const char *previous_color = NULL;
    uint64_t last_transition_time = 0;
    uint64_t last_white_time = 0;
    uint64_t current_time = 0;
    bool decoding_active = false;
    bool initial_black_detected = false;
    uint16_t bar_count = 0;
    uint64_t bar_widths[MAX_BARS] = {0};
    const char *bar_colors[MAX_BARS] = {0};
    uint64_t max_width = 0;

    // Wait until black surface is detected to start
    printf("Waiting for black surface to start...\n");
    do {
        adc_select_input(1); // Read from line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (!line_black_detected) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10 ms
        }
    } while (!line_black_detected);

    printf("Black surface detected! Starting task...\n");

    while (1) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        line_adc_value = adc_read();
        line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (line_black_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);

            // Rotate left, then right if no black is detected
            while (!line_black_detected) {
                rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                vTaskDelay(pdMS_TO_TICKS(200));

                adc_select_input(1); // Check for black again
                line_adc_value = adc_read();
                line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

                if (!line_black_detected) {
                    rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                    set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                    vTaskDelay(pdMS_TO_TICKS(200));

                    adc_select_input(1); // Check again
                    line_adc_value = adc_read();
                    line_black_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);
                }
            }

            // Stop briefly when black is detected again
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // **Barcode Sensor Logic** (Character Detection)
        adc_select_input(0); // Switch to barcode sensor (GPIO 26)
        barcode_adc_value = adc_read();
        current_color = (barcode_adc_value > THRESHOLD) ? "Black" : "White";
        current_time = time_us_64();

        if (!decoding_active && strcmp(current_color, "Black") == 0) {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = current_time;
            previous_color = "Black"; // Initialize previous_color
            printf("Starting barcode detection\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0) {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected) {
            if (strcmp(current_color, previous_color) != 0) {
                uint64_t pulse_duration_us = current_time - last_transition_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS) {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar Color: %s\n", previous_color);
                    printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                    if (pulse_duration_us > max_width) {
                        max_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", max_width);
                    }
                    bar_count++;
                }

                if (bar_count >= MAX_BARS) {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period to force decode
            if (strcmp(current_color, "White") == 0) {
                if (last_white_time == 0) {
                    last_white_time = current_time;
                } else if (current_time - last_white_time > MAX_WHITE_TIME) {
                    printf("Detected extended white space - triggering decode.\n");
                    decode_barcode();
                    last_white_time = 0;
                }
            } else {
                last_white_time = 0;
            }
        } else {
            last_white_time = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for next ADC reading
    }
}


