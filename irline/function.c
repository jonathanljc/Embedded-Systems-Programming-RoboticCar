#include "function.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>
#include "pico/time.h" 
#include "motor.h"



#define IR_SENSOR_PIN 26
#define THRESHOLD 1000
#define BAR_MAX 29
#define NOISE_THRESHOLD 500
#define DEBOUNCE_DELAY 500

uint64_t bar_length[BAR_MAX];
const char *bar_colouring[BAR_MAX];
int num_of_bar = 0;
uint64_t max_width = 0;
const char *initial_colour = "White";
#define MAX_WHITE_TIME 50000

volatile uint64_t last_white_time = 0;
volatile uint64_t last_transition_time = 0;
bool status_decode = false;


bool once_detect_black = false;
bool initial_blackline_detected = false;

// Code 39 Encoding (Narrow = N, Wide = W)
const char *code39_patterns[] = {
    "NWNNWNWNNNNNNWWNWNNNNWNNWNWNN", "NWNNWNWNNNWNNWNNNNWNNWNNWNWNN", "NWNNWNWNNNNNWWNNNNWNNWNNWNWNN", "NWNNWNWNNNWNWWNNNNNNNWNNWNWNN", "NWNNWNWNNNNNNWWNNNWNNWNNWNWNN", // 0-4
    "NWNNWNWNNNWNNWWNNNNNNWNNWNWNN", "NWNNWNWNNNNNWWWNNNNNNWNNWNWNN", "NWNNWNWNNNNNNWNNWNWNNWNNWNWNN", "NWNNWNWNNNWNNWNNWNNNNWNNWNWNN", "NWNNWNWNNNNNWWNNWNNNNWNNWNWNN", // 5-9
    "NWNNWNWNNNWNNNNWNNWNNWNNWNWNN", "NWNNWNWNNNNNWNNWNNWNNWNNWNWNN", "NWNNWNWNNNWNWNNWNNNNNWNNWNWNN", "NWNNWNWNNNNNNNWWNNWNNWNNWNWNN", "NWNNWNWNNNWNNNWWNNNNNWNNWNWNN", // A-E
    "NWNNWNWNNNNNWNWWNNNNNWNNWNWNN", "NWNNWNWNNNNNNNNWWNWNNWNNWNWNN", "NWNNWNWNNNWNNNNWWNNNNWNNWNWNN", "NWNNWNWNNNNNWNNWWNNNNWNNWNWNN", "NWNNWNWNNNNNNNWWWNNNNWNNWNWNN", // F-J
    "NWNNWNWNNNWNNNNNNWWNNWNNWNWNN", "NWNNWNWNNNNNWNNNNWWNNWNNWNWNN", "NWNNWNWNNNWNWNNNNWNNNWNNWNWNN", "NWNNWNWNNNNNNNWNNWWNNWNNWNWNN", "NWNNWNWNNNWNNNWNNWNNNWNNWNWNN", // K-O
    "NWNNWNWNNNNNWNWNNWNNNWNNWNWNN", "NWNNWNWNNNNNNNNNWWWNNWNNWNWNN", "NWNNWNWNNNWNNNNNWWNNNWNNWNWNN", "NWNNWNWNNNNNWNNNWWNNNWNNWNWNN", "NWNNWNWNNNNNNNWNWWNNNWNNWNWNN", // P-T
    "NWNNWNWNNNWWNNNNNNWNNWNNWNWNN", "NWNNWNWNNNNWWNNNNNWNNWNNWNWNN", "NWNNWNWNNNWWWNNNNNNNNWNNWNWNN", "NWNNWNWNNNNWNNWNNNWNNWNNWNWNN", "NWNNWNWNNNWWNNWNNNNNNWNNWNWNN", // U-Y
    "NWNNWNWNNNNWWNWNNNNNNWNNWNWNN", "NWNNWNWNNNNWNNNNWNWNNWNNWNWNN", "NWNNWNWNNNWWNNNNWNNNNWNNWNWNN", "NWNNWNWNNNWWWWWWWWNNNWNNWNWNN"                                   // Z-*
};
const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";




uint16_t read_adc() {
    return adc_read();
}


void detectsurface(void *pvParameters) {
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (true) {
        uint16_t adc_value = read_adc();
        const char *current_color = (adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

    
        if (!status_decode && strcmp(current_color, "Black") == 0) {
            status_decode = true;
            reset();
            last_transition_time = current_time;
            initial_colour = "Black";
            printf("Start barcode\n");
        }

        if (status_decode) {
            if (strcmp(current_color, initial_colour) != 0) {
                uint64_t pulse_timing = current_time - last_transition_time;

                if (pulse_timing > NOISE_THRESHOLD + DEBOUNCE_DELAY && num_of_bar < BAR_MAX) {
                    bar_length[num_of_bar] = pulse_timing;
                    bar_colouring[num_of_bar] = initial_colour;

                    printf("Bar #%d: Width = %llu us, Bar Color = %s\n", num_of_bar, pulse_timing, initial_colour);

                    if (pulse_timing > max_width) {
                        max_width = pulse_timing;
                    }

                    num_of_bar++;
                }

                if (num_of_bar >= BAR_MAX) {
                    decode();
                }

                initial_colour = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period (end of barcode)
            if (strcmp(current_color, "White") == 0) {
                if (last_white_time == 0) {
                    last_white_time = current_time; // Start timing white period
                } else if (current_time - last_white_time > MAX_WHITE_TIME) {
                    decode();
                    last_white_time = 0; // Reset white timing
                }
            } else {
                last_white_time = 0; // Reset white timing if black is detected
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for next ADC reading
    }
}

void reset() {
    for (int i = 0; i < BAR_MAX; i++) {
        bar_length[i] = 0;
        bar_colouring[i] = "not detected";
    }
    num_of_bar = 0;
    max_width = 0;
    once_detect_black = false;
    initial_colour = "White";
    last_white_time = 0;
}

void display() {
    printf("Captured Bar Data:\n");
    for (int i = 0; i < num_of_bar; i++) {
        printf("Number #%d: The Bar Width= %llu us, Color = %s\n", i, bar_length[i], bar_colouring[i]);
    }
}


void decode() {
    printf("Decoding in progress...\n");
    display();

    
    char decoded_char = decodedchar();
    if (decoded_char != '?') {
        printf("Decoded Character: %c\n", decoded_char);
        char telemtryBarcode[100];
        sprintf(telemtryBarcode, "Barcode: %c", decoded_char);
        xMessageBufferSend(wifiMessageBuffer, telemtryBarcode, strlen(telemtryBarcode) + 1, 0);
    } else {
        printf("Failed to decode barcode.\n");
    }

   
    reset();
    status_decode = false;
}

char decodedchar() {
    char pattern[BAR_MAX + 1] = {0};

    for (int i = 0; i < num_of_bar; i++) {
        pattern[i] = classify(bar_length[i], max_width);
    }
    pattern[num_of_bar] = '\0'; 

    printf("Captured Pattern: %s\n", pattern);

    // Check pattern against Code 39 reference patterns
    int minimum_dist = INT_MAX;
    char nearest_bar_result = '?';

    int num_patterns = sizeof(code39_patterns) / sizeof(code39_patterns[0]);
    for (int i = 0; i < num_patterns; i++) {
        const char *reference_pattern = code39_patterns[i];
        int distance = calculate_distance(pattern, reference_pattern);

        if (distance < minimum_dist) {
            minimum_dist = distance;
            nearest_bar_result = code39_chars[i];
        }
    }

    
    int max_acceptable_distance = 10; 
    if (minimum_dist <= max_acceptable_distance) {
        printf("Nearest Bar Result: %c with Distance: %d\n", nearest_bar_result, minimum_dist);
        return nearest_bar_result;
    } else {
        printf("No acceptable match found (Min Distance: %d)\n", minimum_dist);
        return '?';
    }
}

char classify(uint64_t width, uint64_t max_width) {
   
    if (width <= max_width * 0.6) {
        return 'N'; // Narrow
    } else {
        return 'W'; // Wide
    }
}


int calculate_distance(const char *s, const char *t) {
    int len_s = strlen(s);
    int len_t = strlen(t);

    int matrices[len_s + 1][len_t + 1];

    // Initialize matrices
    for (int i = 0; i <= len_s; i++) matrices[i][0] = i;
    for (int j = 0; j <= len_t; j++) matrices[0][j] = j;

    // Fill matrices with distances
    for (int i = 1; i <= len_s; i++) {
        for (int j = 1; j <= len_t; j++) {
            int cost = (s[i - 1] == t[j - 1]) ? 0 : 1;
            int deletion = matrices[i - 1][j] + 1;
            int inclution = matrices[i][j - 1] + 1;
            int replacement = matrices[i - 1][j - 1] + cost;

            int min = deletion < inclution ? deletion : inclution;
            matrices[i][j] = min < replacement ? min : replacement;
        }
    }

    return matrices[len_s][len_t];
}





void unified_task(void *pvParameters) {
    // Initialize ADC for line and barcode sensors
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);    // Line sensor
    adc_gpio_init(BARCODE_SENSOR_PIN); // Barcode sensor

    adc_select_input(0);

    uint16_t line_adc_value = 0;
    bool black_line_detected = false;

    uint16_t barcode_adc_value = 0;
    const char *current_color = NULL;
    const char *initial_colour = NULL;
    uint64_t last_transition_time = 0;
    uint64_t last_white_time = 0;
    uint64_t current_time = 0;
    bool status_decode = false;
    bool once_detect_black = false;
    uint16_t num_of_bar = 0;
    uint64_t bar_length[BAR_MAX] = {0};
    const char *bar_colouring[BAR_MAX] = {0};
    uint64_t max_width = 0;

    // Wait until black surface is detected to start
    printf("Waiting for black surface to start...\n");
    do {
        adc_select_input(1); // Read from line sensor (GPIO 27)
        line_adc_value = adc_read();
        printf("The ADC value is: %u\n", line_adc_value);
        black_line_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (!black_line_detected) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10 ms
        }
    } while (!black_line_detected);

    printf("Black surface detected! Starting task...\n");
    disconnectRemote = true;

    while (1) {
        // **Line Sensor Logic** (Motor Control)
        adc_select_input(1); // Switch to line sensor (GPIO 27)
        line_adc_value = adc_read();
        printf("The ADC value is: %u\n", line_adc_value);
        black_line_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);

        if (black_line_detected) {
            // Move forward on black
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
        } else {
            // Stop motors on white
            set_motor_speed(L_MOTOR_PWM_PIN, 0, true);
            set_motor_speed(R_MOTOR_PWM_PIN, 0, false);

            // Rotate left, then right if no black is detected
            while (!black_line_detected) {
                // Rotate left and continuously check for black
                rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

                
                for (int i = 0; i < 40; i++) { // 200 iterations of 10ms delay = 2 seconds
                    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10ms
                    adc_select_input(1); // Check for black again
                    line_adc_value = adc_read();
                    black_line_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);
                    
                    if (black_line_detected) {
                        // If black is detected, break out of rotation
                        break;
                    }
                }

                if (black_line_detected) {
                    break;
                }

                // Rotate right and continuously check for black
                rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                
                for (int i = 0; i < 40; i++) { // 200 iterations of 10ms delay = 2 seconds
                    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10ms
                    adc_select_input(1); // Check for black again
                    line_adc_value = adc_read();
                    black_line_detected = (line_adc_value >= LINE_SENSOR_THRESHOLD);
                    
                    if (black_line_detected) {
                        // If black is detected, break out of rotation
                        break;
                    }
                }

                if (black_line_detected) {
                    break;
                }
            }

            // If black is detected, move forward
            if (black_line_detected) {
                move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            }
        }

        // Small delay before next iteration
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


