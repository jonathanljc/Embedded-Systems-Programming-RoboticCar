#ifndef FUNCTION_H
#define FUNCTION_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>


extern const char *code39_patterns[];
extern const char code39_chars[];

// Motor Pin Definitions
#define L_MOTOR_PWM_PIN 11
#define L_MOTOR_DIR_PIN1 12
#define L_MOTOR_DIR_PIN2 13
#define R_MOTOR_PWM_PIN 10
#define R_MOTOR_DIR_PIN1 14
#define R_MOTOR_DIR_PIN2 15

#define LINE_SENSOR_PIN 27
#define BARCODE_SENSOR_PIN 26
#define LINE_SENSOR_THRESHOLD 1000
#define BARCODE_SENSOR_THRESHOLD 2480
#define MAX_WHITE_TIME 5000000

extern bool initial_black_detected;
extern bool initial_blackline_detected;
extern bool disconnectRemote;

// extern MessageBufferHandle_t wifiMessageBuffer;

// Motor Control
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
void init_motor_pins();


// Barcode Detection
void detect_surface_contrast_task(void *pvParameters);
void reset_bar_data();
void decode_barcode();

// Utility
uint16_t read_adc();
void display_captured_bars();
char decode_character();
char classify_bar_width(uint64_t width, uint64_t max_width);
int levenshtein_distance(const char *s, const char *t);
void detect_surface_contrast_task(void *pvParameters);
void set_motor_speed(uint32_t gpio, float speed, bool is_left);



// Straight Line Task
//void straight_line_task(void *pvParameters);
// Unified Task
void unified_task(void *pvParameters);
void unified_function();
void blackline_function();

#endif // FUNCTION_H
