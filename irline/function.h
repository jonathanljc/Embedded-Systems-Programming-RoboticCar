#ifndef FUNCTION_H
#define FUNCTION_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "message_buffer.h"


extern const char *code39_patterns[];
extern const char code39_chars[];


#define R_MOTOR_PWM_PIN 10
#define R_MOTOR_DIR_PIN1 14
#define R_MOTOR_DIR_PIN2 15
#define L_MOTOR_PWM_PIN 11
#define L_MOTOR_DIR_PIN1 12
#define L_MOTOR_DIR_PIN2 13

#define LINE_SENSOR_PIN 27
#define BARCODE_SENSOR_PIN 26
#define LINE_SENSOR_THRESHOLD 1000
#define BARCODE_SENSOR_THRESHOLD 1000
#define MAX_WHITE_TIME 50000

extern bool once_detect_black;
extern bool initial_blackline_detected;
extern bool disconnectRemote;

extern MessageBufferHandle_t wifiMessageBuffer;


// Motor Control
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
void init_motor_pins();


// Barcode Detection
void detectsurface(void *pvParameters);
void reset();
void decode();

// Utility
uint16_t read_adc();
void display();
char decodedchar();
char classify(uint64_t width, uint64_t max_width);
int calculate_distance(const char *s, const char *t);
void detectsurface(void *pvParameters);
void setmotor(uint32_t gpio, float speed, bool is_left);




void unified_task(void *pvParameters);


#endif // FUNCTION_H
