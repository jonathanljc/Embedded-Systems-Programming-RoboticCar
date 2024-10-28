// encoder.h
#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/time.h"
#include "message_buffer.h"

// Pin and configuration definitions
#define LEFT_WHEEL_ENCODER_PIN 2
#define RIGHT_WHEEL_ENCODER_PIN 3
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define MICROSECONDS_IN_A_SECOND 1000000.0

// Struct to hold pulse data for each wheel
typedef struct {
    uint32_t pulse_width;
    float speed;
    float total_distance;
    uint32_t notch_count;
} PulseData_t;

// Global variables
extern volatile uint32_t left_notch_count;
extern volatile uint32_t right_notch_count;
extern volatile bool left_data_ready;
extern volatile bool right_data_ready;

// Task handles and message buffers
extern TaskHandle_t leftPulseTaskHandle;
extern TaskHandle_t rightPulseTaskHandle;
extern TaskHandle_t leftSpeedTaskHandle;
extern TaskHandle_t rightSpeedTaskHandle;
extern MessageBufferHandle_t leftMessageBuffer;
extern MessageBufferHandle_t rightMessageBuffer;

// Function prototypes
void gpio_callback(uint gpio, uint32_t events);
void pulse_width_task(void *pvParameters);
void speed_task(void *pvParameters);
void log_task(void *pvParameters);

#endif // WHEEL_ENCODER_H
