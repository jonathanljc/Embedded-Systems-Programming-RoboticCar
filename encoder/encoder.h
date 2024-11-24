#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/time.h"
#include "message_buffer.h"

// Definitions
#define LEFT_WHEEL_ENCODER_PIN 6
#define RIGHT_WHEEL_ENCODER_PIN 8
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define MICROSECONDS_IN_A_SECOND 1000000.0
#define MIN_PULSE_WIDTH 500  // Ignore noise below 500 µs
#define DEBOUNCE_THRESHOLD 500  // Debounce threshold in microseconds
#define AVERAGE_FILTER_SIZE 20  // Moving average buffer size
#define ENCODER_INACTIVITY_TIMEOUT 1000000 // 5 seconds in microseconds

// Structure to hold encoder state
typedef struct {
    uint32_t notch_count;
    uint32_t last_rising_time;
    uint32_t last_falling_time;
    uint32_t pulse_width;
    float speed_buffer[AVERAGE_FILTER_SIZE];
    int buffer_index;
} Encoder;

// External message buffer handles
extern MessageBufferHandle_t motorMessageBuffer;
extern MessageBufferHandle_t wifiReceiveBuffer;
extern MessageBufferHandle_t wifiMessageBuffer;
// Global encoder instances (make accessible globally)
extern Encoder left_encoder;  // Left encoder state
extern Encoder right_encoder; // Right encoder state

extern char telemetryInstruction;
extern char telemetrySpeed;

// Function declarations
float calculate_moving_average(float buffer[], int size);
void init_encoder_gpio(void);
void poll_encoder(Encoder *encoder, uint32_t gpio_pin);  // Generalized polling for an encoder

#endif // ENCODER_H
