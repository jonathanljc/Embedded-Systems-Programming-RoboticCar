// encoder.h
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/time.h"
#include "message_buffer.h"

#ifndef ENCODER_H
#define ENCODER_H
// Definitions
#define LEFT_WHEEL_ENCODER_PIN 8
#define RIGHT_WHEEL_ENCODER_PIN 6
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define MICROSECONDS_IN_A_SECOND 1000000.0
#define MIN_PULSE_WIDTH 500  // Adjust this threshold based on testing results
#define AVERAGE_FILTER_SIZE 10  // Set a buffer size based on desired smoothness

// Data structure for encoder data
typedef struct {
    uint32_t pulse_width;
    float speed;
    float total_distance;
    uint32_t notch_count;
} PulseData_t;

// External message buffer handles for motor control and logging
extern MessageBufferHandle_t leftMotorControlBuffer;
extern MessageBufferHandle_t rightMotorControlBuffer;
extern MessageBufferHandle_t leftMessageBuffer;
extern MessageBufferHandle_t rightMessageBuffer;

// Task handles
extern TaskHandle_t leftPulseTaskHandle;
extern TaskHandle_t leftSpeedTaskHandle;
extern TaskHandle_t rightPulseTaskHandle;
extern TaskHandle_t rightSpeedTaskHandle;



// Function declarations
// encoder.h
void setup_pins_with_unified_callback();
void encoder_init(void);
void encoder_gpio_callback(uint gpio, uint32_t events);
void left_pulse_width_task(void *pvParameters);
void right_pulse_width_task(void *pvParameters);
void left_speed_task(void *pvParameters);
void right_speed_task(void *pvParameters);
void left_log_task(void *pvParameters);
void right_log_task(void *pvParameters);

#endif // ENCODER_H
