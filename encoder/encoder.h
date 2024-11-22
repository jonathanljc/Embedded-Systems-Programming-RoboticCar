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
#define LEFT_WHEEL_ENCODER_PIN 6
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define MICROSECONDS_IN_A_SECOND 1000000.0
#define MIN_PULSE_WIDTH 500  // Adjust this threshold based on testing results
#define AVERAGE_FILTER_SIZE 10  // Set a buffer size based on desired smoothness
extern MessageBufferHandle_t motorMessageBuffer;  // Message buffer for motor control
extern MessageBufferHandle_t wifiReceiveBuffer;   // Message buffer for wifi receive
extern MessageBufferHandle_t wifiMessageBuffer;   // Message buffer for wifi send

float calculate_moving_average(float buffer[], int size);  // Helper function for moving average
void poll_encoder(void); 
void init_encoder_gpio(void);

#endif // ENCODER_H
