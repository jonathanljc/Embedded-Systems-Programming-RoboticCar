#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"  // Include FreeRTOS message buffer library

// Define ultrasonic sensor pins
#define TRIGPIN 1
#define ECHOPIN 0

// Global variables
extern volatile absolute_time_t start_time;
extern volatile uint64_t pulse_width;
extern volatile bool obstacleDetected;
extern MessageBufferHandle_t printMessageBuffer;  // Message buffer for printing data

// Structure to hold the distance and obstacle status
typedef struct
{
    double distance;
    bool obstacleDetected;
} DistanceMessage;

// Kalman filter structure
typedef struct kalman_state_
{
    double q; // process noise covariance
    double r; // measurement noise covariance
    double x; // estimated value
    double p; // estimation error covariance
    double k; // kalman gain
} kalman_state;

// Function prototypes
kalman_state *kalman_init(double q, double r, double p, double initial_value);
void get_echo_pulse(uint gpio, uint32_t events);
void kalman_update(kalman_state *state, double measurement);
void ultrasonic_task(void *pvParameters);
void print_task(void *pvParameters);
void setupUltrasonicPins();

#endif // ULTRASONIC_H