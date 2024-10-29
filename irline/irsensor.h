#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// Pin and threshold definitions
#define IR_SENSOR_PIN 26  // GP26 is ADC0
#define WHITE_THRESHOLD 2200  // Threshold for white and black detection

// Function declarations
void init_adc(void);
int read_ir_sensor(void);
void display_line_width(uint32_t pulse_width, const char* surface);
void ir_sensor_task(void *pvParameters);

#endif // IRSENSOR_H
