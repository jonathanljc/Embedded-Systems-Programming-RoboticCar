#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

extern MessageBufferHandle_t wifiMessageBuffer;

void magnetometer_task(__unused void *params);

#endif