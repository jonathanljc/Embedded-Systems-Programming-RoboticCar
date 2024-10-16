#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdint.h>

// Struct to hold raw magnetometer data (X, Y, Z axes)
typedef struct {
    int16_t axis_x;  // X-axis data
    int16_t axis_y;  // Y-axis data
    int16_t axis_z;  // Z-axis data
} magnetometer_data_t;

// Function prototypes (names changed for consistency)
void setup_i2c(void);                           // Initializes I2C communication
void setup_magnetometer(void);                  // Initializes the magnetometer
void fetch_magnetometer_data(magnetometer_data_t *mag);  // Fetches magnetometer raw data (X, Y, Z)
int32_t calculate_heading(magnetometer_data_t *mag);     // Calculates heading angle from magnetometer data

#endif // MAGNETOMETER_H
