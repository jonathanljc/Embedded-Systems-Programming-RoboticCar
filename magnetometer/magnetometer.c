#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "magnetometer.h"  // Include your header file

#define MAG_ADDR 0x1E         // I2C address for the magnetometer
#define I2C_SPEED 400         // I2C clock speed in kHz
#define POLLING_DELAY 1080    // Time between sensor readings (ms)
#define SDA_PIN 14            // I2C SDA pin (adjustable)
#define SCL_PIN 15            // I2C SCL pin (adjustable)
#define I2C_INSTANCE i2c1     // Using I2C1 peripheral

// Magnetometer registers
#define MAG_CONFIG_A 0x00     // Config Register A
#define MAG_CONFIG_B 0x01     // Config Register B
#define MAG_MODE_REG 0x02     // Mode Register
#define MAG_DATA_START 0x03   // Starting address of data output (X MSB)

// I2C initialization
void setup_i2c() {
    i2c_init(I2C_INSTANCE, I2C_SPEED * 1000);  // Initialize I2C peripheral
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);   // Enable pull-up resistors
    gpio_pull_up(SCL_PIN);
}

// Magnetometer initialization
void setup_magnetometer() {
    uint8_t config_data[2];

    // Set refresh rate to 15 Hz
    config_data[0] = MAG_CONFIG_A;
    config_data[1] = 0x70;  // 15 Hz refresh rate
    i2c_write_blocking(I2C_INSTANCE, MAG_ADDR, config_data, 2, false);

    // Set gain for +/- 8.1 Gauss sensitivity
    config_data[0] = MAG_CONFIG_B;
    config_data[1] = 0xE0;
    i2c_write_blocking(I2C_INSTANCE, MAG_ADDR, config_data, 2, false);

    // Set mode to continuous reading
    config_data[0] = MAG_MODE_REG;
    config_data[1] = 0x00;
    i2c_write_blocking(I2C_INSTANCE, MAG_ADDR, config_data, 2, false);
}

// Fetch raw magnetometer data
void fetch_magnetometer_data(magnetometer_data_t *mag) {
    uint8_t raw_data[6];    // Buffer to hold raw data
    int16_t raw_values[3];  // Array for X, Y, Z data

    // Point to data register
    uint8_t reg_pointer = MAG_DATA_START;
    i2c_write_blocking(I2C_INSTANCE, MAG_ADDR, &reg_pointer, 1, true);

    // Read 6 bytes of data (X MSB -> Z LSB)
    i2c_read_blocking(I2C_INSTANCE, MAG_ADDR, raw_data, 6, false);

    // Combine high and low bytes to get 16-bit signed values for X, Y, Z
    for (int i = 0; i < 3; i++) {
        raw_values[i] = (raw_data[i * 2] << 8) | raw_data[(i * 2) + 1];
    }

    mag->axis_x = raw_values[0];
    mag->axis_y = raw_values[1];
    mag->axis_z = raw_values[2];
}

// Calculate the heading angle using magnetometer data
int32_t calculate_heading(magnetometer_data_t *mag) {
    int32_t heading = atan2(mag->axis_y, mag->axis_x) * 180.0 / M_PI;
    if (heading < 0) {
        heading += 360;  // Normalize angle to [0, 360)
    }
    return heading;
}

int main() {
    // Setup
    stdio_init_all();
    setup_i2c();
    setup_magnetometer();

    magnetometer_data_t mag_data;

    // Loop to continuously fetch and display magnetometer data
    while (true) {
        fetch_magnetometer_data(&mag_data);
        int32_t heading_angle = calculate_heading(&mag_data);

        // Output the raw X, Y, Z magnetometer values and calculated heading
        printf("\nMagnetometer Data:\nX = %4d\nY = %4d\nZ = %4d\n", mag_data.axis_x, mag_data.axis_y, mag_data.axis_z);
        printf("Heading Angle: %d degrees\n", heading_angle);

        sleep_ms(POLLING_DELAY);  // Delay between readings
    }

    return 0;
}
