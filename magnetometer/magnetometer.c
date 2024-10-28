#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

// Define constants
#define ACCEL_I2C_ADDR 0x19  // Accelerometer address
#define MAG_I2C_ADDR 0x1E    // Magnetometer address
#define CTRL_REG1_A 0x20     // Accelerometer control register
#define CTRL_REG1_M 0x00     // Magnetometer control register (CRA_REG_M)
#define OUT_X_L_A 0x28       // Accelerometer data register
#define OUT_X_H_M 0x03       // Magnetometer data register (OUT_X_H_M)
#define CTRL_REG4_A 0x23

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define conversion constants
#define ACCEL_CONVERSION 0.00059841  // Conversion factor for ±2g mode (m/s²/LSB)

// Function to initialize the GY-511 sensor (accelerometer and magnetometer)
void gy511_init(i2c_inst_t *i2c) {
    uint8_t buf[2];

    // Initialize accelerometer (CTRL_REG1_A)
    buf[0] = CTRL_REG1_A;
    buf[1] = 0x57;  // Enable accelerometer, 100Hz data rate, all axes enabled
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, buf, 2, false);

    // Initialize magnetometer (CRA_REG_M)
    buf[0] = CTRL_REG1_M;  // CRA_REG_M
    buf[1] = 0x1C;         // Temp sensor disable, 220Hz output rate
    i2c_write_blocking(i2c, MAG_I2C_ADDR, buf, 2, false);

    // Initialize magnetometer mode (MR_REG_M)
    buf[0] = 0x02;         // MR_REG_M
    buf[1] = 0x00;         // Continuous conversion mode
    i2c_write_blocking(i2c, MAG_I2C_ADDR, buf, 2, false);
}

// Function to read raw accelerometer data (16-bit integers)
void gy511_read_raw_acceleration(i2c_inst_t *i2c, int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az) {
    uint8_t buf[6];
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, (uint8_t[]){OUT_X_L_A | 0x80}, 1, true);  // MSB set for auto-increment
    i2c_read_blocking(i2c, ACCEL_I2C_ADDR, buf, 6, false);

    *raw_ax = (int16_t)(buf[1] << 8 | buf[0]);
    *raw_ay = (int16_t)(buf[3] << 8 | buf[2]);
    *raw_az = (int16_t)(buf[5] << 8 | buf[4]);
}

// Function to read raw magnetometer data (16-bit integers)
void gy511_read_raw_magnetometer(i2c_inst_t *i2c, int16_t *raw_mx, int16_t *raw_my, int16_t *raw_mz) {
    uint8_t buf[6];
    i2c_write_blocking(i2c, MAG_I2C_ADDR, (uint8_t[]){OUT_X_H_M | 0x80}, 1, true);  // MSB set for auto-increment
    i2c_read_blocking(i2c, MAG_I2C_ADDR, buf, 6, false);

    // Magnetometer has different endianness; correct order of bytes
    *raw_mx = (int16_t)(buf[0] << 8 | buf[1]);
    *raw_my = (int16_t)(buf[2] << 8 | buf[3]);
    *raw_mz = (int16_t)(buf[4] << 8 | buf[5]);
}

// Function to calculate the heading (direction in degrees)
float calculate_heading(int16_t raw_mx, int16_t raw_my) {
    float heading = atan2((float)raw_my, (float)raw_mx) * (180.0 / M_PI);  // Convert radians to degrees
    if (heading < 0) {
        heading += 360;  // Ensure heading is positive
    }
    return heading;
}

// Function to convert raw accelerometer data to m/s²
void convert_acceleration_to_mps2(int16_t raw_ax, int16_t raw_ay, int16_t raw_az, float *ax_mps2, float *ay_mps2, float *az_mps2) {
    *ax_mps2 = raw_ax * ACCEL_CONVERSION;
    *ay_mps2 = raw_ay * ACCEL_CONVERSION;
    *az_mps2 = raw_az * ACCEL_CONVERSION;
}

int main() {
    stdio_init_all();

    // Wait for USB serial connection to be established
    while (!stdio_usb_connected()) {
        tight_loop_contents();  // Wait until USB is connected
    }

    // Initialize I2C on I2C1 (GP26 SDA, GP27 SCL)
    i2c_init(i2c1, 100 * 1000);  // 100kHz clock speed
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    gy511_init(i2c1);  // Initialize the GY-511 sensor

    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_mx, raw_my, raw_mz;
    float ax_mps2, ay_mps2, az_mps2;

    while (1) {
        // Read raw acceleration data
        gy511_read_raw_acceleration(i2c1, &raw_ax, &raw_ay, &raw_az);
        // Convert raw acceleration to m/s²
        convert_acceleration_to_mps2(raw_ax, raw_ay, raw_az, &ax_mps2, &ay_mps2, &az_mps2);

        // Read raw magnetometer data
        gy511_read_raw_magnetometer(i2c1, &raw_mx, &raw_my, &raw_mz);
        float heading = calculate_heading(raw_mx, raw_my);

        // Output the raw acceleration, converted acceleration (m/s²), and magnetometer values along with heading
        printf("Raw Acceleration - X: %d | Y: %d | Z: %d\n", raw_ax, raw_ay, raw_az);
        printf("Converted Acceleration (m/s²) - X: %.4f | Y: %.4f | Z: %.4f\n", ax_mps2, ay_mps2, az_mps2);
        printf("Raw Magnetometer - X: %d | Y: %d | Z: %d | Heading: %.2f°\n", raw_mx, raw_my, raw_mz, heading);

        // Delay for 100ms
        sleep_ms(100);
    }

    return 0;
}