#include "accelerometer.h"

#define ACCEL_I2C_ADDR 0x19
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28
#define ACCEL_CONVERSION 0.00059841 // Convert to m/s² for ±2g
#define FILTER_SAMPLES 5            // Reduced samples for faster response

// Circular buffers for moving average filter
float accel_x_buffer[FILTER_SAMPLES] = {0};
float accel_y_buffer[FILTER_SAMPLES] = {0};
float accel_z_buffer[FILTER_SAMPLES] = {0};
int buffer_index = 0;

int counterPrint = 0;

// Struct to hold accelerometer data
typedef struct {
    float x, y, z;
} AccelerometerData;

// Initialize GY-511 Accelerometer
void gy511_init(i2c_inst_t *i2c) {
    uint8_t config[2];
    config[0] = CTRL_REG1_A;
    config[1] = 0x57; // Enable accelerometer, 100Hz, all axes
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, config, 2, false);
}

// Read raw acceleration data
void gy511_read_acceleration(i2c_inst_t *i2c, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, (uint8_t[]){OUT_X_L_A | 0x80}, 1, true);
    i2c_read_blocking(i2c, ACCEL_I2C_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

// Moving average filter
void update_moving_average(float *buffer, float new_value) {
    buffer[buffer_index] = new_value;
}

float calculate_average(float *buffer) {
    float sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        sum += buffer[i];
    }
    return sum / FILTER_SAMPLES;
}

// Convert raw data to m/s² and apply smoothing
void process_acceleration(int16_t raw_ax, int16_t raw_ay, int16_t raw_az, AccelerometerData *accel_data) {
    accel_data->x = raw_ax * ACCEL_CONVERSION;
    accel_data->y = raw_ay * ACCEL_CONVERSION;
    accel_data->z = raw_az * ACCEL_CONVERSION;

    update_moving_average(accel_x_buffer, accel_data->x);
    update_moving_average(accel_y_buffer, accel_data->y);
    update_moving_average(accel_z_buffer, accel_data->z);

    buffer_index = (buffer_index + 1) % FILTER_SAMPLES;

    accel_data->x = calculate_average(accel_x_buffer);
    accel_data->y = calculate_average(accel_y_buffer);
    accel_data->z = calculate_average(accel_z_buffer);
}

// Generate control command based on accelerometer data
void generate_control_command(const AccelerometerData *accel_data, char *command_buffer) {
    float speed = sqrt(accel_data->x * accel_data->x + accel_data->y * accel_data->y) / 9.81; // Normalize to g-force
    speed = fmin(speed, 1.0) * 100;                                                           // Cap speed to 100%

    // Determine control command based on direction
    if (fabs(accel_data->y) > fabs(accel_data->x)) {
        snprintf(command_buffer, 50, "turn %s at %.1f%% speed", accel_data->y > 0 ? "left" : "right", speed);
    } else {
        snprintf(command_buffer, 50, "move %s at %.1f%% speed", accel_data->x > 0 ? "forward" : "backward", speed);
    }
}

// Magnetometer task for reading data and sending commands
void magnetometer_task(__unused void *params) {
    vTaskDelay(pdMS_TO_TICKS(5000));  // Shorter initial delay for faster start
    AccelerometerData accel_data;
    int16_t raw_ax, raw_ay, raw_az;
    char command[50];
    // char last_command[50] = "";  // Store last command to avoid duplicates

    // Initialize I2C and GY-511 sensor
    // i2c_init(i2c1, 100 * 1000);
    // gpio_set_function(26, GPIO_FUNC_I2C);
    // gpio_set_function(27, GPIO_FUNC_I2C);
    // gpio_pull_up(26);
    // gpio_pull_up(27);
    // gy511_init(i2c1);

    while (true) {
        // Read accelerometer data, process, and generate command
        // gy511_read_acceleration(i2c1, &raw_ax, &raw_ay, &raw_az);
        // process_acceleration(raw_ax, raw_ay, raw_az, &accel_data);
        // generate_control_command(&accel_data, command);

        // Only send command if it differs from the previous command
        // if (strcmp(command, last_command) != 0) {
        //     xMessageBufferSend(wifiMessageBuffer, &command, sizeof(command), 0);
        //     strncpy(last_command, command, sizeof(last_command) - 1);  // Update last command
        //     printf("Command: %s\n", command);
        // }

        snprintf(command, 50, "move forward at 25.0%% speed %d\n", counterPrint);
        xMessageBufferSend(wifiMessageBuffer, &command, sizeof(command), 0);
        printf("Command: %s\n", command);
        counterPrint++;

        // vTaskDelay(pdMS_TO_TICKS(400));
        vTaskDelay(pdMS_TO_TICKS(300)); 
    }
}