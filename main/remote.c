#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/tcp.h"

#define WIFI_SSID "Ong"
#define WIFI_PASSWORD "12345678"
#define SERVER_IP "192.168.1.84"
#define TCP_PORT 4242

#define ACCEL_I2C_ADDR 0x19
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28
#define ACCEL_CONVERSION 0.00059841 // Convert to m/s² for ±2g
#define FILTER_SAMPLES 10            // Number of samples for moving average

// Circular buffers for moving average filter
float accel_x_buffer[FILTER_SAMPLES] = {0};
float accel_y_buffer[FILTER_SAMPLES] = {0};
float accel_z_buffer[FILTER_SAMPLES] = {0};
int buffer_index = 0;

// Struct to hold accelerometer data
typedef struct {
    float x, y, z;
} AccelerometerData;

// TCP Client State
typedef struct {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    int sent_len;
    bool connected;
} TCP_CLIENT_T;

TCP_CLIENT_T *tcp_client_state;

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
// Generate control command based on accelerometer data
void generate_control_command(const AccelerometerData *accel_data, char *command_buffer) {
    float speed = sqrt(accel_data->x * accel_data->x + accel_data->y * accel_data->y) / 9.81; // Normalize to g-force
    speed = fmin(speed, 1.0) * 100; // Cap speed to 100%

    if (fabs(accel_data->y) > fabs(accel_data->x)) {
        snprintf(command_buffer, 50, "turn %s at %.1f%% speed", accel_data->y > 0 ? "left" : "right", speed);
    } else {
        snprintf(command_buffer, 50, "move %s at %.1f%% speed", accel_data->x > 0 ? "forward" : "backward", speed);
    }
}


// TCP client functions
void tcp_client_err(void *arg, err_t err) {
    tcp_client_state->connected = false;
}

err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    return ERR_OK;
}

err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err == ERR_OK) {
        tcp_client_state->connected = true;
    }
    return ERR_OK;
}

void setup_tcp_client() {
    tcp_client_state = (TCP_CLIENT_T *)calloc(1, sizeof(TCP_CLIENT_T));
    ip4addr_aton(SERVER_IP, &tcp_client_state->remote_addr);
    tcp_client_state->tcp_pcb = tcp_new();
    tcp_connect(tcp_client_state->tcp_pcb, &tcp_client_state->remote_addr, TCP_PORT, tcp_client_connected);
}

// Magnetometer task for reading data and sending commands
void magnetometer_task(__unused void *params) {
    AccelerometerData accel_data;
    int16_t raw_ax, raw_ay, raw_az;
    char command[50];

    // Initialize I2C and GY-511 sensor
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);
    gy511_init(i2c1);

    while (true) {
        // Read accelerometer data, process, and generate command
        gy511_read_acceleration(i2c1, &raw_ax, &raw_ay, &raw_az);
        process_acceleration(raw_ax, raw_ay, raw_az, &accel_data);
        generate_control_command(&accel_data, command);

        if (tcp_client_state->connected) {
            tcp_write(tcp_client_state->tcp_pcb, command, strlen(command), TCP_WRITE_FLAG_COPY);
        }

        printf("Command: %s\n", command);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// WiFi task to initialize WiFi and setup TCP client
void wifi_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("CYW43 WiFi initialization failed.\n");
        vTaskDelete(NULL); // Terminate the task if initialization fails
        return;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) {
        printf("Wi-Fi connection failed.\n");
        cyw43_arch_deinit(); // Deinitialize WiFi if connection fails
        vTaskDelete(NULL);   // Terminate the task
        return;
    }
    printf("Wi-Fi connected\n");

    setup_tcp_client();
    vTaskDelay(pdMS_TO_TICKS(5000));
    cyw43_arch_deinit();
}

void vLaunch(void) {
    xTaskCreate(magnetometer_task, "MagnetometerTask", 256, NULL, 1, NULL);
    xTaskCreate(wifi_task, "WiFiTask", 256, NULL, 2, NULL);
    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        tight_loop_contents();
    }
    printf("Starting system\n");
    vLaunch();
    return 0;
}