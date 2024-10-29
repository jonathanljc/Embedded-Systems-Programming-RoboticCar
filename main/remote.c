#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define WIFI_SSID "Felix-iPhone"
#define WIFI_PASSWORD "felixfelix"
#define SERVER_IP "172.20.10.4"
#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 2048

#define ACCEL_I2C_ADDR 0x19
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28
#define ACCEL_CONVERSION 0.00059841 // Convert to m/s² for ±2g
#define FILTER_SAMPLES 10           // Number of samples for moving average

// Circular buffers for moving average filter
float accel_x_buffer[FILTER_SAMPLES] = {0};
float accel_y_buffer[FILTER_SAMPLES] = {0};
float accel_z_buffer[FILTER_SAMPLES] = {0};
int buffer_index = 0;

// Struct to hold accelerometer data
typedef struct
{
    float x, y, z;
} AccelerometerData;

// TCP Client State
typedef struct
{
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    int sent_len;
    bool connected;
    bool complete;
} TCP_CLIENT_T;

TCP_CLIENT_T *tcp_client_state;

// Initialize GY-511 Accelerometer
void gy511_init(i2c_inst_t *i2c)
{
    uint8_t config[2];
    config[0] = CTRL_REG1_A;
    config[1] = 0x57; // Enable accelerometer, 100Hz, all axes
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, config, 2, false);
}

// Read raw acceleration data
void gy511_read_acceleration(i2c_inst_t *i2c, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    i2c_write_blocking(i2c, ACCEL_I2C_ADDR, (uint8_t[]){OUT_X_L_A | 0x80}, 1, true);
    i2c_read_blocking(i2c, ACCEL_I2C_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

// Moving average filter
void update_moving_average(float *buffer, float new_value)
{
    buffer[buffer_index] = new_value;
}

float calculate_average(float *buffer)
{
    float sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++)
    {
        sum += buffer[i];
    }
    return sum / FILTER_SAMPLES;
}

// Convert raw data to m/s² and apply smoothing
void process_acceleration(int16_t raw_ax, int16_t raw_ay, int16_t raw_az, AccelerometerData *accel_data)
{
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
void generate_control_command(const AccelerometerData *accel_data, char *command_buffer)
{
    float speed = sqrt(accel_data->x * accel_data->x + accel_data->y * accel_data->y) / 9.81; // Normalize to g-force
    speed = fmin(speed, 1.0) * 100;                                                           // Cap speed to 100%

    if (fabs(accel_data->y) > fabs(accel_data->x))
    {
        snprintf(command_buffer, 50, "turn %s at %.1f%% speed\n", accel_data->y > 0 ? "left" : "right", speed);
    }
    else
    {
        snprintf(command_buffer, 50, "move %s at %.1f%% speed\n", accel_data->x > 0 ? "forward" : "backward", speed);
    }
}

// TCP client functions

static err_t tcp_client_close(void *arg)
{
    tcp_client_state = (TCP_CLIENT_T *)arg;
    err_t err = ERR_OK;
    if (tcp_client_state->tcp_pcb != NULL)
    {
        tcp_arg(tcp_client_state->tcp_pcb, NULL);
        tcp_sent(tcp_client_state->tcp_pcb, NULL);
        tcp_recv(tcp_client_state->tcp_pcb, NULL);
        tcp_err(tcp_client_state->tcp_pcb, NULL);
        err = tcp_close(tcp_client_state->tcp_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(tcp_client_state->tcp_pcb);
            err = ERR_ABRT;
        }
        tcp_client_state->tcp_pcb = NULL;
    }
    return err;
}

static void tcp_client_err(void *arg, err_t err)
{
    if (err != ERR_ABRT)
    {
        DEBUG_printf("tcp_client_err %d\n", err);
        tcp_client_close(arg);
    }
}

err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    tcp_client_state = (TCP_CLIENT_T *)arg;
    if (err != ERR_OK)
    {
        DEBUG_printf("connect failed %d\n", err);
        return tcp_client_close(arg);
    }
    tcp_client_state->connected = true;
    DEBUG_printf("Connected to server\n");

    return ERR_OK;
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    tcp_client_state = (TCP_CLIENT_T *)arg;
    tcp_client_state->sent_len += len;
    return ERR_OK;
}

static bool tcp_client_open(void *arg)
{
    tcp_client_state = (TCP_CLIENT_T *)arg;
    DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&tcp_client_state->remote_addr), TCP_PORT);
    tcp_client_state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&tcp_client_state->remote_addr));
    if (!tcp_client_state->tcp_pcb)
    {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    tcp_arg(tcp_client_state->tcp_pcb, tcp_client_state);
    // tcp_poll(state->tcp_pcb, tcp_client_poll, 2); // Poll every 2 ticks
    tcp_sent(tcp_client_state->tcp_pcb, tcp_client_sent);
    tcp_err(tcp_client_state->tcp_pcb, tcp_client_err);

    tcp_client_state->sent_len = 0;
    tcp_client_state->complete = false;

    // Begin TCP connection to the server
    cyw43_arch_lwip_begin();
    err_t err = tcp_connect(tcp_client_state->tcp_pcb, &tcp_client_state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err == ERR_OK;
}

static TCP_CLIENT_T *tcp_client_init(void)
{
    tcp_client_state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!tcp_client_state)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    ip4addr_aton(SERVER_IP, &tcp_client_state->remote_addr);
    return tcp_client_state;
}

void setup_tcp_client()
{
    tcp_client_state = tcp_client_init();
    if (!tcp_client_state)
    {
        return;
    }
    if (!tcp_client_open(tcp_client_state))
    {
        tcp_client_close(tcp_client_state);
        return;
    }

    while (!tcp_client_state->complete)
    {
        // Wait for 1 second between sends
        // CHANGE TIMING HERE FOR SENDING MESSAGES
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    free(tcp_client_state);
}

// Magnetometer task for reading data and sending commands
void magnetometer_task(__unused void *params)
{
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

    while (true)
    {
        // Read accelerometer data, process, and generate command
        gy511_read_acceleration(i2c1, &raw_ax, &raw_ay, &raw_az);
        process_acceleration(raw_ax, raw_ay, raw_az, &accel_data);
        generate_control_command(&accel_data, command);

        if (tcp_client_state->connected)
        {
            tcp_write(tcp_client_state->tcp_pcb, command, strlen(command), TCP_WRITE_FLAG_COPY);
        }

        printf("Command: %s\n", command);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// WiFi task to initialize WiFi and setup TCP client
void wifi_task(__unused void *params)
{
    DEBUG_printf("Initialising WiFi...\n");
    if (cyw43_arch_init())
    {
        DEBUG_printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    int retries = 0;
    while (retries < 5)
    {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
        {
            printf("Failed to connect. Retrying... (%d/%d)\n", retries + 1, 5);
            retries++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            printf("Connected.\n");
            break;
        }
    }

    if (retries == 5)
    {
        printf("Failed to connect after 5 attempts.\n");
        return;
    }

    printf("Running TCP client test...\n");

    setup_tcp_client();

    cyw43_arch_deinit();
}

void vLaunch(void)
{
    TaskHandle_t magnetometertask;
    xTaskCreate(magnetometer_task, "MagnetometerTask", 256, NULL, 1, &magnetometertask);
    TaskHandle_t wifitask;
    xTaskCreate(wifi_task, "WiFiTask", 256, NULL, 2, &wifitask);
    vTaskStartScheduler();
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("Starting system\n");
    vLaunch();
    return 0;
}