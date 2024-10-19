#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "message_buffer.h"

// For testing dummy data (temperature sensor)
#include "hardware/adc.h"

#define WIFI_SSID "Felix-iPhone"
#define WIFI_PASS "felixfelix"
#define SERVER_IP "172.20.10.12" // Replace with the IP address of your TCP server
#define SERVER_PORT 1111

// Task handles
TaskHandle_t wifi_task_handle = NULL;
TaskHandle_t temp_task_handle = NULL;

static MessageBufferHandle_t xControlMessageBuffer;
#define MESSAGE_BUFFER_SIZE 100

// Function to read onboard temperature
float read_onboard_temperature()
{
    const float conversion_factor = 3.3f / (1 << 12); // ADC conversion factor
    uint16_t adc_value = adc_read(); // Read raw ADC value
    float voltage = adc_value * conversion_factor;
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f; // Convert to Celsius
    return temperature;
}

// Forward declaration of the TCP client connection function
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_client_connect();
struct tcp_pcb *client_pcb = NULL;

// Callback function for when the client connects to the server
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err != ERR_OK)
    {
        printf("Connection failed with error: %d\n", err);
        tcp_close(tpcb);
        return err;
    }

    printf("Connected to server!\n");
    tcp_sent(tpcb, tcp_client_sent); // Register the sent callback

    // After connection, keep sending temperature data in a loop
    float temperature;
    size_t xReceivedBytes;
    while (true)
    {
        // Retrieve temperature data from the message buffer
        xReceivedBytes = xMessageBufferReceive(xControlMessageBuffer, &temperature, sizeof(temperature), pdMS_TO_TICKS(2000));
        
        if (xReceivedBytes > 0)
        {
            // Format and send the temperature as a message
            char message[50];
            snprintf(message, sizeof(message), "Temperature: %.2f C", temperature);
            printf("Sending message: %s\n", message);
            
            err_t result = tcp_write(tpcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
            if (result != ERR_OK)
            {
                printf("Failed to send message: %d\n", result);
                tcp_close(tpcb);
                return result;
            }

            tcp_output(tpcb); // Ensure the message is sent
        }
    }
}

// Callback function for when data has been successfully sent
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    printf("Message sent successfully!\n");
    return ERR_OK;
}

// Function to initiate TCP client connection
void tcp_client_connect()
{
    client_pcb = tcp_new(); // Create a new PCB

    if (client_pcb == NULL)
    {
        printf("Failed to create TCP control block\n");
        return;
    }

    ip4_addr_t server_ip;
    ip4addr_aton(SERVER_IP, &server_ip);

    // Initiate connection
    tcp_connect(client_pcb, &server_ip, SERVER_PORT, tcp_client_connected);
}

// Task for initializing WiFi
void wifi_task(void *pvParameters)
{
    stdio_init_all();
    while (!stdio_usb_connected())
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for USB to initialize

    printf("Starting PICO!\n");
    printf("Starting WiFi...\n");

    if (cyw43_arch_init())
    {
        printf("Failed to initialize\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect.\n");
        exit(1);
    }
    else
    {
        printf("Connected to %s.\n", WIFI_SSID);
    }

    // Get and print the IP address
    struct netif *netif = netif_list;
    if (netif_is_up(netif))
    {
        printf("Pico IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
    }
    else
    {
        printf("Failed to get IP address\n");
    }

    // Initialize the TCP client connection
    tcp_client_connect();

    // Delete the task as it is no longer needed
    vTaskDelete(NULL);
}

// Task to continuously read temperature and send to the message buffer
void temp_task(__unused void *params)
{
    float temperature = 0.0;

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second between reads
        temperature = read_onboard_temperature();
        xMessageBufferSend(xControlMessageBuffer, &temperature, sizeof(temperature), 0); // Send to the buffer
    }
}

int main()
{
    // Create the message buffer for temperature data
    xControlMessageBuffer = xMessageBufferCreate(MESSAGE_BUFFER_SIZE);

    // Create the WiFi task
    xTaskCreate(wifi_task, "WiFi Task", 1024, NULL, 1, &wifi_task_handle);

    // Create the temperature reading task
    xTaskCreate(temp_task, "Temperature Task", 1024, NULL, 1, &temp_task_handle);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true)
    {
    }

    return 0;
}
