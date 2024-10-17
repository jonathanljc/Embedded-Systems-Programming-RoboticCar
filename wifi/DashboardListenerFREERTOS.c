#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"

#define WIFI_SSID "Felix-iPhone"
#define WIFI_PASS "felixfelix"
#define LISTEN_PORT 1111

// TODO: create print task so when receiving data, it will print to the console


// Forward declaration of the callback functions
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Task handles
TaskHandle_t wifi_task_handle = NULL;
TaskHandle_t tcp_server_task_handle = NULL;

// Initialize the TCP server
void tcp_server_init()
{
    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, LISTEN_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_accept_callback);
}

// Callback function to handle received data
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        // Connection closed by the remote side
        printf("Connection closed\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Check if the payload length is greater than zero before printing
    if (p->len > 0)
    {
        char *message = (char *)p->payload;
        char sanitized_message[p->len + 1]; // +1 for null terminator
        int sanitized_len = 0;

        // Sanitize the message by filtering out non-printable characters
        for (int i = 0; i < p->len; i++)
        {
            if (message[i] >= 32 && message[i] <= 126)
            {
                sanitized_message[sanitized_len++] = message[i];
            }
        }
        sanitized_message[sanitized_len] = '\0'; // Null-terminate the sanitized message

        printf("%s\n", sanitized_message);
    }

    // Free the received pbuf
    pbuf_free(p);

    return ERR_OK;
}

// Callback function to handle new connections
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, tcp_recv_callback);
    return ERR_OK;
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
        printf("failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
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

    // Initialize the TCP server
    tcp_server_init();

    // Delete the task as it is no longer needed
    vTaskDelete(NULL);
}

// Task for handling the main loop
void main_loop_task(void *pvParameters)
{
    while (true)
    {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main()
{
    // Create the WiFi task
    xTaskCreate(wifi_task, "WiFi Task", 1024, NULL, 1, &wifi_task_handle);

    // Create the main loop task
    // xTaskCreate(main_loop_task, "Main Loop Task", 1024, NULL, 1, &tcp_server_task_handle);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true)
    {
    }

    return 0;
}