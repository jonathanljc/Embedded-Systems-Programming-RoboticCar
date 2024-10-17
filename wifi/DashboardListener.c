#include <stdio.h>
#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"

#define WIFI_SSID "Felix-iPhone"
#define WIFI_PASS "felixfelix"
#define LISTEN_PORT 1111

// Forward declaration of the callback functions
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Initialize the TCP server
void tcp_server_init() {
    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, LISTEN_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_accept_callback);
}

// Callback function to handle received data
static err_t tcp_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Check if the payload length is greater than zero before printing
    if (p->len > 0) {
        printf("Received: %.*s\n", p->len, (char *)p->payload);
    }

    // Free the received pbuf
    pbuf_free(p);

    return ERR_OK;
}

// Callback function to handle new connections
static err_t tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, tcp_recv_callback);
    return ERR_OK;
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(100); // Wait for USB to initialize

    printf("Starting PICO!\n");
    printf("Starting WiFi...\n");

    // Initialize the WiFi module
    int result = cyw43_arch_init();
    if (result) {
        printf("WiFi init failed with error code: %d\n", result);
        return -1;
    }

    printf("Connecting to WiFi...\n");
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("WiFi connection failed\n");
        return -1;
    }

    printf("WiFi connected\n");

    // Get and print the IP address
    struct netif *netif = netif_list;
    if (netif_is_up(netif)) {
        printf("Pico IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
    } else {
        printf("Failed to get IP address\n");
    }

    // Initialize the TCP server
    tcp_server_init();

    // Main loop
    while (true) {
        cyw43_arch_poll();
        sleep_ms(1000);
    }

    return 0;
}