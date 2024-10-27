/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 2048
#define POLL_TIME_S 5
#define WIFI_SSID "SHARKY0"
#define WIFI_PASSWORD "B204491DR"

typedef struct TCP_SERVER_T_
{
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
} TCP_SERVER_T;

static void tcp_server_err(void *arg, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_server_close(void *arg);
static bool tcp_server_open(TCP_SERVER_T *state);

static TCP_SERVER_T *tcp_server_init(void)
{
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    return state;
}

static err_t tcp_server_close(void *arg)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL)
    {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (!p)
    {
        DEBUG_printf("Client disconnected\n");
        tcp_server_close(state);  // Close client connection
        return ERR_OK;  // Stay in listening mode without re-binding
    }

    cyw43_arch_lwip_check();
    if (p->tot_len > 0)
    {
        int received_len = pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        state->buffer_recv[received_len] = '\0'; // Null-terminate the string

        while (received_len > 0 && (state->buffer_recv[received_len - 1] == '\r' || state->buffer_recv[received_len - 1] == '\n'))
        {
            state->buffer_recv[received_len - 1] = '\0';
            received_len--;
        }

        if (received_len > 0)
        {
            DEBUG_printf("Received data: %s\n", state->buffer_recv);
        }
        tcp_recved(tpcb, p->tot_len); // Acknowledge that data was received
    }

    pbuf_free(p); // Free the packet buffer
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    DEBUG_printf("tcp_server_poll_fn\n");
    return ERR_OK;
}

static void tcp_server_err(void *arg, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (err != ERR_ABRT)
    {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_server_close(state); // Close client connection only
    }
}

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    cyw43_arch_lwip_check();
    const char *welcomeMessage = "Welcome Client";
    err_t err = tcp_write(tpcb, welcomeMessage, strlen(welcomeMessage), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        DEBUG_printf("Failed to write data %d\n", err);
        return false;
    }
    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err)
{
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;
    if (err != ERR_OK || client_pcb == NULL)
    {
        DEBUG_printf("Failure in accept\n");
        tcp_server_close(state);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_err(client_pcb, tcp_server_err);

    return tcp_server_send_data(arg, state->client_pcb);
}

static bool tcp_server_open(TCP_SERVER_T *state)
{
    DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb)
    {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err)
    {
        DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb)
    {
        DEBUG_printf("failed to listen\n");
        tcp_close(pcb);
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

void run_tcp_server(void)
{
    TCP_SERVER_T *state = tcp_server_init();
    if (!state)
    {
        return;
    }
    if (!tcp_server_open(state))
    {
        tcp_server_close(state);
        return;
    }
    while (!state->complete)
    {
#if PICO_CYW43_ARCH_POLL
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
#else
        sleep_ms(1000);
#endif
    }
    free(state);
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
        sleep_ms(100);

    printf("Starting PICO!\n");
    printf("Starting WiFi...\n");

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
        return 1;
    }
    else
    {
        printf("Connected.\n");
    }
    run_tcp_server();
    cyw43_arch_deinit();
    return 0;
}
