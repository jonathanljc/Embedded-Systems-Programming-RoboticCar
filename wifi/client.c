/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// For testing dummy data (temperature sensor)
#include "hardware/adc.h"

#define WIFI_SSID "Felix-iPhone"
#define WIFI_PASSWORD "felixfelix"
#define TEST_TCP_SERVER_IP "172.20.10.13"
#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 2048

#define HELLO_MESSAGE "hello"
#define SEND_INTERVAL_MS 1000 // 1 second

typedef struct TCP_CLIENT_T_
{
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    int sent_len;
    bool complete;
    int run_count;
    bool connected;
    struct tcp_pcb *client_pcb;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int recv_len;
} TCP_CLIENT_T;

// Function to read onboard temperature
float read_onboard_temperature()
{
    const float conversion_factor = 3.3f / (1 << 12); // ADC conversion factor
    uint16_t adc_value = adc_read();                  // Read raw ADC value
    float voltage = adc_value * conversion_factor;
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f; // Convert to Celsius
    return temperature;
}

static err_t tcp_client_close(void *arg)
{
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    err_t err = ERR_OK;
    if (state->tcp_pcb != NULL)
    {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK)
        {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }
    return err;
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    // DEBUG_printf("tcp_client_sent %u\n", len);
    state->sent_len += len;
    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    if (err != ERR_OK)
    {
        DEBUG_printf("connect failed %d\n", err);
        return tcp_client_close(arg);
    }
    state->connected = true;
    DEBUG_printf("Connected to server. Sending 'hello' every 1 second\n");
    return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
    // DEBUG_printf("tcp_client_poll\n");

    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    // Send "hello" message
    const char *message = HELLO_MESSAGE;
    err_t err = tcp_write(tpcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK)
    {
        DEBUG_printf("Failed to send 'hello' %d\n", err);
        return tcp_client_close(arg);
    }
    DEBUG_printf("Sent: %s\n", message);

    return ERR_OK;
}

static void tcp_client_err(void *arg, err_t err)
{
    if (err != ERR_ABRT)
    {
        DEBUG_printf("tcp_client_err %d\n", err);
        tcp_client_close(arg);
    }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    if (!p)
    {
        return tcp_client_close(arg); // Close connection if the client disconnects.
    }

    cyw43_arch_lwip_check();
    if (p->tot_len > 0)
    {
        // DEBUG_printf("Received %d bytes\n", p->tot_len);

        // Receive the buffer
        int received_len = pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        state->buffer_recv[received_len] = '\0'; // Null-terminate the string

        // Remove any trailing \r\n (carriage return/newline) characters
        while (received_len > 0 && (state->buffer_recv[received_len - 1] == '\r' || state->buffer_recv[received_len - 1] == '\n'))
        {
            state->buffer_recv[received_len - 1] = '\0';
            received_len--;
        }

        // Only print if there's remaining content after trimming
        if (received_len > 0)
        {
            DEBUG_printf("Received data: %s\n", state->buffer_recv);
        }
        //  else {
        //     // DEBUG_printf("Received only newline characters, ignoring...\n");
        // }

        tcp_recved(tpcb, p->tot_len); // Acknowledge that data was received
    }
}

    static bool tcp_client_open(void *arg)
    {
        TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
        DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
        state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
        if (!state->tcp_pcb)
        {
            DEBUG_printf("failed to create pcb\n");
            return false;
        }

        tcp_arg(state->tcp_pcb, state);
        tcp_poll(state->tcp_pcb, tcp_client_poll, 2); // Poll every 2 ticks
        tcp_sent(state->tcp_pcb, tcp_client_sent);
        tcp_err(state->tcp_pcb, tcp_client_err);
        tcp_recv(state->tcp_pcb, tcp_client_recv);

        state->sent_len = 0;
        state->complete = false;

        // Begin TCP connection to the server
        cyw43_arch_lwip_begin();
        err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
        cyw43_arch_lwip_end();

        return err == ERR_OK;
    }

    // Perform initialization
    static TCP_CLIENT_T *tcp_client_init(void)
    {
        TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
        if (!state)
        {
            DEBUG_printf("failed to allocate state\n");
            return NULL;
        }
        ip4addr_aton(TEST_TCP_SERVER_IP, &state->remote_addr);
        return state;
    }

    void run_tcp_client_test(void)
    {
        TCP_CLIENT_T *state = tcp_client_init();
        if (!state)
        {
            return;
        }
        if (!tcp_client_open(state))
        {
            tcp_client_close(state);
            return;
        }

        while (!state->complete)
        {
            // Depending on whether pico_cyw43_arch_poll is being used, either poll or sleep.
#if PICO_CYW43_ARCH_POLL
            cyw43_arch_poll();
            cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
#else
            sleep_ms(SEND_INTERVAL_MS); // Wait for 1 second between sends
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
            DEBUG_printf("failed to initialise\n");
            return 1;
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
                sleep_ms(1000);
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
            return 1;
        }

        run_tcp_client_test();
        cyw43_arch_deinit();
        return 0;
    }
