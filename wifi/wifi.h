#ifndef WIFI_H
#define WIFI_H

// MQTT Headers
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "lwip/api.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

// MQTT Definitions
#define MQTT_SERVER_IP "192.168.222.40"
#define MQTT_SSID "CPH1719"
#define MQTT_PW "12345678"
// #define MQTT_SERVER_IP "5.196.78.28"
#define MQTT_SERVER_PORT 1883

#define DEBUG_printf printf

extern MessageBufferHandle_t wifiMessageBuffer;
extern MessageBufferHandle_t wifiReceiveBuffer;

extern bool disconnectRemote;

typedef struct {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

void main_task(void *pvParameters);
void publish_to_topic(const char *topic, const char *message);

#endif