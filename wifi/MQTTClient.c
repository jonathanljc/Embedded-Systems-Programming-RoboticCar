#include "MQTTClient.h"

u32_t data_in = 0;

u8_t buffer[1025];
u8_t data_len = 0;

int counter = 0;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void) {
    MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    return state;
}

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len) {
    DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > 1024) {
        DEBUG_printf("Message length exceeds buffer size, discarding");
    } else {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags) {
    if (data_in > 0) {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0) {
            buffer[data_len] = 0;
            DEBUG_printf("Message received: %s\n", &buffer);
        }
    }
}

void mqtt_pub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
    // DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
    state->received++;
}

void mqtt_sub_request_cb(void *arg, err_t err) {
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status) {
    if (status != 0) {
        DEBUG_printf("Error during connection: err %d.\n", status);
    } else {
        DEBUG_printf("MQTT connected.\n");
    }
}

err_t mqtt_test_publish(MQTT_CLIENT_T *state) {
    char buffer[128];

    // sprintf(buffer, "{\"message\":\"hello from picow %d / %d\"}",
    //         state->received, state->counter);
    sprintf(buffer, "HELLO %d", counter);

    err_t err;
    u8_t qos = 0; /* 0 1 or 2, see MQTT specification.  AWS IoT does not support
                     QoS 2 */
    u8_t retain = 0;
    cyw43_arch_lwip_begin();
    err = mqtt_publish(state->mqtt_client, "inf2004/p1c", buffer,
                       strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();
    counter++;
    // if (err != ERR_OK) {
    //     DEBUG_printf("Publish err: %d\n", err);
    // }

    return err;
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "PicoW";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 0;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr),
                              MQTT_SERVER_PORT, mqtt_connection_cb, state,
                              client_info);

    if (err != ERR_OK) {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void mqtt_run_test(MQTT_CLIENT_T *state) {

    while (true) {
    }
}

void main_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    cyw43_arch_wifi_connect_timeout_ms("Felix-iPhone", "felixfelix", CYW43_AUTH_WPA2_AES_PSK, 30000);
    printf("Connected.\n");

    DEBUG_printf("Pi IP at %s on port\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));

    MQTT_CLIENT_T *state = mqtt_client_init();
    ipaddr_aton(MQTT_SERVER_IP, &(state->remote_addr));

    state->mqtt_client = mqtt_client_new();
    state->counter = 0;

    if (state->mqtt_client == NULL) {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    while (mqtt_test_connect(state) != ERR_OK) {
        printf("attempting to connect...... \n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    absolute_time_t timeout = nil_time;
    bool subscribed = false;
    mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb,
                            mqtt_pub_data_cb, 0);

    while (1) {
        if (mqtt_client_is_connected(state->mqtt_client)) {
            cyw43_arch_lwip_begin();

            if (!subscribed) {
                mqtt_sub_unsub(state->mqtt_client, "pico_w/recv", 0,
                               mqtt_sub_request_cb, 0, 1);
                subscribed = true;
            }

            if (mqtt_test_publish(state) == ERR_OK) {
                if (state->counter != 0) {
                    // DEBUG_printf(
                    //     "Compass Heading: %.2f°    Pitch: %.2f Roll: %.2f\n",
                    //     global_heading, global_pitch, global_roll);
                }
                timeout = make_timeout_time_ms(5000);
                state->counter++;
            } // else ringbuffer is full and we need to wait for
              // messages to flush.
            cyw43_arch_lwip_end();
        } else {
            // DEBUG_printf(".");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    }

    cyw43_arch_deinit();
}