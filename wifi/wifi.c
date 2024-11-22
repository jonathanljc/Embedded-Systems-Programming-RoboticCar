#include "wifi.h"


#define MQTT_QOS 1
u32_t data_in = 0;

u8_t buffer[512];
u8_t data_len = 0;

int counter = 0;
char *topic;

MQTT_CLIENT_T *mqtt_state;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void)
{
    mqtt_state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!mqtt_state)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    mqtt_state->received = 0;
    return mqtt_state;
}

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len)
{
    if (tot_len > 512)
    {
        DEBUG_printf("Message length exceeds buffer size, discarding");
    }
    else
    {
        data_in = tot_len;
        data_len = 0;
    }
}

// When data is received, this callback is called
static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags)
{
    if (data_in > 0)
    {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0)
        {
            buffer[data_len] = 0;
            // DEBUG_printf("WIFI: %s\n", &buffer);
            // Place the data into the message buffer
            char command[512];
            strncpy(command, (char *)&buffer, sizeof(buffer));
            xMessageBufferSend(wifiReceiveBuffer, command, strlen(command) + 1, 0);
        }
    }
}

// Callback for when a publish request is completed
void mqtt_pub_request_cb(void *arg, err_t err)
{
    
}

// Callback for when a subscribe/unsubscribe request is completed
void mqtt_sub_request_cb(void *arg, err_t err)
{
}

// Callback for when a connection is established
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    if (status != 0)
    {
        DEBUG_printf("Error during connection: err %d.\n", status);
    }
    else
    {
        DEBUG_printf("MQTT connected.\n");
    }
}

// Not needed, was from example code. Remains here just to refer
// err_t mqtt_test_publish(MQTT_CLIENT_T *mqtt_state)
// {
//     char buffer[256];

//     // sprintf(buffer, "{\"message\":\"hello from picow %d / %d\"}",
//     //         state->received, state->counter);
//     sprintf(buffer, "HELLO %d", counter);

//     err_t err;
//     u8_t qos = MQTT_QOS;
//     u8_t retain = 0;
//     cyw43_arch_lwip_begin();
//     // err = mqtt_publish(state->mqtt_client, "inf2004/p1c/remote", buffer,
//     //                    strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
//     cyw43_arch_lwip_end();

//     return err;
// }

// Connect to the MQTT server
err_t mqtt_test_connect(MQTT_CLIENT_T *mqtt_state, const char *topic)
{
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    if (strcmp(topic, "car") == 0){
        ci.client_id = "PicoWP1CCar";
    }else if (strcmp(topic, "remote") == 0){
        ci.client_id = "PicoWP1CRemote";
    }else if (strcmp(topic, "dashboard") == 0){
        ci.client_id = "PicoWP1CDashboard";
    }
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 0;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = MQTT_QOS;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(mqtt_state->mqtt_client, &(mqtt_state->remote_addr),
                              MQTT_SERVER_PORT, mqtt_connection_cb, mqtt_state,
                              client_info);

    if (err != ERR_OK)
    {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

// Run the MQTT test (but not in use right now)
void mqtt_run_test(MQTT_CLIENT_T *mqtt_statee)
{

    while (true)
    {
    }
}

// Subscribe to a topic
void subscribe_to_topic(const char *topic)
{
    if (strcmp(topic, "car") == 0)
    {
        mqtt_sub_unsub(mqtt_state->mqtt_client, "inf2004/p1c/remote", MQTT_QOS, mqtt_sub_request_cb, 0, 1);
        printf("Subscribed to inf2004/p1c/remote\n");
    }
    else if (strcmp(topic, "dashboard") == 0)
    {
        mqtt_sub_unsub(mqtt_state->mqtt_client, "inf2004/p1c/car", MQTT_QOS, mqtt_sub_request_cb, 0, 1);
        printf("Subscribed to inf2004/p1c/car\n");
    }
}

// Publish to a topic
void publish_to_topic(const char *topic, const char *message)
{
    err_t err;
    if (strcmp(topic, "car") == 0)
    {
        err = mqtt_publish(mqtt_state->mqtt_client, "inf2004/p1c/car", message, strlen(message), MQTT_QOS, 0, mqtt_pub_request_cb, 0);
    }
    else if (strcmp(topic, "remote") == 0)
    {
        err = mqtt_publish(mqtt_state->mqtt_client, "inf2004/p1c/remote", message, strlen(message), MQTT_QOS, 0, mqtt_pub_request_cb, 0);
    }

    if(err == ERR_OK){
        printf("Published %s\n", message);
    }else{
        printf("err: %d\n", err);
    };
    return;
}

// Main task for wifi. Starts wifi, connects to wifi, connects to MQTT server, subscribes to topic (if relevant), and then sends messages to the topic
void main_task(void *pvParameters)
{
    topic = (char *)pvParameters;

    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    cyw43_arch_wifi_connect_timeout_ms("Felix-iPhone", "felixfelix", CYW43_AUTH_WPA2_AES_PSK, 30000);
    printf("Connected.\n");

    DEBUG_printf("Pi IP at %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));

    mqtt_state = mqtt_client_init();
    ipaddr_aton(MQTT_SERVER_IP, &(mqtt_state->remote_addr));

    mqtt_state->mqtt_client = mqtt_client_new();
    mqtt_state->counter = 0;

    if (mqtt_state->mqtt_client == NULL)
    {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    while (mqtt_test_connect(mqtt_state, topic) != ERR_OK)
    {
        printf("attempting to connect...... \n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    absolute_time_t timeout = nil_time;
    bool subscribed = false;
    mqtt_set_inpub_callback(mqtt_state->mqtt_client, mqtt_pub_start_cb,
                            mqtt_pub_data_cb, 0);

    while (1)
    {
        if (mqtt_client_is_connected(mqtt_state->mqtt_client))
        {
            // cyw43_arch_lwip_begin();

            if (!subscribed && (strcmp(topic, "dashboard") == 0 || strcmp(topic, "car") == 0))
            {
                subscribe_to_topic(topic);
                subscribed = true;
            }

            if (strcmp(topic, "remote") == 0)
            {
                char command[50];
                if (xMessageBufferReceive(wifiMessageBuffer, &command, sizeof(command), portMAX_DELAY) > 0)
                {
                    publish_to_topic(topic, command);
                    // vTaskDelay(pdMS_TO_TICKS(250));
                    vTaskDelay(pdMS_TO_TICKS(75));
                }
            }

            // This code is for when integrating to the 3 Picos for week 13
            // if (strcmp(topic, "car") == 0)
            // {
            //     publish_to_topic(topic, "Hello from car to dashboard");
            // }
            // else if (strcmp(topic, "remote") == 0)
            // {
            //     publish_to_topic(topic, "Hello from remote to car");
            //     counter++;
            //     vTaskDelay(pdMS_TO_TICKS(50));
            // }
            // cyw43_arch_lwip_end();
        }
        else
        {
            subscribed = false;
            printf("MQTT not connected\n");
            mqtt_test_connect(mqtt_state, topic);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    cyw43_arch_deinit();
}