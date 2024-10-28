// wheel_encoder.c
#include "encoder.h"

// Global variable definitions
volatile uint32_t left_notch_count = 0;
volatile uint32_t right_notch_count = 0;
volatile bool left_data_ready = false;
volatile bool right_data_ready = false;

// Internal timing variables
static volatile uint32_t left_last_rising_time = 0;
static volatile uint32_t right_last_rising_time = 0;
static volatile uint32_t left_last_falling_time = 0;
static volatile uint32_t right_last_falling_time = 0;
static uint32_t left_pulse_width = 0;
static uint32_t right_pulse_width = 0;

TaskHandle_t leftPulseTaskHandle = NULL;
TaskHandle_t rightPulseTaskHandle = NULL;
TaskHandle_t leftSpeedTaskHandle;
TaskHandle_t rightSpeedTaskHandle;
MessageBufferHandle_t leftMessageBuffer;
MessageBufferHandle_t rightMessageBuffer;

// Interrupt callback function for left and right wheels
void gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (gpio == LEFT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            left_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            left_last_falling_time = current_time;
            left_notch_count++;
            left_data_ready = true;
        }
    } else if (gpio == RIGHT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            right_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            right_last_falling_time = current_time;
            right_notch_count++;
            right_data_ready = true;
        }
    }
}

// Pulse Width Calculation Task for each wheel
void pulse_width_task(void *pvParameters) {
    uint32_t *pulse_width_ptr = (uint32_t *)pvParameters;

    while (1) {
        if (left_data_ready && pulse_width_ptr == &left_pulse_width) {
            left_data_ready = false;
            left_pulse_width = left_last_falling_time - left_last_rising_time;
            xTaskNotifyGive(leftSpeedTaskHandle);
        }

        if (right_data_ready && pulse_width_ptr == &right_pulse_width) {
            right_data_ready = false;
            right_pulse_width = right_last_falling_time - right_last_rising_time;
            xTaskNotifyGive(rightSpeedTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Speed and Distance Calculation Task for each wheel
void speed_task(void *pvParameters) {
    uint32_t *pulse_width_ptr = (uint32_t *)pvParameters;
    MessageBufferHandle_t messageBuffer = (MessageBufferHandle_t)pvParameters;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (*pulse_width_ptr > 0) {
            float pulse_width_sec = *pulse_width_ptr / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;
            float total_distance;

            PulseData_t data;
            if (pulse_width_ptr == &left_pulse_width) {
                total_distance = left_notch_count * DISTANCE_PER_NOTCH;
                data.notch_count = left_notch_count;
                data.pulse_width = left_pulse_width;
            } else {
                total_distance = right_notch_count * DISTANCE_PER_NOTCH;
                data.notch_count = right_notch_count;
                data.pulse_width = right_pulse_width;
            }

            data.speed = speed;
            data.total_distance = total_distance;

            xMessageBufferSend(messageBuffer, &data, sizeof(data), portMAX_DELAY);
        }
    }
}

// Logging Task for each wheel
void log_task(void *pvParameters) {
    MessageBufferHandle_t messageBuffer = (MessageBufferHandle_t)pvParameters;

    while (1) {
        PulseData_t received_data;
        xMessageBufferReceive(messageBuffer, &received_data, sizeof(received_data), portMAX_DELAY);

        printf("Notch Count: %u, Pulse Width: %u us, Speed: %.3f m/s, Total Distance: %.3f m\n",
               received_data.notch_count, received_data.pulse_width, received_data.speed, received_data.total_distance);
    }
}
