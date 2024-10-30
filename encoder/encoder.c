#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/time.h"
#include "message_buffer.h"

#define LEFT_WHEEL_ENCODER_PIN 26
#define RIGHT_WHEEL_ENCODER_PIN 8
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define MICROSECONDS_IN_A_SECOND 1000000.0

typedef struct {
    uint32_t pulse_width;
    float speed;
    float total_distance;
    uint32_t notch_count;
} PulseData_t;

// Variables for the left encoder
static volatile uint32_t left_notch_count = 0;
static volatile uint32_t left_last_rising_time = 0;
static volatile uint32_t left_last_falling_time = 0;
static uint32_t left_pulse_width = 0;

// Variables for the right encoder
static volatile uint32_t right_notch_count = 0;
static volatile uint32_t right_last_rising_time = 0;
static volatile uint32_t right_last_falling_time = 0;
static uint32_t right_pulse_width = 0;

TaskHandle_t leftPulseTaskHandle = NULL;
TaskHandle_t leftSpeedTaskHandle;
TaskHandle_t rightPulseTaskHandle = NULL;
TaskHandle_t rightSpeedTaskHandle;
MessageBufferHandle_t leftMessageBuffer;
MessageBufferHandle_t rightMessageBuffer;
MessageBufferHandle_t leftMotorControlBuffer;  // New motor control buffer
MessageBufferHandle_t rightMotorControlBuffer;

// Unified interrupt callback
void encoder_gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (gpio == LEFT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            left_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            left_last_falling_time = current_time;
            left_notch_count++;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(leftPulseTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else if (gpio == RIGHT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            right_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            right_last_falling_time = current_time;
            right_notch_count++;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(rightPulseTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// Task for left pulse width calculation
void left_pulse_width_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        left_pulse_width = left_last_falling_time - left_last_rising_time;
        xTaskNotifyGive(leftSpeedTaskHandle);
    }
}

// Task for right pulse width calculation
void right_pulse_width_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        right_pulse_width = right_last_falling_time - right_last_rising_time;
        xTaskNotifyGive(rightSpeedTaskHandle);
    }
}

// Task for left wheel speed and distance calculation
void left_speed_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (left_pulse_width > 0) {
            float pulse_width_sec = left_pulse_width / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;
            float total_distance = left_notch_count * DISTANCE_PER_NOTCH;
            PulseData_t data = { left_pulse_width, speed, total_distance, left_notch_count };
            // Send data to the motor control buffer
            xMessageBufferSend(leftMotorControlBuffer, &data, sizeof(data), portMAX_DELAY);
            xMessageBufferSend(leftMessageBuffer, &data, sizeof(data), portMAX_DELAY);
            
        }
    }
}

// Task for right wheel speed and distance calculation
void right_speed_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (right_pulse_width > 0) {
            float pulse_width_sec = right_pulse_width / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;
            float total_distance = right_notch_count * DISTANCE_PER_NOTCH;
            PulseData_t data = { right_pulse_width, speed, total_distance, right_notch_count };
            // Send data to the motor control buffer
            xMessageBufferSend(rightMotorControlBuffer, &data, sizeof(data), portMAX_DELAY);
            xMessageBufferSend(rightMessageBuffer, &data, sizeof(data), portMAX_DELAY);
            
        }
    }
}

// Task for logging left wheel data
void left_log_task(void *pvParameters) {
    while (1) {
        PulseData_t received_data;
        xMessageBufferReceive(leftMessageBuffer, &received_data, sizeof(received_data), portMAX_DELAY);
        printf("Left Wheel -> Notch Count: %u, Pulse Width: %u us, Speed: %.3f m/s, Total Distance: %.3f m\n",
               received_data.notch_count, received_data.pulse_width, received_data.speed, received_data.total_distance);
    }
}

// Task for logging right wheel data
void right_log_task(void *pvParameters) {
    while (1) {
        PulseData_t received_data;
        xMessageBufferReceive(rightMessageBuffer, &received_data, sizeof(received_data), portMAX_DELAY);
        printf("Right Wheel -> Notch Count: %u, Pulse Width: %u us, Speed: %.3f m/s, Total Distance: %.3f m\n",
               received_data.notch_count, received_data.pulse_width, received_data.speed, received_data.total_distance);
    }
}

int main() {
    stdio_init_all();

    // Initialize GPIO pins and attach the same callback for both pins
    gpio_init(LEFT_WHEEL_ENCODER_PIN);
    gpio_set_dir(LEFT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_WHEEL_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);

    gpio_init(RIGHT_WHEEL_ENCODER_PIN);
    gpio_set_dir(RIGHT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_WHEEL_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);

    // Create message buffers
    leftMotorControlBuffer = xMessageBufferCreate(256);
    rightMotorControlBuffer = xMessageBufferCreate(256);
    leftMessageBuffer = xMessageBufferCreate(256);
    rightMessageBuffer = xMessageBufferCreate(256);

    // Create tasks
    xTaskCreate(left_pulse_width_task, "Left Pulse Width Task", 1024, NULL, 1, &leftPulseTaskHandle);
    xTaskCreate(left_speed_task, "Left Speed Task", 1024, NULL, 1, &leftSpeedTaskHandle);
    xTaskCreate(left_log_task, "Left Log Task", 1024, NULL, 1, NULL);

    xTaskCreate(right_pulse_width_task, "Right Pulse Width Task", 1024, NULL, 1, &rightPulseTaskHandle);
    xTaskCreate(right_speed_task, "Right Speed Task", 1024, NULL, 1, &rightSpeedTaskHandle);
    xTaskCreate(right_log_task, "Right Log Task", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}
