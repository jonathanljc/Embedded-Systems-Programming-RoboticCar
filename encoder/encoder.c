#include "encoder.h"
#include "ultrasonic.h"

// Encoder variables for the left encoder
static volatile uint32_t left_notch_count = 0;
static volatile uint32_t left_last_rising_time = 0;
static volatile uint32_t left_last_falling_time = 0;
static uint32_t left_pulse_width = 0;

// Encoder variables for the right encoder
static volatile uint32_t right_notch_count = 0;
static volatile uint32_t right_last_rising_time = 0;
static volatile uint32_t right_last_falling_time = 0;
static uint32_t right_pulse_width = 0;


// Add these variables to keep track of previous speeds
static float left_speed_buffer[AVERAGE_FILTER_SIZE] = {0};
static float right_speed_buffer[AVERAGE_FILTER_SIZE] = {0};
static int left_buffer_index = 0;
static int right_buffer_index = 0;

// Task handles
TaskHandle_t leftPulseTaskHandle = NULL;
TaskHandle_t leftSpeedTaskHandle;
TaskHandle_t rightPulseTaskHandle = NULL;
TaskHandle_t rightSpeedTaskHandle;

MessageBufferHandle_t leftMessageBuffer;
MessageBufferHandle_t rightMessageBuffer;



// Unified interrupt callback for both ultrasonic and encoder pins
void unified_gpio_callback(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Check if the interrupt is for the ultrasonic echo pin
    if (gpio == ECHOPIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            // Rising edge for ultrasonic echo pin: start the timer
            start_time = get_absolute_time();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            // Falling edge for ultrasonic echo pin: calculate the pulse width
            pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
        }
    }
    // Check if the interrupt is for the left wheel encoder pin
    else if (gpio == LEFT_WHEEL_ENCODER_PIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            left_last_rising_time = current_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            left_last_falling_time = current_time;
            left_notch_count++;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(leftPulseTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    // Check if the interrupt is for the right wheel encoder pin
    else if (gpio == RIGHT_WHEEL_ENCODER_PIN) {
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

float calculate_moving_average(float buffer[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}


// Task for left wheel speed and distance calculation
void left_speed_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (left_pulse_width > MIN_PULSE_WIDTH) {
            float pulse_width_sec = left_pulse_width / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;

            // Update the buffer and calculate the moving average
            left_speed_buffer[left_buffer_index] = speed;
            left_buffer_index = (left_buffer_index + 1) % AVERAGE_FILTER_SIZE;
            float average_speed = calculate_moving_average(left_speed_buffer, AVERAGE_FILTER_SIZE);

            float total_distance = left_notch_count * DISTANCE_PER_NOTCH;
            PulseData_t data = { left_pulse_width, average_speed, total_distance, left_notch_count };
            
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
        if (right_pulse_width > MIN_PULSE_WIDTH) {
            float pulse_width_sec = right_pulse_width / MICROSECONDS_IN_A_SECOND;
            float speed = DISTANCE_PER_NOTCH / pulse_width_sec;

            // Update the buffer and calculate the moving average
            right_speed_buffer[right_buffer_index] = speed;
            right_buffer_index = (right_buffer_index + 1) % AVERAGE_FILTER_SIZE;
            float average_speed = calculate_moving_average(right_speed_buffer, AVERAGE_FILTER_SIZE);

            float total_distance = right_notch_count * DISTANCE_PER_NOTCH;
            PulseData_t data = { right_pulse_width, average_speed, total_distance, right_notch_count };
            
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

void encoder_init() {
    // Create pulse width and speed tasks for both left and right encoders
    printMessageBuffer = xMessageBufferCreate(256);
    leftMessageBuffer = xMessageBufferCreate(256);
    rightMessageBuffer = xMessageBufferCreate(256);
    if (leftMessageBuffer == NULL || rightMessageBuffer == NULL) {
        printf("Failed to create message buffers\n");
        while (true);  // Halt if buffer creation fails
    }
    xTaskCreate(left_pulse_width_task, "Left Pulse Width Task", 256, NULL, 1, &leftPulseTaskHandle);
    xTaskCreate(left_speed_task, "Left Speed Task", 256, NULL, 1, &leftSpeedTaskHandle);
    xTaskCreate(right_pulse_width_task, "Right Pulse Width Task", 256, NULL, 1, &rightPulseTaskHandle);
    xTaskCreate(right_speed_task, "Right Speed Task", 256, NULL, 1, &rightSpeedTaskHandle);

    // Create logging tasks for both left and right encoders
    xTaskCreate(left_log_task, "Left Encoder Log Task", 256, NULL, 1, NULL);
    xTaskCreate(right_log_task, "Right Encoder Log Task", 256, NULL, 1, NULL);
}


// Initialization for ultrasonic and encoder pins with unified interrupt callback
void setup_pins_with_unified_callback() {
    // Initialize ultrasonic sensor pins
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);

    // Initialize encoder pins
    gpio_init(LEFT_WHEEL_ENCODER_PIN);
    gpio_set_dir(LEFT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_WHEEL_ENCODER_PIN);

    gpio_init(RIGHT_WHEEL_ENCODER_PIN);
    gpio_set_dir(RIGHT_WHEEL_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_WHEEL_ENCODER_PIN);

    // Set up interrupts for ultrasonic and encoder pins using the unified callback
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &unified_gpio_callback);
    gpio_set_irq_enabled_with_callback(LEFT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &unified_gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_WHEEL_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &unified_gpio_callback);
}
