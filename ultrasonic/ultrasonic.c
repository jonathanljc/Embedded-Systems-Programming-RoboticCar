#include "ultrasonic.h"

// Define external variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_width;
volatile bool obstacleDetected;
MessageBufferHandle_t printMessageBuffer;


// Initialize the Kalman filter state
kalman_state *kalman_init(double q, double r, double p, double initial_value)
{
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;

    return state;
}

// Interrupt handler for echo pin (rising and falling edges)
void get_echo_pulse(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        // Rising edge detected, start the timer
        start_time = get_absolute_time();
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        // Falling edge detected, calculate the pulse width
        pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
    }
}

// Update Kalman filter with new measurements
void kalman_update(kalman_state *state, double measurement)
{
    // Prediction update
    state->p = state->p + state->q;

    // Measurement update
    state->k = state->p / (state->p + state->r);  // Calculate Kalman gain
    state->x = state->x + state->k * (measurement - state->x);  // Update estimate
    state->p = (1 - state->k) * state->p;  // Update uncertainty
}

// Task to handle the ultrasonic sensor
void ultrasonic_task(void *pvParameters)
{
    kalman_state *state = (kalman_state *)pvParameters;
    double measured;
    DistanceMessage message;

    while (true)
    {
        // Send a pulse to trigger the ultrasonic sensor
        gpio_put(TRIGPIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));  // 10us pulse
        gpio_put(TRIGPIN, 0);

        // Wait for the pulse to complete
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms for echo response

        // Convert pulse width to distance in cm (Speed of sound: 343 m/s = 29 us/cm)
        measured = pulse_width / 29.0 / 2.0;

        // Update the Kalman filter with the measured distance
        kalman_update(state, measured);

        // Check if an obstacle is detected within 10 cm
        obstacleDetected = (state->x < 10);

        // Prepare the message with distance and obstacle detection status
        message.distance = state->x;
        message.obstacleDetected = obstacleDetected;

        // Send the message to the print task using a message buffer
        xMessageBufferSend(printMessageBuffer, &message, sizeof(message), 0);

        // Delay between readings to avoid excessive polling
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Task to handle printing
void print_task(void *pvParameters)
{
    DistanceMessage receivedMessage;

    while (true)
    {
        // Wait to receive data from the ultrasonic task
        if (xMessageBufferReceive(printMessageBuffer, &receivedMessage, sizeof(receivedMessage), portMAX_DELAY) > 0)
        {
            // Print the received distance
            printf("Distance: %.2lf cm\n", receivedMessage.distance);

            // If an obstacle is detected, print a warning
            if (receivedMessage.obstacleDetected)
            {
                printf("Obstacle detected within 10 cm! Taking action.\n");
            }
        }
    }
}

// Set up the ultrasonic sensor pins
void setupUltrasonicPins()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);

    // Enable rising and falling edge interrupts on the echo pin
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &get_echo_pulse);
}
