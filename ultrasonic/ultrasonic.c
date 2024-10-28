#include "ultrasonic.h"

// Define external variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_width;
volatile bool obstacleDetected;

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

// Send a pulse and get the pulse width
uint64_t getPulse()
{
    // Trigger a 10us pulse (non-blocking delay)
    gpio_put(TRIGPIN, 1);
    busy_wait_us(10);  // Use busy-wait for a short 10us delay
    gpio_put(TRIGPIN, 0);
    
    // With a busy-wait for a short delay if you want precise timing:
    busy_wait_us(1000);  // Wait for 1 ms using a non-blocking busy-wait

    return pulse_width;
}

// Get distance in centimeters and update with Kalman filter
double getCm(kalman_state *state)
{
    // Get pulse width
    uint64_t pulseLength = getPulse();
    
    // Convert pulse length to distance in cm (Speed of sound: 343 m/s = 29 us/cm)
    double measured = pulseLength / 29.0 / 2.0;

    // Update the Kalman filter state with the new measurement
    kalman_update(state, measured);

    // Check if an obstacle is detected within 10 cm
    if (state->x < 10)
    {
        obstacleDetected = true;
    }
    else
    {
        obstacleDetected = false;
    }

    // Return the filtered distance
    return state->x;
}

// Task to handle the ultrasonic sensor
void ultrasonic_task(void *pvParameters)
{
    kalman_state *state = (kalman_state *)pvParameters;
    double cm;
    DistanceMessage message;

    while (true)
    {
        // Get the filtered distance in centimeters
        cm = getCm(state);

        // Prepare the message with distance and obstacle detection status
        message.distance = cm;
        message.obstacleDetected = obstacleDetected;

        // Send the message to the print task using a message buffer
        xMessageBufferSend(printMessageBuffer, &message, sizeof(message), 0);

        // Delay between readings
        vTaskDelay(pdMS_TO_TICKS(50));  // Use FreeRTOS delay
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
