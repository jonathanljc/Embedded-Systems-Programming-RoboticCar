#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"

#define TRIGPIN 1
#define ECHOPIN 0

#define MOVING_AVERAGE_SIZE 5  // Number of recent measurements to consider for moving average

volatile absolute_time_t start_time;
volatile uint64_t pulse_width = 0;
volatile bool obstacleDetected = false;

double moving_average_buffer[MOVING_AVERAGE_SIZE];  // Buffer to store recent distance measurements
int buffer_index = 0;  // Index to keep track of the position in the buffer
int buffer_filled = 0;  // Flag to indicate whether the buffer is fully populated

// Kalman filter structure
typedef struct kalman_state_
{
    double q; // process noise covariance
    double r; // measurement noise covariance
    double x; // estimated value
    double p; // estimation error covariance
    double k; // kalman gain
} kalman_state;

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

// Send a pulse and get the pulse width
uint64_t getPulse()
{
    // Trigger a 10us pulse
    gpio_put(TRIGPIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // Use FreeRTOS delay instead of sleep_us()
    gpio_put(TRIGPIN, 0);
    
    // Wait for 1ms to allow time for pulse measurement
    vTaskDelay(pdMS_TO_TICKS(1));  // Use FreeRTOS delay instead of sleep_ms()

    return pulse_width;
}

// Calculate the moving average of the buffer
double calculate_moving_average(double new_value)
{
    moving_average_buffer[buffer_index] = new_value;  // Store the new value in the buffer
    buffer_index = (buffer_index + 1) % MOVING_AVERAGE_SIZE;  // Update buffer index

    // If the buffer is fully populated, use all the values for averaging
    if (buffer_index == 0)
        buffer_filled = 1;

    double sum = 0;
    int count = buffer_filled ? MOVING_AVERAGE_SIZE : buffer_index;  // Use available values if buffer is not filled

    // Calculate the sum of the buffer
    for (int i = 0; i < count; i++)
    {
        sum += moving_average_buffer[i];
    }

    return sum / count;  // Return the average
}

// Get distance in centimeters and update with Kalman filter and moving average
double getCm(kalman_state *state)
{
    // Get pulse width
    uint64_t pulseLength = getPulse();
    
    // Convert pulse length to distance in cm (Speed of sound: 343 m/s = 29 us/cm)
    double measured = pulseLength / 29.0 / 2.0;

    // Apply the moving average filter
    double averaged_distance = calculate_moving_average(measured);

    // Update the Kalman filter state with the new averaged value
    kalman_update(state, averaged_distance);

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

    while (true)
    {
        // Get the filtered distance in centimeters
        double cm = getCm(state);

        // Print the distance
        printf("Distance: %.2lf cm\n", cm);

        // If an obstacle is detected, print a message
        if (obstacleDetected)
        {
            printf("Obstacle detected within 10 cm!\n");
        }

        // Delay between readings
        vTaskDelay(pdMS_TO_TICKS(500));  // Use FreeRTOS delay
    }
}

int main()
{
    // Initialize the sensor and Kalman filter
    stdio_init_all();
    printf("Setting up ultrasonic pins\n");
    setupUltrasonicPins();

    // Initialize the Kalman filter with example parameters
    kalman_state *state = kalman_init(1, 100, 1, 0);

    // Create the ultrasonic sensor task
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, (void *)state, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The program should never reach here because the scheduler takes over
    while (1);
    return 0;
}
