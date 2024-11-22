#include "ultrasonic.h"
#include "motor.h"

// Define external variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_width;
volatile bool obstacleDetected;

// Define and initialize the message buffer
MessageBufferHandle_t printMessageBuffer;


// Initialize the Kalman filter state
kalman_state *kalman_init(double q, double r, double p, double initial_value) {
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;

    return state;
}

// Update Kalman filter with new measurements
void kalman_update(kalman_state *state, double measurement) {
    // Prediction update
    state->p = state->p + state->q;

    // Measurement update
    state->k = state->p / (state->p + state->r);  // Calculate Kalman gain
    state->x = state->x + state->k * (measurement - state->x);  // Update estimate
    state->p = (1 - state->k) * state->p;  // Update uncertainty
}

// GPIO interrupt callback for measuring pulse width
void ultrasonic_echo_callback(uint gpio, uint32_t events) {
    if (gpio == ECHOPIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            // Rising edge detected, start timing
            start_time = get_absolute_time();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            // Falling edge detected, calculate pulse width
            pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
        }
    }
}

void ultrasonic_task(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;
    double measured;
    DistanceMessage message;

    setupUltrasonicPins();
    init_motor_pins();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    while (true) {
        gpio_put(TRIGPIN, 1);
        sleep_us(10);
        gpio_put(TRIGPIN, 0);

        measured = pulse_width / 29.0 / 2.0;

        kalman_update(state, measured);

        message.distance = state->x;
        message.obstacleDetected = (state->x < 10);

        if (message.obstacleDetected) {
            // Obstacle detected: rotate right to avoid it
            stop_motors();

            // After rotation, resume moving forward
            //move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            //set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
            //vTaskDelay(pdMS_TO_TICKS(5500));  // Adjust delay as needed
            //stop_motors();
            //vTaskDelete(NULL);
   
        } else {
            // No obstacle detected, continue moving forward
            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
        }

        char command[512];
        // If wifiReceiveBuffer is not NULL, print the message in buffer
        if (xMessageBufferReceive(wifiReceiveBuffer, &command, sizeof(command), portMAX_DELAY) > 0) {
            printf("ULTRASONIC: %s\n", command);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Adjust delay as needed
    }
}


// Set up the ultrasonic sensor pins with interrupt for ECHOPIN
void setupUltrasonicPins() {
    gpio_init(TRIGPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);

    gpio_init(ECHOPIN);
    gpio_set_dir(ECHOPIN, GPIO_IN);

    // Enable interrupt on ECHOPIN for both rising and falling edges
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &ultrasonic_echo_callback);
}
