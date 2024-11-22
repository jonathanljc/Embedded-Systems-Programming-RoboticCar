#include "ultrasonic.h"
#include "motor.h"
#include <string.h>

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
    char command[512];

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
        } else if (xMessageBufferReceive(wifiReceiveBuffer, &command, sizeof(command), portMAX_DELAY) > 0) {
            printf("ULTRASONIC Received: %s\n", command);
            // printf("First 4 char: %c%c%c%c\n", command[0], command[1], command[2], command[3]);
            // printf("Instruction: %c\n", command[5]);
            // printf("Speed: %c\n", command[strlen(command) - 3]);

            // Parse the instruction and speed from the command
            char stopCheck = command[0];
            char instruction = command[5];
            char speed = command[strlen(command) - 3];

            // Check if the command is "stop"
            if(stopCheck == 's') {
                stop_motors();
                printf("STOP\n");
            }else{
                // Use switch-case to call the appropriate motor function
                switch (instruction) {
                    case 'f':  // Move forward
                        move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("FORWARD\n");
                        break;
                    case 'b':  // Stop
                        move_backward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("BACKWARD\n");
                        break;
                    case 'l':  // Rotate left
                        rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("LEFT\n");
                        break;
                    case 'r':  // Rotate right
                        rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("RIGHT\n");
                        break;
                    default:
                        printf("Unknown instruction: %c\n", instruction);
                        break;
                }

                // Use switch-case to set the speed
                switch (speed) {
                    case '4':  // Set speed to 50%
                        set_speed40(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 40\n");
                        break;
                    case '7':  // Set speed to 70%
                        set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 70\n");
                        break;
                    case '0':  // Set speed to 100%
                        set_speed100(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 100\n");
                        break;
                    default:
                        printf("Unknown speed: %c\n", speed);
                        break;
                }
            }
        }
        
        else {
            // No obstacle detected, continue moving forward
           stop_motors();
        }


        // If wifiReceiveBuffer is not NULL, print the message in buffer
        /*if (xMessageBufferReceive(wifiReceiveBuffer, &command, sizeof(command), portMAX_DELAY) > 0) {
            printf("ULTRASONIC Received: %s\n", command);
            // printf("First 4 char: %c%c%c%c\n", command[0], command[1], command[2], command[3]);
            // printf("Instruction: %c\n", command[5]);
            // printf("Speed: %c\n", command[strlen(command) - 3]);

            // Parse the instruction and speed from the command
            char stopCheck = command[0];
            char instruction = command[5];
            char speed = command[strlen(command) - 3];

            // Check if the command is "stop"
            if(stopCheck == 's') {
                stop_motors();
                printf("STOP\n");
            }else{
                // Use switch-case to call the appropriate motor function
                switch (instruction) {
                    case 'f':  // Move forward
                        move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("FORWARD\n");
                        break;
                    case 'b':  // Stop
                        move_backward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("BACKWARD\n");
                        break;
                    case 'l':  // Rotate left
                        rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("LEFT\n");
                        break;
                    case 'r':  // Rotate right
                        rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("RIGHT\n");
                        break;
                    default:
                        printf("Unknown instruction: %c\n", instruction);
                        break;
                }

                // Use switch-case to set the speed
                switch (speed) {
                    case '4':  // Set speed to 50%
                        set_speed40(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 40\n");
                        break;
                    case '7':  // Set speed to 70%
                        set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 70\n");
                        break;
                    case '0':  // Set speed to 100%
                        set_speed100(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                        printf("SPEED 100\n");
                        break;
                    default:
                        printf("Unknown speed: %c\n", speed);
                        break;
                }
            } 
        } */

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
