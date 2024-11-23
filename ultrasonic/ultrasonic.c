#include "ultrasonic.h"
#include "motor.h"
#include <string.h>
#include "encoder.h"

// Define external variables
volatile absolute_time_t start_time;
volatile uint64_t pulse_width;
volatile bool obstacleDetected;
volatile bool obstacleFlag = false;


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
    char command[100];
    char wifi_message[100];  // Buffer for Wi-Fi message
    absolute_time_t previous_time = get_absolute_time();
    absolute_time_t current_time;
    float dt = 0.0;
    float setpoint = 0.0;
    float left_average_speed = 0.0;
    float right_average_speed = 0.0;
    init_encoder_gpio();
    setupUltrasonicPins();
    init_motor_pins();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    init_pid();

    while (true) {
        // Trigger the ultrasonic sensor
        gpio_put(TRIGPIN, 1);
        sleep_us(10);
        gpio_put(TRIGPIN, 0);
        poll_encoder(&left_encoder, LEFT_WHEEL_ENCODER_PIN);  // Poll the left encoder
        poll_encoder(&right_encoder, RIGHT_WHEEL_ENCODER_PIN); // Poll the right encoder

        // Calculate the distance
        measured = pulse_width / 29.0 / 2.0;
        kalman_update(state, measured);

        message.distance = state->x;
        message.obstacleDetected = (state->x < 15);
        

        if (message.obstacleDetected && obstacleFlag == false) {
            stop_motors();
            //xMessageBufferSend(motorMessageBuffer, "0.0", 3, portMAX_DELAY);
            obstacleFlag = true;
            // Format the Wi-Fi message
            snprintf(wifi_message, sizeof(wifi_message), 
                    "Obstacle detected at %.2f cm. Motors stopping.\n", 
                    message.distance);
            xMessageBufferSend(wifiMessageBuffer, wifi_message, strlen(wifi_message) + 1, portMAX_DELAY);
        }else if(message.obstacleDetected && obstacleFlag == true){
            if (xMessageBufferReceive(wifiReceiveBuffer, &command, sizeof(command), 0) > 0){
                if(command[0] != 's' && command[5] != 'f'){
                    char instruction = command[5];
                    char speed = command[strlen(command) - 3];

                    switch(instruction){
                        case 'b':
                            move_backward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            break;
                        case 'l':
                            rotate_left(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            break;
                        case 'r':
                            rotate_right(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            break;
                        default:
                            break;
                    }
                    switch(speed){
                        case '5':
                            set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            //xMessageBufferSend(motorMessageBuffer, "0.5", 3, portMAX_DELAY);
                            break;
                        case '7':
                            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            //xMessageBufferSend(motorMessageBuffer, "0.7", 3, portMAX_DELAY);
                            break;
                        case '0':
                            set_speed100(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            //xMessageBufferSend(motorMessageBuffer, "0.4", 3, portMAX_DELAY);
                            break;
                        default:
                            break;
                    }
                }
            }
        } else if(disconnectRemote == false){
            obstacleFlag = false;
            // Check for commands in a non-blocking manner
            if (xMessageBufferReceive(wifiReceiveBuffer, &command, sizeof(command), 0) > 0) {
                printf("ULTRASONIC Received: %s\n", command);
                
                // Parse the instruction and speed from the command
                char stopCheck = command[0];
                char instruction = command[5];
                char speed = command[strlen(command) - 3];

                // Check if the command is "stop"
                if (stopCheck == 's') {
                    //xMessageBufferSend(motorMessageBuffer, "0.0", 3, portMAX_DELAY);
                    stop_motors();
                    printf("STOP\n");
                } else {
                    // Use switch-case to call the appropriate motor function
                    switch (instruction) {
                        case 'f':  // Move forward
                            move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            printf("FORWARD\n");
                            break;
                        case 'b':  // Move backward
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
                        case '5':  // Set speed to 50%
                            set_speed50(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

                            //xMessageBufferSend(motorMessageBuffer, "0.5", 3, portMAX_DELAY);

                            printf("SPEED 50\n");
                            break;
                        case '7':  // Set speed to 70%
                            set_speed70(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

                            //xMessageBufferSend(motorMessageBuffer, "0.7", 3, portMAX_DELAY);

                            printf("SPEED 70\n");
                            break;
                        case '0':  // Set speed to 100%
                            set_speed100(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
                            
                           //xMessageBufferSend(motorMessageBuffer, "1.0", 3, portMAX_DELAY);

                            printf("SPEED 100\n");
                            break;
                        default:
                            printf("Unknown speed: %c\n", speed);
                            break;
                    }
                }
            }
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
