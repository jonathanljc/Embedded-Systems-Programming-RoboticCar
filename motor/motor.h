#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include <stdint.h>

// Motor GPIO pin definitions
#define L_MOTOR_PWM_PIN 11     // PWM pin for the left motor
#define L_MOTOR_DIR_PIN1 12    // Direction pin 1 for left motor
#define L_MOTOR_DIR_PIN2 13    // Direction pin 2 for left motor

#define R_MOTOR_PWM_PIN 10     // PWM pin for the right motor
#define R_MOTOR_DIR_PIN1 14    // Direction pin 1 for right motor
#define R_MOTOR_DIR_PIN2 15    // Direction pin 2 for right motor

// PWM and speed settings
#define MAX_DUTY_CYCLE 12500
#define MAX_SPEED 0.48

// Function declarations
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
void init_motor_pins();
void move_forward(uint32_t gpioLeft, uint32_t gpioRight);
// void move_backward(uint32_t gpioLeft, uint32_t gpioRight, float speed);  // Add only if defined in motor.c
void rotate_left(uint32_t gpioLeft, uint32_t gpioRight);
void rotate_right(uint32_t gpioLeft, uint32_t gpioRight);
void stop_motors();
void set_motor_speed(uint32_t gpio, float speed, bool is_left);
void set_speed50(uint32_t gpioLeft, uint32_t gpioRight);
void set_speed70(uint32_t gpioLeft, uint32_t gpioRight);
void set_speed100(uint32_t gpioLeft, uint32_t gpioRight);
void set_left_motor_speed(uint32_t gpio, float speed);
void set_right_motor_speed(uint32_t gpio, float speed);
void motor_init_buffers(); // Initialize motor control message buffer
// void week10task1(void *pvParameters);

#endif
