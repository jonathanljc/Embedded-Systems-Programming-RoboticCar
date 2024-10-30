#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pid.h"
#include <stdint.h>  // Add this line to motor.h

// Define GPIO pins - now only in motor.h
#define L_MOTOR_PWM_PIN 10     // PWM pin for the left motor
#define L_MOTOR_DIR_PIN1 12    // Direction pin 1 for left motor
#define L_MOTOR_DIR_PIN2 13    // Direction pin 2 for left motor

#define R_MOTOR_PWM_PIN 11     // PWM pin for the right motor
#define R_MOTOR_DIR_PIN1 14    // Direction pin 1 for right motor
#define R_MOTOR_DIR_PIN2 15    // Direction pin 2 for right motor

// Define PID controller
extern PIDController pid;

// Function to set up the PWM
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);

// Functions to control the car
void init_motor_pins(); // initialize the motor pin setttings
void move_forward(uint32_t gpioLeft, uint32_t gpioRight);
void move_backward(uint32_t gpioLeft, uint32_t gpioRight);
void stop_car(uint32_t gpioLeft, uint32_t gpioRight);
void set_speed30(uint32_t gpioLeft, uint32_t gpioRight);
void set_speed50(uint32_t gpioLeft, uint32_t gpioRight);
void set_speed100(uint32_t gpioLeft, uint32_t gpioRight);
void rotate_left(uint32_t gpioLeft, uint32_t gpioRight);
void rotate_right(uint32_t gpioLeft, uint32_t gpioRight);

#endif
