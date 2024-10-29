#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pid.h"

// Define GPIO pins
#define L_MOTOR_PWM_PIN 0     // GP2 for PWM
#define L_MOTOR_DIR_PIN1 2    // GP0 for direction
#define L_MOTOR_DIR_PIN2 3    // GP1 for direction

#define R_MOTOR_PWM_PIN 1     // GP2 for PWM
#define R_MOTOR_DIR_PIN1 4    // GP0 for direction
#define R_MOTOR_DIR_PIN2 5    // GP1 for direction

// Define PID controller
extern PIDController pid;

// Function to set up the PWM
void setup_pwm(uint gpioLeft, uint gpioRight);

//Functions to control the car
void move_forward(uint gpioLeft, uint gpioRight);
void move_backward(uint gpioLeft, uint gpioRight);
void stop_car(uint gpioLeft, uint gpioRight);
void set_speed1(uint gpioLeft, uint gpioRight);
void set_speed2(uint gpioLeft, uint gpioRight);
void set_speed3(uint gpioLeft, uint gpioRight);

#endif