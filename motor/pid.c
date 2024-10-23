#include "pid.h"

void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

float pid_compute(PIDController *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    pid->integral += error;
    float derivative = error - pid->previous_error;
    pid->previous_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}