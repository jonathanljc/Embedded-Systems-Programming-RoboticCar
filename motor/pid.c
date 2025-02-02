#include "pid.h"


void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

float pid_compute(PIDController *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    pid->integral += error * dt;

    // Clamp the integral to avoid windup
    if (pid->integral > INTEGRAL_WINDUP_LIMIT) {
        pid->integral = INTEGRAL_WINDUP_LIMIT;
    } else if (pid->integral < -INTEGRAL_WINDUP_LIMIT) {
        pid->integral = -INTEGRAL_WINDUP_LIMIT;
    }

    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > 1) {
        output = 1;
        pid->integral -= error;
    } else if (output < 0.0) {
        output = 0.0;
        pid->integral -= error;
    }
    return output;
}
    // return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

