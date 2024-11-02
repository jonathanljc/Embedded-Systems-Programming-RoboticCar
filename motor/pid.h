#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


#define INTEGRAL_WINDUP_LIMIT 1000  // Example limit, adjust as needed
    typedef struct {
        float kp;
        float ki;
        float kd;
        float previous_error;
        float integral;
    } PIDController;

    void pid_init(PIDController *pid, float kp, float ki, float kd);
    float pid_compute(PIDController *pid, float setpoint, float measured_value);

    #endif // PID_CONTROLLER_H