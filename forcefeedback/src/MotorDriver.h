#pragma once
#include "PIDController.h"
#include <SimpleKalmanFilter.h>

class MotorDriver {
public:
    MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp);

    void update();
    void setSetpoint(int setpoint);

private:
    int potPin;
    int forwardPin;
    int backwardPin;

    PIDController pid;
    SimpleKalmanFilter filter;
    
    float aggressiveness = 1.0f; // Default: no extra scaling

    // Helper function to drive motor based on PID output
    void driveMotor(float pidOutput);
};
