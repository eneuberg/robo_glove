#pragma once
#include "PIDController.h"
#include <SimpleKalmanFilter.h>

class MotorDriver {
public:
    MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp);

    void update();

private:
    int potPin;
    int forwardPin;
    int backwardPin;

    PIDController pid;
    SimpleKalmanFilter filter;

    // Helper function to drive motor based on PID output
    void dither();
    void driveMotor(float pidOutput);
};
