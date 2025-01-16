#pragma once
#include "PIDController.h"
#include <SimpleKalmanFilter.h>

class MotorDriver {
public:
    MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp);

    void begin();
    void dither();
    void drivePID();

private:
    int potPin;
    int forwardPin;
    int backwardPin;

    int ditherInterval;
    int pidInterval;

    int currentPWM = 0;

    PIDController pid;
    SimpleKalmanFilter filter;

    // Helper function to drive motor based on PID output
    static void ditherTask(void *pvParameters);
    static void pidTask(void *pvParameters);

    void driveMotor(int pwmValue);
};
