#pragma once
#include <SimpleKalmanFilter.h>
#include <Arduino.h>
#include <math.h>

class PIDController {
public:
    PIDController(int setpoint, bool feedbackUp);
    float getOutput(float potiValue);
    void changeSetpoint(float delta) { this->setpoint += delta; }

private:
    float setpoint;
    bool feedbackUp;

    float lastPotiValue = 0;
    unsigned long lastTime = 0;

    SimpleKalmanFilter derivativeFilter;

    bool active = false;

    const int activationOffset = 50;
    const int deadzone = 40;
    const float derivativeDeadzone = 0.05f;

    float Kp = 1.4f; // Start with a small proportional gain
    float Kd = 1.0f; // Start with a small derivative gain

    float error(float potiValue);
    float potiDerivative(float potiValue);
    bool checkActivation(float potiValue, float err, float der);
};