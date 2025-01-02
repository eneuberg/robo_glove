#pragma once
#include <SimpleKalmanFilter.h>
#include <Arduino.h>
#include <math.h>

class PIDController {
public:
    PIDController(int threshhold, bool feedbackUp);
    float getOutput(float potiValue);
    void setKd(float Kd) { this->Kd = Kd; }
private:
    int threshhold;
    bool feedbackUp;

    float lastPotiValue = 0;
    unsigned long lastTime = 0;

    SimpleKalmanFilter derivativeFilter;

    bool active = false;

    const int activationOffset = 16;
    const int deadzone = 15;
    const float derivativeDeadzone = 0.05f;

    float Kp = 1.2f; // Start with a small proportional gain
    float Kd = 0.7f; // Start with a small derivative gain

    float error(float potiValue);
    float potiDerivative(float potiValue);
    bool checkActivation(float potiValue, float err, float der);
};