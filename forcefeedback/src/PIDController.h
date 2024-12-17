#pragma once

class PIDController {
public:
    PIDController(int threshhold, bool feedbackUp);
    float getOutput(int potiValue);

private:
    int threshhold;
    bool feedbackUp;

    int lastPotiValue = 0;
    long lastTime = 0;
    
    bool active = false;

    const int activationOffset = 20;
    const int deadzone = 15;
    const float derivativeDeadzone = 0.05f;

    float Kp = 0.5f; // Start with a small proportional gain
    float Kd = 0.1f; // Start with a small derivative gain

    float error(int potiValue);
    float potiDerivative(int potiValue);
    bool checkActivation(int potiValue, float err, float der);
};