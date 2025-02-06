#pragma once
#include <SimpleKalmanFilter.h>
#include <Arduino.h>
#include <math.h>

class FeedbackController {
public:
    FeedbackController(int setpoint, bool feedbackUp);
    float getOutput(float potiValue);
    void setSetpoint(int setpoint) { this->setpoint = setpoint; }
    int getSetpoint() { return setpoint; }
    void setFeedforward(int forward, int backward) {
        this->feedforwardPWMForward = forward;
        this->feedforwardPWMBackward = backward;
    }
    
private:
    float setpoint;
    bool feedbackUp;

    float lastPotiValue = 0;
    unsigned long lastTime = 0;

    SimpleKalmanFilter derivativeFilter;

    bool active = false;

    const int activationOffset = 40;
    const int deadzone = 30;
    const float derivativeDeadzone = 0.05f;

    float Kp = 1.4f; 
    float Kd = 1.0f; 

    float Ki = 0.01f;               
    float integralSum = 0.0f;     
    const float integralLimit = 25000.0f; // Anti-windup limit for the integral term

    //const int fwActivationThresholdFromSetpoint = 50;
    int feedforwardPWMForward = 0;
    int feedforwardPWMBackward = 0;

    float error(float potiValue);
    float potiDerivative(float potiValue);
    bool checkActivation(float potiValue, float err, float der);
};