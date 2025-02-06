#include "FeedbackController.h"


FeedbackController::FeedbackController(int setpoint, bool feedbackUp) 
    : setpoint(setpoint), 
    feedbackUp(feedbackUp),
    derivativeFilter(3, 1, 0.2)
{}


float FeedbackController::getOutput(float potiValue) {

    float currentError = error(potiValue);
    float currentDerivative = potiDerivative(potiValue);
    bool currentlyActive = checkActivation(potiValue, currentError, currentDerivative);

    if (!currentlyActive) {
        integralSum = 0.0f;
        return 0.0f;
    } else {
        integralSum += currentError;
        integralSum = constrain(integralSum, -integralLimit, integralLimit);
        float proportional = this->Kp * currentError;
        float derivative = this->Kd * currentDerivative;
        float integral = this->Ki * integralSum;
        float pid = proportional;

        int feedforward = 0;
        //if (currentError > fwActivationThresholdFromSetpoint) {
        if (currentError < 0) {
            feedforward = feedforwardPWMForward;
        //} else if (currentError < -fwActivationThresholdFromSetpoint) {
        } else {
            feedforward = feedforwardPWMBackward;
        }

        float output = feedforward;
        return output;
    }
}

bool FeedbackController::checkActivation(float potiValue, float err, float der) {
    if (this->active) {
        if ((abs(err) < this->deadzone)) {//&& (abs(der) < this->derivativeDeadzone)) {
            this->active = false;
        }
    }
    else if (abs(err) > this->activationOffset)   {
        this->active = true;
    }
    return this->active;
}

float FeedbackController::error(float potiValue) {
    return this->feedbackUp ? potiValue - this->setpoint : this->setpoint - potiValue;
}

float FeedbackController::potiDerivative(float potiValue) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - this->lastTime;
    float deltaPoti = potiValue - this->lastPotiValue;
    this->lastPotiValue = potiValue;
    this->lastTime = currentTime;
    
    float derivative = static_cast<float>(deltaPoti) / static_cast<float>(deltaTime);
    float scaledDerivative = derivative * 100;

    return scaledDerivative;
}

