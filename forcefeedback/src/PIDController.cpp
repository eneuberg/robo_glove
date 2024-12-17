#include <math.h>
#include <Arduino.h>
#include "PIDController.h"


PIDController::PIDController(int threshhold, bool feedbackUp) {
    this->threshhold = threshhold;
    this->feedbackUp = feedbackUp;
}


float PIDController::getOutput(int potiValue) {
    // First, determine if PID is active
    float currentError = error(potiValue);
    float currentDerivative = potiDerivative(potiValue);
    bool currentlyActive = checkActivation(potiValue, currentError, currentDerivative);
    
    Serial.print(">currentError:");
    Serial.println(currentError);
    Serial.print(">currentDerivative:");
    Serial.println(currentDerivative);
    Serial.print(">currentlyActive:");
    Serial.println(currentlyActive);
    if (!currentlyActive) {
        // If PID is not active, return zero output
        return 0.0f;
    } else {
        float output = (this->Kp * currentError) + (this->Kd * currentDerivative);
        return output;
    }
}

bool PIDController::checkActivation(int potiValue, float err, float der) {
    if (this->active) {
        if ((abs(err) < this->deadzone) && (abs(der) < this->derivativeDeadzone)) {
            this->active = false;
        }
    }
    else if (err > this->activationOffset)   {
        this->active = true;
    }
    return this->active;
}

float PIDController::error(int potiValue) {
    return this->feedbackUp ? potiValue - this->threshhold : this->threshhold - potiValue;
}

float PIDController::potiDerivative(int potiValue) {
    long currentTime = millis();
    long deltaTime = currentTime - this->lastTime;
    int deltaPoti = potiValue - this->lastPotiValue;
    this->lastPotiValue = potiValue;
    this->lastTime = currentTime;

    Serial.print(">deltaTime:");
    Serial.println(deltaTime);
    Serial.print(">deltaPoti:");
    Serial.println(deltaPoti);
    if (deltaTime == 0) {
        return 0.0f;
    }
    return deltaPoti / static_cast<float>(deltaTime);
}