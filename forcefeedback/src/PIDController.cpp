#include "PIDController.h"


PIDController::PIDController(int setpoint, bool feedbackUp) 
    : setpoint(setpoint), 
    feedbackUp(feedbackUp),
    derivativeFilter(1, 3, 0.03)
{}


float PIDController::getOutput(float potiValue) {
    // First, determine if PID is active
    float currentError = error(potiValue);
    float currentDerivative = potiDerivative(potiValue);
    bool currentlyActive = checkActivation(potiValue, currentError, currentDerivative);
    
    //Serial.print(">currentError:");
    //Serial.println(currentError);
    //Serial.print(">currentlyActive:");
    //Serial.println(currentlyActive);

    if (!currentlyActive) {
        // If PID is not active, return zero output
        return 0.0f;
    } else {
        float proportional = this->Kp * currentError;
        float derivative = this->Kd * currentDerivative;
        //Serial.print(">proportionalTerm:");
        //Serial.println(proportional);
        //Serial.print(">derivativeTerm:");
        //Serial.println(derivative);
        float output = proportional + derivative;
        return output;
    }
}

bool PIDController::checkActivation(float potiValue, float err, float der) {
    if (this->active) {
        if ((abs(err) < this->deadzone)) {//&& (abs(der) < this->derivativeDeadzone)) {
            this->active = false;
        }
    }
    else if (err > this->activationOffset)   {
        this->active = true;
    }
    return this->active;
}

float PIDController::error(float potiValue) {
    return this->feedbackUp ? potiValue - this->setpoint : this->setpoint - potiValue;
}

float PIDController::potiDerivative(float potiValue) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - this->lastTime;
    float deltaPoti = potiValue - this->lastPotiValue;
    this->lastPotiValue = potiValue;
    this->lastTime = currentTime;

    //Serial.print(">deltaPoti:");
    //Serial.println(deltaPoti);
    Serial.print(">deltaTimeMillis:");
    Serial.println(deltaTime); 

    float derivative = static_cast<float>(deltaPoti) / static_cast<float>(deltaTime);
    //Serial.print(">currentDerivative:");
    //Serial.println(derivative);

    float scaledDerivative = derivative * 100; // 100 ms
    //Serial.print(">scaledDerivative100Ms:");
    //Serial.println(scaledDerivative);

    float estimatedDerivative = this->derivativeFilter.updateEstimate(scaledDerivative);
    //Serial.print(">estimatedDerivative100Ms:");
    //Serial.println(estimatedDerivative);

    return scaledDerivative;
}