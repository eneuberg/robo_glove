#include "MotorDriver.h"

/*
SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

MotorDriver::MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp)
    : potPin(potPin), 
    forwardPin(forwardPin), 
    backwardPin(backwardPin), 
    pid(setpoint, feedbackUp),
    filter(12, 30, 0.03)
{
    // Configure pins
    pinMode(potPin, INPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);

    // Optionally initialize these pins to LOW
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

void MotorDriver::update() {
    // Read the potentiometer value
    int potiValue = analogRead(potPin);

    //Serial.print(">potiValue:");
    //Serial.println(potiValue);

    float estimate = filter.updateEstimate(potiValue);
    //Serial.print(">estimate:");
    //Serial.println(estimate);

    // Get PID output
    float pidOutput = pid.getOutput(estimate);


    Serial.print(">pidOutput:");
    Serial.println(pidOutput);
    
    // Drive motor based on PID output
    driveMotor(pidOutput);
}

void MotorDriver::driveMotor(float pidOutput) {
    // Convert PID output to direction and speed
    int pwmValue = (int)abs(pidOutput); 
    if (pwmValue > 1023) pwmValue = 1023; // Clamp max to 255

    //Serial.print(">pwmValue:");
    //Serial.println(pwmValue);

    if (pidOutput == 0) {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, 0);
    } else {
        float setpointDelta = pidOutput/150.0f;
        pid.changeSetpoint(setpointDelta);
        // Backward direction: set backwardPin to PWM, forwardPin low
        if (pidOutput < 0) {
            analogWrite(forwardPin, 0);
            analogWrite(backwardPin, pwmValue);
        } else {
            analogWrite(forwardPin, pwmValue);
            analogWrite(backwardPin, 0);
        }
    }
}
