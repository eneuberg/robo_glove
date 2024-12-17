#include <Arduino.h>
#include "MotorDriver.h"
#include "PIDController.h"

MotorDriver::MotorDriver(int potPin, int forwardPin, int backwardPin, int thresholdValue, bool feedbackUp)
    : potPin(potPin), forwardPin(forwardPin), backwardPin(backwardPin), pid(thresholdValue, feedbackUp)
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

    Serial.print(">potiValue:");
    Serial.println(potiValue);

    // Get PID output
    float pidOutput = pid.getOutput(potiValue);

    pidOutput *= aggressiveness;

    Serial.print(">pidOutputWScaling:");
    Serial.println(pidOutput);
    
    // Drive motor based on PID output
    driveMotor(pidOutput);
}

void MotorDriver::driveMotor(float pidOutput) {
    // Convert PID output to direction and speed
    int pwmValue = (int)abs(pidOutput);
    if (pwmValue > 255) pwmValue = 255; // Clamp max to 255

    Serial.print(">pwmValue:");
    Serial.println(pwmValue);

    if (pidOutput > 0) {
        // Forward direction: set forwardPin to PWM, backwardPin low
        analogWrite(forwardPin, pwmValue);
        analogWrite(backwardPin, 0);
    } else if (pidOutput < 0) {
        // Backward direction: set backwardPin to PWM, forwardPin low
        analogWrite(backwardPin, pwmValue);
        analogWrite(forwardPin, 0);
    } else {
        // Zero output: stop
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, 0);
    }
}

void MotorDriver::setAggressiveness(float factor) {
    // Ensure factor is reasonable (e.g. 0.1 to 5.0)
    if (factor < 0.0f) factor = 0.0f;
    aggressiveness = factor;
}
