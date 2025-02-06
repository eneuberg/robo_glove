#include "MotorDriver.h"

MotorDriver::MotorDriver(String fingerName, int potPin, int forwardPin, int backwardPin, bool feedbackUp)
    : name(fingerName),
    potPin(potPin), 
    forwardPin(forwardPin), 
    backwardPin(backwardPin), 
    pidController(0, feedbackUp),
    filter(12, 30, 0.1)
{
    pinMode(potPin, INPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);

    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);

    for (int i = 0; i < 100; i++) {
        readAndFilter();
        delay(1);
    }

    int currentEstimate = readAndFilter();
    pidController.setSetpoint(currentEstimate);
}

void MotorDriver::begin() {
    pidController.setSetpoint((fingerMin + fingerMax) / 2);
}

float MotorDriver::readAndFilter() {
    int potiValue = analogRead(potPin);
    float estimate = filter.updateEstimate(potiValue);
    currentEstimate = estimate;
    return estimate;
}

void MotorDriver::pid() {
    float estimate = readAndFilter();
    currentPid = pidController.getOutput(estimate);

    driveMotor();
}

void MotorDriver::dither() {
    unsigned long t = millis(); 
    int interval = 23;        
    bool state = (t / interval) % 2 == 0;
    double sawtoothValue = sawtooth(t, interval, 450);
    int pwmValue = state ? (int)sawtoothValue : -1 * (int)sawtoothValue;
    currentDither = pwmValue;

    driveMotor();
}

// konvention: finger unten, poti value unten
void MotorDriver::driveMotor() {

    currentPid = constrain(currentPid, -1023, 1023);
    int pwmSum = currentPid + currentDither;
    pwmSum = constrain(pwmSum, -1023, 1023);

    if (pwmSum == 0) {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, 0);
    } else if (pwmSum > 0) {
        analogWrite(forwardPin, pwmSum);
        analogWrite(backwardPin, 0);
    } else {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, abs(pwmSum));
    }
}

void MotorDriver::calibrateRanges()   {
    float estimate = readAndFilter();
    if (estimate < fingerMin) {
        fingerMin = estimate;
    }
    if (estimate > fingerMax) {
        fingerMax = estimate;
    }
}

// need to figure out how to differentiate between intentional pushing vs friction before implementing
void MotorDriver::calibrateFeedforward() {
    const int fwDetectionThreshold = 70;
    const float increment = 0.1f;

    float forwardPWM = 80;
    int initialEstimate = readAndFilter();
    int forwardEstimate = initialEstimate;
    while (forwardEstimate - initialEstimate > -fwDetectionThreshold) {
        forwardPWM += increment;
        analogWrite(forwardPin, floor(forwardPWM));
        forwardEstimate = readAndFilter();
        delay(1);
    }
    analogWrite(forwardPin, 0);
    delay(250);

    float backwardPWM = 80;
    initialEstimate = readAndFilter();
    int backwardEstimate = initialEstimate;
    while (backwardEstimate - initialEstimate < fwDetectionThreshold) {
        backwardPWM += increment;
        analogWrite(backwardPin, floor(backwardPWM));
        backwardEstimate = readAndFilter();
        delay(1);
    }
    analogWrite(backwardPin, 0);
    Serial.print(">");
    Serial.print(name);
    Serial.print("ForwardPWM:");
    Serial.println(forwardPWM);
    Serial.print(">");
    Serial.print(name);
    Serial.print("BackwardPWM:");
    Serial.println(backwardPWM);
    pidController.setFeedforward(floor(forwardPWM), floor(backwardPWM));
}

void MotorDriver::mapToGripperSetpoint(float gripperValue)   {
    float setpoint = mapFloat(gripperValue, gripperMin, gripperMax, fingerMin, fingerMax);
    pidController.setSetpoint(setpoint);
}

void MotorDriver::updateSinusoidalSetpoint(int motorIndex, int motorCount) {
    const float spatialSpan = 1.0;
    const unsigned long period = 5000;
    float basePhase = (millis() % period) / float(period) * 2.0 * PI;

    float offset = 0.0;
    if (motorIndex != 0)    {
        offset = (motorIndex / float(motorCount - 1)) * spatialSpan * PI;
    }

    float totalPhase = basePhase + offset;
    float sinVal = sin(totalPhase);

    float setpoint = mapFloat(sinVal, -1.0f, 1.0f, fingerMin, fingerMax);
    pidController.setSetpoint(setpoint);
}