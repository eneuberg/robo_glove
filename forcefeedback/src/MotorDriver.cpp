#include "MotorDriver.h"

/*
SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */


MotorDriver::MotorDriver(String fingerName, int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp)
    : name(fingerName),
    potPin(potPin), 
    forwardPin(forwardPin), 
    backwardPin(backwardPin), 
    pidController(setpoint, feedbackUp),
    filter(12, 30, 0.1)
{
    pinMode(potPin, INPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);

    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);

    for (int i = 0; i < 100; i++) {
        readAndFilter();
    }
    Serial.print(">");
    Serial.print(name);
    Serial.print(">potPin:");
    Serial.println(potPin);
}

float MotorDriver::readAndFilter() {
    int potiValue = analogRead(potPin);
    float estimate = filter.updateEstimate(potiValue);
    return estimate;
}

void MotorDriver::pid() {
    float estimate = readAndFilter();
    Serial.print(">");
    //Serial.print(name);
    Serial.print("Estimate:");
    Serial.println(estimate);

    float pidOutput = pidController.getOutput(estimate);
    //Serial.print(">pidOutput:");
    //Serial.println(pidOutput);

    float factor = 2.0f;
    currentPid = (int)(pidOutput * factor); 

    driveMotor();
}

void MotorDriver::dither() {
    unsigned long t = micros(); 
    int interval = 100000;        

    bool state = (t / interval) % 2 == 0;

    double sawtoothValue = sawtooth(t, interval, 300);

    int pwmValue = state ? (int)sawtoothValue : -1 * (int)sawtoothValue;

    currentDither = pwmValue;
    //Serial.print(">ditherPWM:");
    //Serial.println(pwmValue);

    driveMotor();
}

void MotorDriver::driveMotor() {

    currentPid = constrain(currentPid, -1023, 1023);
    int pwmSum = currentPid;// + currentDither;
    pwmSum = constrain(pwmSum, -1023, 1023);

    //Serial.print(">");
    //Serial.print(name);
    //Serial.print("PWMSum:");
    //Serial.println(pwmSum);

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

void MotorDriver::calibrate()   {
    float estimate = readAndFilter();
    if (estimate < fingerMin) {
        fingerMin = estimate;
    }
    if (estimate > fingerMax) {
        fingerMax = estimate;
    }
    //Serial.print(">");
    //Serial.print(name);
    //Serial.print("fingerMin:");
    //Serial.println(fingerMin);
    //Serial.print(">");
    //Serial.print(name);
    //Serial.print("fingerMax:");
    //Serial.println(fingerMax);
    //digitalWrite(33, HIGH);
}

void MotorDriver::mapToSetpoint(float gripperValue)   {
    float setpoint = mapFloat(gripperValue, gripperMin, gripperMax, fingerMin, fingerMax);
    //Serial.print(">setpoint:");
    //Serial.println(setpoint);
    pidController.setSetpoint(setpoint);
}