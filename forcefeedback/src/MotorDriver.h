#pragma once
#include "PIDController.h"
#include <SimpleKalmanFilter.h>
#include "HelperFuncs.h"

typedef uint32_t msInterval;

class MotorDriver
{
public:
    MotorDriver(String fingerName, int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp);

    void dither();
    void pid();
    void mapToSetpoint(float gripperValue);
    float getCurrentPid() { return currentPid; }
    void calibrate();

private:
    String name;

    int potPin;
    int forwardPin;
    int backwardPin;

    int currentPid = 0;
    int currentDither = 0;

    PIDController pidController;
    SimpleKalmanFilter filter;

    
    float gripperMin = 37.0f;
    float gripperMax = 0.0f;

    int fingerMin = 1000; 
    int fingerMax = 500;

    void driveMotor();
    float readAndFilter();

};
