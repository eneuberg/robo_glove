#pragma once
#include "FeedbackController.h"
#include <SimpleKalmanFilter.h>
#include "HelperFuncs.h"

typedef uint32_t msInterval;

class MotorDriver
{
public:
    MotorDriver(String fingerName, int potPin, int forwardPin, int backwardPin, bool feedbackUp);

    void begin();
    void dither();
    void pid();
    void mapToGripperSetpoint(float gripperValue);
    float getCurrentPid() { return currentPid; }
    String getName() { return name; }
    void calibrateRanges();
    
    void setSetpoint(int setpoint) { pidController.setSetpoint(setpoint); }
    void updateSinusoidalSetpoint(int motorIndex, int motorCount);
    
private:
    String name;

    int potPin;
    int forwardPin;
    int backwardPin;

    int currentPid = 0;
    int currentDither = 0;


    FeedbackController pidController;
    SimpleKalmanFilter filter;

    
    const float gripperMin = 37.0f;
    const float gripperMax = 0.0f;

    int fingerMin = 10000; 
    int fingerMax = 1;

    void driveMotor();
    float readAndFilter();

};
