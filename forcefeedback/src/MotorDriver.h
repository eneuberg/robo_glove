#pragma once
#include "PIDController.h"
#include <SimpleKalmanFilter.h>
//#include "HelperFuncs.h"

typedef uint32_t msInterval;

class MotorDriver {
public:
    MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp);

    void begin();
    void dither();
    void pid();
    void handleSerial();

private:
    int potPin;
    int forwardPin;
    int backwardPin;

    msInterval ditherInterval;
    msInterval pidInterval;
    msInterval serialInterval;

    int currentPid = 0;
    int currentDither = 0;

    PIDController pidController;
    SimpleKalmanFilter filter;

    String serialBuffer;
    float gripperMin = 37.0f;
    float gripperMax = 0.0f;

    int fingerMin = 1600;
    int fingerMax = 2400;

    void driveMotor();
    void mapGripperToSetpoint(String serialLine);
    void calibrate();

    static void taskRunner(void* pvParameters);

    //for testing
    float testGripperValue = 25.0f;
};
