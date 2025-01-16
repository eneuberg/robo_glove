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
    filter(12, 30, 0.1),
    ditherInterval(30),
    pidInterval(10)
{
    pinMode(potPin, INPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);

    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

void MotorDriver::begin() {
    // Create DITHER task
    xTaskCreate(
        ditherTask,            // Task function
        "DitherTask",          // Name (for debugging)
        2048,                  // Stack size in words
        this,                  // Pointer to current MotorDriver instance
        2,                     // Priority
        nullptr                // Task handle (optional)
    );

    // Create PID task
    xTaskCreate(
        pidTask,
        "PIDTask",
        2048,
        this,
        1,
        nullptr
    );
    
}

void MotorDriver::drivePID() {
    int potiValue = analogRead(potPin);
    //Serial.print(">potiValue:");
    //Serial.println(potiValue);

    float estimate = filter.updateEstimate(potiValue);
    Serial.print(">estimate:");
    Serial.println(estimate);

    float pidOutput = pid.getOutput(estimate);
    Serial.print(">pidOutput:");
    Serial.println(pidOutput);

    // Convert PID output to direction and speed
    float factor = 2.0f;
    int pwmValue = (int)abs(pidOutput * factor); 
    if (pwmValue > 1023) pwmValue = 1023; // Clamp max to 255
    //Serial.print(">pwmValue:");
    //Serial.println(pwmValue);

    if (pidOutput == 0) {
        driveMotor(0);
    } else {
        float setpointDelta = pidOutput/300.0f;
        pid.changeSetpoint(setpointDelta);
        driveMotor(pidOutput > 0 ? pwmValue : -pwmValue);
    }
}

void MotorDriver::dither() {
    //something
}

void MotorDriver::driveMotor(int pwmValue) {
    currentPWM = pwmValue;
    //Serial.print(">currentPWM:");
    //Serial.println(currentPWM);
    if (pwmValue == 0) {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, 0);
    } else if (pwmValue > 0) {
        analogWrite(forwardPin, currentPWM);
        analogWrite(backwardPin, 0);
    } else {
        analogWrite(forwardPin, 0);
        analogWrite(backwardPin, abs(currentPWM));
    }
}

void MotorDriver::ditherTask(void *pvParameters) {
    MotorDriver* driver = static_cast<MotorDriver*>(pvParameters);
    
    TickType_t ditherPeriod = pdMS_TO_TICKS(driver->ditherInterval);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        driver->dither();

        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, ditherPeriod);

        if (xWasDelayed == pdFALSE) {
            //Serial.print(">ditherTaskMiss:");
            //Serial.println("1");
        } else {
            //Serial.print(">ditherTaskMiss:");
            //Serial.println("0");
        }
    }
}

void MotorDriver::pidTask(void *pvParameters) {
    MotorDriver* driver = static_cast<MotorDriver*>(pvParameters);
    
    TickType_t pidPeriod = pdMS_TO_TICKS(driver->pidInterval);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        driver->drivePID();

        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pidPeriod);

        if (xWasDelayed == pdFALSE) {
            //Serial.print(">pidTaskMiss:");
            //Serial.println("1");
        } else {
            //Serial.print(">pidTaskMiss:");
            //Serial.println("0");
        }
    }
}
