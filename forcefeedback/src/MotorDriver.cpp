#include "MotorDriver.h"

/*
SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

struct TaskParams {
    MotorDriver* instance;        // Pointer to the MotorDriver instance
    msInterval interval;            // Interval in milliseconds
    void (*callback)(MotorDriver*); // Callback function for the task
    const char* taskName;         // Task name (optional, for debugging)
};

int intClamp(int x, int min, int max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

double sawtooth(double t, double period, double amplitude) {
    if (period <= 0) {
        return 0.0; // Avoid division by zero
    }

    // Wrap time t within one period using fmod
    double fractionalPart = fmod(t, period);
    if (fractionalPart < 0) {
        fractionalPart += period; // Handle negative times
    }

    // Map to range [-amplitude, +amplitude]
    return amplitude * (2.0 * (fractionalPart / period) - 1.0);
}


MotorDriver::MotorDriver(int potPin, int forwardPin, int backwardPin, int setpoint, bool feedbackUp)
    : potPin(potPin), 
    forwardPin(forwardPin), 
    backwardPin(backwardPin), 
    pidController(setpoint, feedbackUp),
    filter(12, 30, 0.1),
    ditherInterval(10000),
    pidInterval(10),
    serialInterval(200)
{
    pinMode(potPin, INPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);

    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
}

void MotorDriver::begin() {
    // Static allocation ensures the lifetime of the params matches the tasks
    static TaskParams ditherParams = {
        .instance = this,
        .interval = ditherInterval,
        .callback = [](MotorDriver* driver) { driver->dither(); },
        .taskName = "DitherTask"
    };

    static TaskParams pidParams = {
        .instance = this,
        .interval = pidInterval,
        .callback = [](MotorDriver* driver) { driver->pid(); },
        .taskName = "PIDTask"
    };

    static TaskParams serialParams = {
        .instance = this,
        .interval = serialInterval,
        .callback = [](MotorDriver* driver) { driver->handleSerial(); },
        .taskName = "SerialTask"
    };

    xTaskCreate(taskRunner, "DitherTask", 2048, &ditherParams, 1, nullptr);
    xTaskCreate(taskRunner, "PIDTask", 2048, &pidParams, 3, nullptr);
    xTaskCreate(taskRunner, "SerialTask", 2048, &serialParams, 2, nullptr);
    
}


void MotorDriver::pid() {
    int potiValue = analogRead(potPin);
    //Serial.print(">potiValue:");
    //Serial.println(potiValue);

    float estimate = filter.updateEstimate(potiValue);
    Serial.print(">estimate:");
    Serial.println(estimate);

    float pidOutput = pidController.getOutput(estimate);
    //Serial.print(">pidOutput:");
    //Serial.println(pidOutput);

    float factor = 2.0f;
    currentPid = (int)(pidOutput * factor); 

    //erial.print(">pwmValue:");
    //Serial.println(pwmValue);

    driveMotor();
}

void MotorDriver::dither() {
    unsigned long t = micros(); 
    int interval = 4758;        

    bool state = (t / interval) % 2 == 0;

    double sawtoothValue = sawtooth(t, interval, 300);

    int pwmValue = state ? (int)sawtoothValue : -1 * (int)sawtoothValue;

    currentDither = pwmValue;
    //Serial.print(">ditherPWM:");
    //Serial.println(pwmValue);

    driveMotor();
}

void MotorDriver::driveMotor() {

    currentPid = intClamp(currentPid, -1023, 1023);
    int pwmSum = currentDither + currentPid;
    pwmSum = intClamp(pwmSum, -1023, 1023);

    Serial.print(">pwmSum:");
    Serial.println(pwmSum);

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
    
}

void MotorDriver::handleSerial() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n') {
            mapGripperToSetpoint(serialBuffer);
            serialBuffer = "";
        } else {
            serialBuffer += incomingChar; 
        }
    }
    Serial.print(">currentPid:");
    Serial.println(currentPid);

    //test
    float updateVec = mapFloat(currentPid, -1000, 1000, 3, -3);
    testGripperValue += updateVec;
    testGripperValue = intClamp(testGripperValue, 0, 37);
    mapGripperToSetpoint(String(testGripperValue));
    Serial.print(">updateVec:");
    Serial.println(updateVec);
}

void MotorDriver::mapGripperToSetpoint(String serialLine)   {
    float gripperValue = serialLine.toFloat();
    Serial.print(">gripperValue:");
    Serial.println(gripperValue);
    float setpoint = mapFloat(gripperValue, gripperMin, gripperMax, fingerMin, fingerMax);
    Serial.print(">setpoint:");
    Serial.println(setpoint);
    pidController.setSetpoint(setpoint);
}

void MotorDriver::taskRunner(void *pvParameters) {
    // Cast the parameters to the TaskParams structure
    TaskParams* params = static_cast<TaskParams*>(pvParameters);

    MotorDriver* driver = params->instance;
    TickType_t period = pdMS_TO_TICKS(params->interval);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Call the callback function with the MotorDriver instance
        params->callback(driver);

        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, period);

        if (xWasDelayed == pdFALSE && params->taskName != nullptr) {
            //Serial.print(">");
            //Serial.print(params->taskName);
            //Serial.println("TaskMiss:1");
        }
    }
}