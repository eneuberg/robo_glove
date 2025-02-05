#include "GripperController.h"

template <typename T>
struct TaskParams {
    T** instances;                   
    size_t instanceCount;            
    msInterval interval;             
    void (*callback)(T*);           
    const char* taskName;
};

template <typename T>
void taskRunner(void* pvParameters)
{
    auto params = static_cast<TaskParams<T>*>(pvParameters);

    TickType_t period = pdMS_TO_TICKS(params->interval);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {   
        for (size_t i = 0; i < params->instanceCount; i++)
        {
            T* instance = params->instances[i];
            if (instance != nullptr) {
                params->callback(instance);
            }
        }


        if (params->taskName != nullptr) {
            //UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(nullptr);
            //Serial.print(">");
            //Serial.print(params->taskName);
            //Serial.print("Mark:");
            //Serial.println(stackHighWaterMark);
        }

        BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, period);
        
        if (xWasDelayed == pdFALSE && params->taskName != nullptr) {
            Serial.print(">");
            Serial.print(params->taskName);
            Serial.println("TaskMiss:1");
        }
        else {
            //Serial.print(">");
            //Serial.print(params->taskName);
            //Serial.println("TaskMiss:0");
        }
    }
}


GripperController::GripperController() 
    : index("index", 36, 32, 33, 1400, true),
    //thumb("thumb", 35, 4, 2, 1400, true),
    middle("middle", 39, 4, 16, 1400, true),
    ring("ring", 34, 18, 19, 1400, true),
    pinky("pinky", 13, 5, 17, 1400, true),
    ditherInterval(100000),
    pidInterval(10),
    updateInterval(50)
{}

void GripperController::begin()  {
    
    for (int i = 0; i < motorCount; i++) {
        motors[i]->begin();
        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("Range:");
        Serial.println(motors[i]->getFingerMin());
        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("Range:");
        Serial.println(motors[i]->getFingerMax());
    }

    static TaskParams<MotorDriver> pidMotorParams = {
        .instances = motors,
        .instanceCount = 3,
        .interval = pidInterval,
        .callback = [](MotorDriver* md) { md->pid(); },
        .taskName = "PidMotorTask"
    };

    static GripperController* controller[] = { this };

    static TaskParams<GripperController> updateParams = {
        .instances = controller,
        .instanceCount = 1,
        .interval = updateInterval,
        .callback = [](GripperController* gc) { gc->update(); },
        .taskName = "UpdateTask"
    };

    xTaskCreatePinnedToCore(
        taskRunner<MotorDriver>,
        pidMotorParams.taskName, 
        4048, 
        &pidMotorParams, 
        1, 
        nullptr, 
        1 // Pin to core 1
    );

    xTaskCreatePinnedToCore(
        taskRunner<GripperController>,
        updateParams.taskName, 
        4048, 
        &updateParams, 
        1, 
        nullptr, 
        0 // Pin to core 0
    );

}

void GripperController::update() {
    /*
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n') {
            currentGripperValue = serialBuffer.toFloat();
            serialBuffer = "";
        } else {
            serialBuffer += incomingChar; 
        }
    }

    index.mapToSetpoint(currentGripperValue);
    thumb.mapToSetpoint(currentGripperValue);
    
    */
    const unsigned long period = 5000;
    float basePhase = (millis() % period) / float(period); // normalized phase [0,1)

    Serial.print(">basePhase:");
    Serial.println(basePhase);
    
    float pidValues[motorCount];
    for (int i = 0; i < motorCount; i++) {
        // Calculate the phase offset for this motor.
        // If there is more than one motor, map i from 0 -> motorCount-1 to 0 -> 0.5 (half a cycle).
        float offset = (motorCount > 1) ? (i / float(motorCount - 1)) * 0.5 : 0.0;

        // Compute the motor's specific phase and corresponding sine value.
        float motorPhase = basePhase + offset;
        float motorSinValue = sin(2 * PI * motorPhase);

        // Store or log the PID value if needed.
        float pidValue = motors[i]->getCurrentPid();
        pidValues[i] = pidValue;

        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print(" CurrentEstimate: ");
        Serial.println(motors[i]->getEstimate());

        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print(" CurrentPid: ");
        Serial.println(pidValue);

        // Set the sinusoidal setpoint for this motor with its unique phase offset.
        motors[i]->setSinusoidalSetpoint(motorSinValue);
    }
}

void GripperController::calibrate() {
    int buttonPin = 14;
    int ledPin = 12;
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
    while (true) {
        for (int i = 0; i < motorCount; i++) {
            motors[i]->calibrate();
        }
        int button = digitalRead(buttonPin);
        Serial.print(">button1:");
        Serial.println(button);
        if (button == LOW) {
            break;
        }
        delay(100);
    }
    digitalWrite(ledPin, LOW);
}