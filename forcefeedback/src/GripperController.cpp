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
            Serial.print(">");
            Serial.print(params->taskName);
            Serial.println("TaskMiss:0");
        }
    }
}


GripperController::GripperController() 
    : index("index", 34, 14, 12, 1400, true),
    thumb("thumb", 35, 4, 2, 1400, true),
    middle("middle", 32, 27, 25, 1400, true),
    ring("ring", 33, 26, 13, 1400, true),
    pinky("pinky", 25, 33, 15, 1400, true),
    ditherInterval(100000),
    pidInterval(10),
    updateInterval(50)
{}

void GripperController::begin()  {
    
    static MotorDriver* motors[] = { &index, &thumb, &middle, &ring, &pinky };

    static TaskParams<MotorDriver> pidMotorParams = {
        .instances = motors,
        .instanceCount = 3,
        .interval = pidInterval,
        .callback = [](MotorDriver* md) { md->pid(); },
        .taskName = "PidMotorTask"
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

}

void GripperController::update() {
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

    float indexPid = index.getCurrentPid();
    float thumbPid = thumb.getCurrentPid();

    bool indexBiggest = abs(indexPid) > abs(thumbPid);
    float currentPid = indexBiggest ? indexPid : thumbPid;

    Serial.print(">currentPid:");
    Serial.println(currentPid);
}

void GripperController::calibrate() {
    pinMode(26, INPUT_PULLUP);
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);
    while (true) {
        index.calibrate();
        thumb.calibrate();
        int button = digitalRead(26);
        Serial.print(">button1:");
        Serial.println(button);
        if (button == LOW) {
            break;
        }
        delay(100);
    }
    digitalWrite(27, LOW);
}