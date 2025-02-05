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

// konvention: finger unten, poti value unten
GripperController::GripperController() 
    : index("index", 36, 2, 0, 1400, true),
    //thumb("thumb", 35, 4, 2, 1400, true),
    middle("middle", 39, 4, 16, 1400, true),
    ring("ring", 34, 18, 19, 1400, true),
    pinky("pinky", 35, 5, 17, 1400, true),
    ditherInterval(100000),
    pidInterval(10),
    updateInterval(50)
{}

void GripperController::begin()  {
    
    for (int i = 0; i < motorCount; i++) {
        motors[i]->begin();
    }

    static TaskParams<MotorDriver> pidMotorParams = {
        .instances = motors,
        .instanceCount = motorCount,
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
    float pidValues[motorCount];

    for (int i = 0; i < motorCount; i++) {
    
        motors[i]->updateSinusoidalSetpoint(i, motorCount);

        // Store or log the PID value if needed.
        float pidValue = motors[i]->getCurrentPid();
        pidValues[i] = pidValue;

        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("CurrentEstimate: ");
        Serial.println(motors[i]->getEstimate());

        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("CurrentPid: ");
        Serial.println(pidValue);

        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("CurrentSetpoint: ");
        Serial.println(motors[i]->getSetpoint());
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