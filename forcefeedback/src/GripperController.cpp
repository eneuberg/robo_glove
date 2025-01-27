#include "GripperController.h"


struct MotorTaskParams {
    MotorDriver** instances;        // Array of MotorDriver pointers
    size_t instanceCount;           // Number of MotorDriver objects
    msInterval interval;            // Interval in milliseconds
    void (*callback)(MotorDriver*); // Specific callback for MotorDriver
    const char* taskName;           // Optional: Task name for debugging
};

struct ControllerTaskParams {
    GripperController* instance;    // Pointer to the GripperController instance
    msInterval interval;            // Interval in milliseconds
    void (*callback)(GripperController*); // Specific callback for GripperController
    const char* taskName;           // Optional: Task name for debugging
};


GripperController::GripperController() 
    : index("index", 34, 14, 12, 1400, true),
    thumb("thumb", 35, 4, 2, 1400, true),
    ditherInterval(100000),
    pidInterval(40),
    updateInterval(50)
{}

void GripperController::begin()  {
    MotorDriver* driverList[] = {&index, &thumb};
    
    static MotorTaskParams ditherParams = {
        .instances = driverList,
        .instanceCount = sizeof(driverList) / sizeof(driverList[0]),
        .interval = ditherInterval,
        .callback = [](MotorDriver* driver) { driver->dither(); },
        .taskName = "MotorTask"
    };

    static MotorTaskParams pidParams = {
        .instances = driverList,
        .instanceCount = sizeof(driverList) / sizeof(driverList[0]),
        .interval = pidInterval,
        .callback = [](MotorDriver* driver) { driver->pid(); },
        .taskName = "PIDTask"
    };

    static ControllerTaskParams controllerParams = {
        .instance = this,
        .interval = updateInterval,
        .callback = [](GripperController* ctrl) { ctrl->update(); },
        .taskName = "ControllerTask"
    };

    xTaskCreate(motorTaskRunner, ditherParams.taskName, 2048, &ditherParams, 1, nullptr);
    //xTaskCreate(motorTaskRunner, pidParams.taskName, 8192, &pidParams, 3, nullptr);
    //xTaskCreate(controllerTaskRunner, controllerParams.taskName, 2048, &controllerParams, 2, nullptr);
    

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

void GripperController::motorTaskRunner(void* pvParameters) {
    // Cast the parameters to the MotorTaskParams structure
    MotorTaskParams* params = static_cast<MotorTaskParams*>(pvParameters);

    MotorDriver** drivers = params->instances;  // Array of MotorDriver objects
    size_t driverCount = params->instanceCount; // Number of MotorDriver objects
    TickType_t period = pdMS_TO_TICKS(params->interval); // Interval as ticks
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        for (size_t i = 0; i < driverCount; ++i) {
            if (drivers[i] != nullptr) {
                params->callback(drivers[i]); // Call the callback for each driver
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
            // Optional: Debugging for missed deadlines
            // Serial.print(">");
            // Serial.print(params->taskName);
            // Serial.println("TaskMiss:1");
        }
    }
}


void GripperController::controllerTaskRunner(void* pvParameters) {
    // Cast the parameters to the ControllerTaskParams structure
    ControllerTaskParams* params = static_cast<ControllerTaskParams*>(pvParameters);

    GripperController* controller = params->instance; // Single GripperController object
    TickType_t period = pdMS_TO_TICKS(params->interval); // Interval as ticks
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (controller != nullptr) {
            params->callback(controller); // Call the callback for the controller
        }

        if (params->taskName != nullptr) {
            UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(nullptr);
            Serial.print(">");
            Serial.print(params->taskName);
            Serial.print("Mark:");
            Serial.println(stackHighWaterMark);
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
