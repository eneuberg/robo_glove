#include "GripperController.h"


struct MotorTaskParams {
    MotorDriver* instance;        // Array of MotorDriver pointers
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
    
    static MotorTaskParams indexDitherParams = {
        .instance = &index,  
        .interval = ditherInterval,
        .callback = [](MotorDriver* driver) { driver->dither(); },
        .taskName = "indexMotorTask"
    };

    static MotorTaskParams indexPidParams = {
        .instance = &index,
        .interval = pidInterval,
        .callback = [](MotorDriver* driver) { driver->pid(); },
        .taskName = "indexPIDTask"
    };

    static MotorTaskParams thumbDitherParams = {
        .instance = &thumb,
        .interval = ditherInterval,
        .callback = [](MotorDriver* driver) { driver->dither(); },
        .taskName = "thumbMotorTask"
    };

    static MotorTaskParams thumbPidParams = {
        .instance = &thumb,
        .interval = pidInterval,
        .callback = [](MotorDriver* driver) { driver->pid(); },
        .taskName = "thumbPIDTask"
    };

    static ControllerTaskParams controllerParams = {
        .instance = this,
        .interval = updateInterval,
        .callback = [](GripperController* ctrl) { ctrl->update(); },
        .taskName = "ControllerTask"
    };

    //xTaskCreate(motorTaskRunner, indexDitherParams.taskName, 4048, &indexDitherParams, 1, nullptr);
    xTaskCreate(motorTaskRunner, indexPidParams.taskName, 4048, &indexPidParams, 1, nullptr);
    //xTaskCreate(motorTaskRunner, thumbDitherParams.taskName, 2048, &thumbDitherParams, 1, nullptr);
    //xTaskCreate(motorTaskRunner, thumbPidParams.taskName, 2048, &thumbPidParams, 1, nullptr);
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

    MotorDriver* driver = params->instance;  // Array of MotorDriver objects
    TickType_t period = pdMS_TO_TICKS(params->interval); // Interval as ticks
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (driver != nullptr) {
            params->callback(driver); 
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
            // Optional: Debugging for missed deadlines
            Serial.print(">");
            Serial.print(params->taskName);
            Serial.println("TaskMiss:1");
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
