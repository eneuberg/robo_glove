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
    : index("index", 36, 0, 2, true),
    thumb("thumb", 32, 21, 3, true),
    middle("middle", 39, 4, 16, true),
    ring("ring", 34, 18, 19, true),
    pinky("pinky", 35, 5, 17, true), // TODO switch around because of pin order on esp (conventional)
    ditherInterval(8),
    pidInterval(10),
    updateInterval(50)
{}

void GripperController::begin()  {
    
    for (int i = 0; i < motorCount; i++) {
        motors[i]->begin();
        motors[i]->setPidActive(false);
    }

    pinMode(recordingLedPin, OUTPUT);
    recording = true;
    digitalWrite(recordingLedPin, HIGH);

    static TaskParams<MotorDriver> pidMotorParams = {
        .instances = motors,
        .instanceCount = motorCount,
        .interval = pidInterval,
        .callback = [](MotorDriver* md) { md->pid(); },
        .taskName = "PidMotorTask"
    };

    static TaskParams<MotorDriver> ditherMotorParams = {
        .instances = motors,
        .instanceCount = motorCount,
        .interval = ditherInterval,
        .callback = [](MotorDriver* md) { md->dither(); },
        .taskName = "DitherMotorTask"
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
        taskRunner<MotorDriver>,
        ditherMotorParams.taskName, 
        4048, 
        &ditherMotorParams, 
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

    index.mapToGripperSetpoint(currentGripperValue);
    thumb.mapToGripperSetpoint(currentGripperValue);
    
    */

   for (int i = 0; i < motorCount; i++) {
        float estimate = motors[i]->getEstimate();
        Serial.print(">");
        Serial.print(motors[i]->getName());
        Serial.print("Estimate:");
        Serial.println(estimate);
   }

    
    bool buttonPressed = false;
    int button = digitalRead(buttonPin);
    if (button == LOW) {
        int time = millis();
        if (time - buttonPressTime > buttonPressDebounce) {
            buttonPressTime = time;
            buttonPressed = true;
        }
    }

    Serial.print(">buttonPressed:");
    Serial.println(button);

    Serial.print(">recording:");
    Serial.println(recording);

    if (recording) {
        // In recording mode, push current estimates into the queues.
        // Also, check if any of the queues is full OR the button was pressed.
        bool stopRecording = false;
        for (int i = 0; i < motorCount; i++) {
            // If any record queue is full, mark that we should stop recording.
            if (records[i]->full()) {
                stopRecording = true;
                break;
            }
        }
        // If the button is pressed (or any queue was full), we want to stop recording.
        if (buttonPressed || stopRecording) {
            digitalWrite(recordingLedPin, LOW);
            recording = false;
            for (int i = 0; i < motorCount; i++) {
                motors[i]->setPidActive(true);
            }
            // Optionally, print a message or perform additional actions.
            Serial.println("Recording stopped.");
        }
        else {
            // Otherwise, push the current estimate for each motor.
            for (int i = 0; i < motorCount; i++) {
                uint16_t sample = static_cast<uint16_t>(motors[i]->getEstimate());
                if (!records[i]->push(sample)) {
                    Serial.print(">");
                    Serial.print(motors[i]->getName());
                    Serial.println(" ERROR: push failed in UpdateTask!");
                }
            }
        }
    }
    else {
        // In playback mode, pop values from the queues and set motor setpoints.
        for (int i = 0; i < motorCount; i++) {
            uint16_t outVal;
            if (records[i]->pop(outVal)) {
                motors[i]->setSetpoint(static_cast<int>(outVal));
            }
        }
        
        // --- Missing Part: Check if we need to restart recording ---
        // If the button is pressed while not recording, clear all queues and restart recording.
        if (buttonPressed) {
            // Clear all queues.
            for (int i = 0; i < motorCount; i++) {
                records[i]->clear(); // Assuming a clear() method exists.
            }
            // Optionally, light up the LED to indicate recording has started.
            digitalWrite(recordingLedPin, HIGH);
            recording = true;
            for (int i = 0; i < motorCount; i++) {
                motors[i]->setPidActive(false);
            }
            Serial.println("Recording restarted.");
        }
    }
}

void GripperController::calibrate() {
    int ledPin = 12;
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    //int button = digitalRead(buttonPin);
    //while (button == HIGH) {
    //    button = digitalRead(buttonPin);
    //    delay(100);
    //}
    //digitalWrite(ledPin, LOW);
    //delay(500);
    //digitalWrite(ledPin, HIGH);
    //for (int i = 0; i < motorCount; i++) {
    //    //motors[i]->calibrateFeedforward();
    //}
//
    while (true) {
        for (int i = 0; i < motorCount; i++) {
            motors[i]->calibrateRanges();
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