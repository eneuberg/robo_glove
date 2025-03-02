#pragma once

#include <Arduino.h>
#include "MotorDriver.h"
#include "BitPackedQueue12.h"
#include <functional>

typedef uint32_t msInterval;


class GripperController
{
    public:
        GripperController();
        void begin();
        void calibrate();

    private:
        bool recording = false;
        int recordingLedPin = 27;

        int buttonPin = 14;
        int buttonPressTime = 0;
        int buttonPressDebounce = 1000;

        MotorDriver thumb;
        MotorDriver index;
        MotorDriver middle;
        MotorDriver ring;
        MotorDriver pinky;

        BitPackedQueue12 thumbRecord;
        BitPackedQueue12 indexRecord;
        BitPackedQueue12 middleRecord;
        BitPackedQueue12 ringRecord;
        BitPackedQueue12 pinkyRecord;

        static constexpr size_t motorCount = 4;
        MotorDriver* motors[motorCount] = { &thumb, &index, &middle, &pinky };
        BitPackedQueue12* records[motorCount] = { &thumbRecord, &indexRecord, &middleRecord, &pinkyRecord };

        msInterval ditherInterval;
        msInterval pidInterval;
        msInterval updateInterval;

        String serialBuffer;

        void update();
    	
        float currentGripperValue = 25.0f;

};