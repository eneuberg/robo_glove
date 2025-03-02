#pragma once

#include <Arduino.h>
#include "MotorDriver.h"
#include <functional>

typedef uint32_t msInterval;


class GripperController
{
    public:
        GripperController();
        void begin();
        void calibrate();

    private:
        int buttonPin = 14;
        int buttonPressTime = 0;
        int buttonPressDebounce = 1000;

        MotorDriver thumb;
        MotorDriver index;
        MotorDriver middle;
        MotorDriver ring;
        MotorDriver pinky;

        static constexpr size_t motorCount = 5;
        MotorDriver* motors[motorCount] = { &thumb, &index, &middle, &ring, &pinky };

        msInterval ditherInterval;
        msInterval pidInterval;
        msInterval updateInterval;

        String serialBuffer;

        void update();
    	
        float currentGripperValue = 25.0f;

};