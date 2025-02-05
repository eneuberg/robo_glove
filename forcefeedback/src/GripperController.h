#pragma once

#include <Arduino.h>
#include "MotorDriver.h"

typedef uint32_t msInterval;


class GripperController
{
    public:
        GripperController();
        void begin();
        void calibrate();

    private:

        //MotorDriver thumb;
        MotorDriver index;
        MotorDriver middle;
        MotorDriver ring;
        MotorDriver pinky;

        static constexpr size_t motorCount = 4;
        MotorDriver* motors[motorCount] = { &index, &middle, &ring, &pinky };

        msInterval ditherInterval;
        msInterval pidInterval;
        msInterval updateInterval;

        String serialBuffer;

        void update();
    	
        float currentGripperValue = 25.0f;

};