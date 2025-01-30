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

        MotorDriver thumb;
        MotorDriver index;
        MotorDriver middle;
        MotorDriver ring;
        MotorDriver pinky;

        msInterval ditherInterval;
        msInterval pidInterval;
        msInterval updateInterval;

        String serialBuffer;

        void update();
    	
        float currentGripperValue = 25.0f;

};