#include <Arduino.h>
#include "GripperController.h"
#include "TestFuncs.h"

//GripperController controller;


void setup() {
  Serial.begin(115200);
  analogWriteResolution(10);
  while(!Serial) {
        delay(100);
    }
    testBitPackedQueue12();
  //controller.calibrate();
  //controller.begin();
}

void loop() 
{
  //testMotor();
  //testPoti();
}