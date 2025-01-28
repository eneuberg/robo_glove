#include <Arduino.h>
#include "GripperController.h"


void setup() {
  Serial.begin(115200);
  analogWriteResolution(10);
  while(!Serial) {
        delay(100);
    }
  GripperController controller;
  //pinMode(35, INPUT);
  //pinMode(34, INPUT);
  controller.calibrate();
  controller.begin();
}

void loop() 
{
  
  //int value = analogRead(35);
  //Serial.print(">value:");
  //Serial.println(value);
  //int value2 = analogRead(34);
  //Serial.print(">value2:");
  //Serial.println(value2);
  
  //Serial.print(">timeTaken:");
  //Serial.println(micros() - currentMicros);
}