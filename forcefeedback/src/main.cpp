#include <Arduino.h>
#include "MotorDriver.h"

MotorDriver motorDriver(34, 14, 12, 1400, true);

void setup() {
  Serial.begin(115200);
  analogWriteResolution(10);
  motorDriver.begin();
}

void loop() 
{
  //Serial.print(">timeTaken:");
  //Serial.println(micros() - currentMicros);
}