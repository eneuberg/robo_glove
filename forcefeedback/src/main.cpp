#include <Arduino.h>
#include "FeedbackFuncs.h"
#include "MotorDriver.h"

const int intervalMs = 10;
unsigned long lastMicros = 0;

MotorDriver motorDriver(34, 14, 12, 1400, true);

void setup() {
  Serial.begin(115200);
}

void loop() 
{
  unsigned long currentMicros = micros();
  if (currentMicros - lastMicros >= (intervalMs * 1000)) {
    motorDriver.update();
    lastMicros = currentMicros;
  }
  Serial.print(">timeTaken:");
  Serial.println(micros() - currentMicros);
}
