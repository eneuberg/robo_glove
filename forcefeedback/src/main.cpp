#include <Arduino.h>
#include "FeedbackFuncs.h"
#include "MotorDriver.h"

int timestep = 0;

const int intervalMs = 20;
unsigned long lastMicros = 0;

MotorDriver motorDriver(34, 14, 12, 1700, true);

void test();

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
}



void test() {
  double sinValue = sin(timestep/10.0);
  Serial.print(">sinValue:");
  Serial.println(sinValue);

  float mapValue = mapFloat(sinValue, 0, 1, 0, 255);
  Serial.print(">mapValue:");
  Serial.println(mapValue);

  if (sinValue > 0) {
    analogWrite(9,mapValue);
  }
  else {
    analogWrite(10,abs(mapValue));
  }
}