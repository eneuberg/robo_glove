#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include "FeedbackFuncs.h"
#include "MotorDriver.h"

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
//SimpleKalmanFilter filter(100, 1000, 10);

int timestep = 0;


MotorDriver motorDriver(A0, 9, 10, 450, true);

float maxAggressiveness = 5.0f;
int aggPin = A5;
float currentAggressiveness = 1.0f;

void test();

void setup() {
  Serial.begin(9600);
  pinMode(aggPin, INPUT);
}

void loop() 
{
  motorDriver.update();
  float mappedAggressiveness = mapFloat(analogRead(aggPin), 0, 1023, 0, maxAggressiveness);
  if (abs(mappedAggressiveness - currentAggressiveness) > 0.1f) {
    currentAggressiveness = mappedAggressiveness;
    motorDriver.setAggressiveness(currentAggressiveness);
  }
  Serial.print(">currentAggressiveness:");
  Serial.println(currentAggressiveness);
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