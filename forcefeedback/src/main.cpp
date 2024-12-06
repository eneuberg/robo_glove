#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include "FeedbackFuncs.h"

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
SimpleKalmanFilter filter(100, 1000, 10);


int timestep = 0;

void test();

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(9, OUTPUT);
}

void loop() 
{
  timestep++;
  int poti = analogRead(A0);
  Serial.print(">poti:");
  Serial.println(poti);

  float estimate = filter.updateEstimate(poti);

  int pwmValue = getFeedbackPwm(poti, 700, true);
  Serial.print(">pwmValue:");
  Serial.println(pwmValue);
  analogWrite(10, pwmValue);
  //test();
  
  Serial.print(">timestep:");
  Serial.println(timestep);
  
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