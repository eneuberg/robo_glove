#include <Arduino.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>

Servo servo;
int reboundSpeed = 1;
int lowerBound = 0;
int upperBound = 650;
int feedbackAngle = 150;

bool wasOutOfBounds = false;

float timestep = 0;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
SimpleKalmanFilter filter(20, 100, 10);

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
}

void loop() 
{
  int poti = analogRead(A0);
  Serial.print(">poti:");
  Serial.println(poti);

  float estimate = filter.updateEstimate(poti);
  Serial.print(">filter:");
  Serial.println(int(floor(estimate)));

  int theoreticalAngle = servo.read();
  Serial.print(">theoreticalAngle:");
  Serial.println(theoreticalAngle);

  int actualAngle = map(estimate, lowerBound, upperBound, 180, 0);
  Serial.print(">actualAngle:");
  Serial.println(actualAngle);
  
  /*
  timestep = timestep + 0.05;

  int sinValue = int(floor(sin(timestep) * 90) + 90);
  Serial.print(">sin:");
  Serial.println(sinValue);
  servo.write(sinValue);
  
  */

 
  int isOutOfBounds = feedbackAngle - feedbackAngle;
  Serial.print(">error:");
  Serial.println(outOfBounds);

  if (isOutOfBounds) {
    if (!wasOutOfBounds) {
      servo.attach(8);
      wasOutOfBounds = true;
      servo.write(int(floor(actualAngle-reboundSpeed)));
    }
    //servo.write(int(floor(actualAngle-reboundSpeed)));
  }
  else {
    if (wasOutOfBounds) {
      servo.detach();
      wasOutOfBounds = false;
    }
  }
}