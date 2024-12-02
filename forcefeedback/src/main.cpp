#include <Arduino.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>

Servo servo;

int lowerBound = 60;
int upperBound = 600;

int feedbackThreshhold = 150;
int targetAngle = 147;
int deadzone = 3;
float kP = 0.7; // propotional gain
bool feedbackActive = false;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
SimpleKalmanFilter filter(100, 1000, 10);


float timestep = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  //servo.attach(8);
}

void loop() 
{
  int poti = analogRead(A0);
  Serial.print(">poti:");
  Serial.println(poti);

  float estimate = filter.updateEstimate(poti);

  int readAngle = servo.read();
  Serial.print(">readAngle:");
  Serial.println(readAngle);

  int actualAngle = map(estimate, lowerBound, upperBound, 180, 0);
  Serial.print(">actualAngle:");
  Serial.println(actualAngle);
  
  Serial.print(">feedbackActive:");
  Serial.println(feedbackActive);

  /*
  timestep = timestep + 0.05;

  int sinValue = int(floor(sin(timestep) * 180) + 90);
  Serial.print(">sin:");
  Serial.println(sinValue);
  servo.write(sinValue);
*/

  if (!feedbackActive && actualAngle > feedbackThreshhold) {
    servo.attach(8);
    feedbackActive = true;
  }

  if (feedbackActive) {
    float error = targetAngle - actualAngle;
    Serial.print(">error:");
    Serial.println(error);

    
    float adjustedAngle = actualAngle + (error * kP);

    servo.write(adjustedAngle);

    if (abs(error) < deadzone) {
      servo.detach();
      feedbackActive = false;
    }
  }

 /*
  int isOutOfBounds = feedbackAngle - feedbackAngle;
  Serial.print(">error:");
  Serial.println(isOutOfBounds);

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
  */
}