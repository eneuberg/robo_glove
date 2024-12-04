#include <Arduino.h>
#include <SimpleKalmanFilter.h>


int lowerBound = 60;
int upperBound = 600;

int feedbackThreshhold = 100;
int targetAngle = 103;
int deadzone = 3;
float kP = 0.7; // propotional gain

unsigned long previousTime = 0; // To store the time of the previous reading
int previousPoti = 0;           // To store the previous potentiometer reading

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
*/
SimpleKalmanFilter filter(100, 1000, 10);


int timestep = 0;

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

  int actualAngle = map(estimate, lowerBound, upperBound, 180, 0);
  Serial.print(">actualAngle:");
  Serial.println(actualAngle);

  int write = (timestep % 127) * 2;
  analogWrite(9, write);

  Serial.print(">write:"); 
  Serial.println(write);

  Serial.print(">timestep:");
  Serial.println(timestep);


  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - previousTime;
  int deltaPoti = poti - previousPoti;

  float velocity = 0;

  if (deltaTime > 0) {
    // Calculate the velocity (rate of change)
    velocity = (float)deltaPoti / deltaTime; // Units: change per millisecond
  }

  previousTime = currentTime;
  previousPoti = poti;

  Serial.print(">velocity:");
  Serial.println(velocity);
}