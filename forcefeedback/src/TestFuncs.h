#pragma once

#include "HelperFuncs.h"

static int testTimestep = 0;
static int testForwardPin = 25;
static int testBackwardPin = 26;
static int testPotiPin = 36;

inline void testMotor() {
    testTimestep++;
    float sine = sin(testTimestep * 0.001);
    float mappedValue = mapFloat(sine, -1.0, 1.0, -1023, 1023);
    Serial.print(">mappedValue:");
    Serial.println(mappedValue);
    if (mappedValue > 0) {
        analogWrite(testForwardPin, mappedValue);
        analogWrite(testBackwardPin, 0);
    } else {
        analogWrite(testForwardPin, 0);
        analogWrite(testBackwardPin, abs(mappedValue));
    }
}

inline void testPoti() {
    int value = analogRead(testPotiPin);
    Serial.print(">value:");
    Serial.println(value);
}