#pragma once

#include <math.h>
#include <Arduino.h>

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

inline double sawtooth(double t, double period, double amplitude) {
    if (period <= 0) {
        return 0.0; // Avoid division by zero
    }

    // Wrap time t within one period using fmod
    double fractionalPart = fmod(t, period);
    if (fractionalPart < 0) {
        fractionalPart += period; // Handle negative times
    }

    // Map to range [-amplitude, +amplitude]
    return amplitude * (2.0 * (fractionalPart / period) - 1.0);
}

inline double sineWave(double t, double period, double amplitude) {
    if (period <= 0) {
        return 0.0; // Avoid division by zero
    }

    // Convert period to angular frequency
    double omega = 2.0 * PI / period;

    // Evaluate sine wave
    return amplitude * sin(omega * t);
}

inline int intClamp(int x, int min, int max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}