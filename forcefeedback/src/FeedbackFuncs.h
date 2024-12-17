#include <math.h>
#pragma once

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

float feedbackCurve(float input, float steepness, float offset) {
    return -log10(-((input / offset) - 1)) / steepness;
}

float calcMaxValue(float steepness, float offset, float maxValueFactor) {
    if (maxValueFactor >= 1) {
        return NAN;
    }
    return feedbackCurve(offset * maxValueFactor, steepness, offset);
}

float normalizedFeedbackCurve(float input, float steepness, float offset, float maxValueFactor) {
    if (maxValueFactor >= 1 || maxValueFactor <= 0) {
        return NAN;
    }
    if (input < 0) {
        return 0;
    }
    if (input > offset * maxValueFactor) {
        return 1;
    }
    float feedback = feedbackCurve(input, steepness, offset);
    float maxValue = calcMaxValue(steepness, offset, maxValueFactor);
    return mapFloat(feedback, 0, maxValue, 0, 1);
}

float softMaterial(int unitsInFeedback) {
    return normalizedFeedbackCurve((float)unitsInFeedback, 0.15, 200, 0.999);
}

float hardMaterial(int unitsInFeedback) {
    return normalizedFeedbackCurve((float)unitsInFeedback, 0.4, 100, 0.999);
}

int getFeedbackPwm(int poti, int feedbackThreshhold, bool feedbackUp)  {
    bool isInFeedbackZone = (feedbackUp && poti > feedbackThreshhold) || (!feedbackUp && poti < feedbackThreshhold);
    int distanceFromThreshhold = abs(poti - feedbackThreshhold);
    if (!isInFeedbackZone) {
        return 0;
    }
    float feedback = softMaterial(distanceFromThreshhold);
    int pwmValue = floor(mapFloat(feedback, 0, 1, 0, 255));
    return pwmValue;
}