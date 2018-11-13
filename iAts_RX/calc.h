#ifndef CALC_H
#define CALC_H

#include "inttypes.h"
#include "defines.h"
#include "Arduino.h"

void calcTargetDistanceAndHeading(Tracker *tracker, Airplane *target);
void setHome(Tracker *tracker, Airplane *target);
float distance_between(float lat1, float long1, float lat2, float long2);
float course_to(float lat1, float long1, float lat2, float long2);
float easeTilt(float t, float b, float c, float d);
float easeOutQuart(float t, float b, float c, float d);
float easeInOutQuart(float t, float b, float c, float d);
float easeOutCirc(float t, float b, float c, float d);
#endif
