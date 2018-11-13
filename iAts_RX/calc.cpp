/**
*
* Calculation from cesco1: https://github.com/Cesco1/ManualRTH/blob/master/Nav.ino
*/

#include <math.h>
#include "calc.h"

float lonScale = 1.0f;

void calcTargetDistanceAndHeading(Tracker *tracker, Airplane *target) {
  int16_t dLat = tracker->lat - target->lat;
  int16_t dLon = (tracker->lon - target->lon);// * lonScale;
  target->distance = uint16_t(sqrt(sq(dLat) + sq(dLon)) * 1.113195f);
  target->heading = uint16_t(atan2(dLon, dLat) * 572.90f);
}

void setHome(Tracker *tracker, Airplane *target) {
  //todo check if this is correct
  float rads = (abs(float(target->lat)) / 10000000.0) * 0.0174532925;
  lonScale = cos(rads);

  tracker->lat = target->lat;
  tracker->lon = target->lon;
  tracker->alt = target->alt;
  HOME_SETED = true;
}

/*
计算两个经纬度之间的距离
*/
float distance_between(float lat1, float long1, float lat2, float long2)
{
	// returns distance in meters between two positions, both specified
	// as signed decimal-degrees latitude and longitude. Uses great-circle
	// distance computation for hypothetical sphere of radius 6372795 meters.
	// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
	// Courtesy of Maarten Lamers
	float delta = radians(long1 - long2);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6372795;
}

/*
计算坐标点1与坐标点2之间相对于正北的夹角
*/
float course_to(float lat1, float long1, float lat2, float long2)
{
	// returns course in degrees (North=0, West=270) from position 1 to position 2,
	// both specified as signed decimal-degrees latitude and longitude.
	// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
	// Courtesy of Maarten Lamers
	float dlon = radians(long2 - long1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float a1 = sin(dlon) * cos(lat2);
	float a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += TWO_PI;
	}
	return degrees(a2);
}

float easeTilt(float t, float b, float c, float d) 
{
  #ifdef EASE_OUT_QRT
  return easeOutQuart(t, b, c, d);
  #endif
  #ifdef EASE_INOUT_QRT
  return easeInOutQuart(t, b, c, d);
  #endif
  #ifdef EASE_OUT_CIRC
   return easeOutCirc(t, b, c, d);
  #endif
}

float easeOutQuart(float t, float b, float c, float d) 
{
    return -c * ((t=t/d-1)*t*t*t - 1) + b;
}

float easeInOutQuart(float t, float b, float c, float d) 
{
  t /= d/2;
  if (t < 1) return c/2*t*t*t*t + b;
  t -= 2;
  return -c/2 * (t*t*t*t - 2) + b;
}

float easeOutCirc(float t, float b, float c, float d) 
{
  t /= d/2;
  if (t < 1) return -c/2 * (sqrt(1 - t*t) - 1) + b;
  t -= 2;
  return c/2 * (sqrt(1 - t*t) + 1) + b;
}
