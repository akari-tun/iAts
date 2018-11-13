#ifndef SERVOS_H
#define SERVOS_H
/**
 * Functions for handling tilt and pan servos
 *
 * The servos needs to be connected to the hardware PWMs
 * of the arduino, these are Digital 9 (PAN) and Digital 10 (Tilt)
 */

#include <avr/io.h>
#include "config.h"
#include "defines.h"

// use this define to set the tilt servo pulse duration directly
#define TILT_SERVO OCR1B
#define PAN_SERVO OCR1A
#define SET_PAN_SERVO_SPEED(A) PAN_SERVO = A * 2
#define SET_TILT_SERVO_SPEED(A) TILT_SERVO = A * 2

void initServos(Parameter *param);
//void initServos();
void moveServoTilt(float value);
void servo_tilt_update(int16_t &_servo_tilt_must_move);
#endif
