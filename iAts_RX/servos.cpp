#include "calc.h"
#include "servos.h"

float _lasttilt = 0.0;
uint8_t _tilt_pos = 0;
static Parameter *PTParam;

// Initializes both hardware PWMs to be used for tilt and pan servos.
//void initServos()
void initServos(Parameter *param)
{
	PTParam = param;
	// Set OC1A (PB1) and OC1B (PB2) to output.
	// these are our hardware PWM ports.

	DDRB |= _BV(PORTB1) | _BV(PORTB2);

	// Setup of Timer1, Prescaler 8, 16bit fast pwm
	TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
	TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);
	// we need to multiply our pwm value by 2
	OCR1A = PTParam->pan_center * 2;
	OCR1B = PTParam->tilt_0 * 2;
	//OCR1A = PAN_0 * 2;
	//OCR1B = TITL_0 * 2;
	// ((16mhz / 8 Prescaler) / 50hz) - 1 = 40000
	ICR1 = 39999;
}

void moveServoTilt(float value)
{
	int _pwmpulse;
	float easingout;
	if (abs(_lasttilt - value) > TILT_EASING_MIN_ANGLE)
	{
		for (int pos = 0; pos < TILT_EASING_STEPS; pos++)
		{
			if (_lasttilt <= value)
				easingout = _lasttilt + easeTilt(pos, 0, value - _lasttilt, TILT_EASING_STEPS);
			else
				easingout = _lasttilt - easeTilt(pos, 0, _lasttilt - value, TILT_EASING_STEPS);
			_pwmpulse = (int)map(easingout, 0, 90, PTParam->tilt_0, PTParam->tilt_90);
			//_pwmpulse = (int)map(easingout, 0, 90, TITL_0, TITL_90);
			SET_TILT_SERVO_SPEED(_pwmpulse);
			delay(15);
		}
	}
	else
		SET_TILT_SERVO_SPEED(map(value, 0, 90, PTParam->tilt_0, PTParam->tilt_90));
		//SET_TILT_SERVO_SPEED(map(value, 0, 90, TITL_0, TITL_90));
	_lasttilt = (float)value;
}

void servo_tilt_update(int16_t &_servo_tilt_must_move)
{
	int _pwmpulse;
	float easingout;
	if (_servo_tilt_must_move < 0)
		_tilt_pos = 0;
	if (!SERVO_TILT_HAS_ARRIVED && _servo_tilt_must_move > -1)
	{
		if (abs(_lasttilt - _servo_tilt_must_move) > TILT_EASING_MIN_ANGLE)
		{
			if (_tilt_pos < TILT_EASING_STEPS)
			{
				if (_lasttilt <= _servo_tilt_must_move)
					easingout = _lasttilt + easeTilt(_tilt_pos, 0, _servo_tilt_must_move - _lasttilt, TILT_EASING_STEPS);
				else
					easingout = _lasttilt - easeTilt(_tilt_pos, 0, _lasttilt - _servo_tilt_must_move, TILT_EASING_STEPS);
				_pwmpulse = (int)map(easingout, 0, 90, PTParam->tilt_0, PTParam->tilt_90);
				//_pwmpulse = (int)map(easingout, 0, 90, TITL_0, TITL_90);
				SET_TILT_SERVO_SPEED(_pwmpulse);

#if TILT_EASING_MILIS
				delay(TILT_EASING_MILIS);
#endif
				_tilt_pos++;
			}
			else
			{
				if (_tilt_pos == TILT_EASING_STEPS)
				{
					SET_TILT_SERVO_SPEED(map(_servo_tilt_must_move, 0, 90, PTParam->tilt_0, PTParam->tilt_90));
					//SET_TILT_SERVO_SPEED(map(_servo_tilt_must_move, 0, 90, TITL_0, TITL_90));
#if TILT_EASING_MILIS
					delay(TILT_EASING_MILIS);
#endif
					_lasttilt = (float)_servo_tilt_must_move;
					_tilt_pos = 0;
					SERVO_TILT_HAS_ARRIVED = true;
					_servo_tilt_must_move = -1;
				}
			}
		}
		else
		{
			SERVO_TILT_HAS_ARRIVED = true;
			_servo_tilt_must_move = -1;
		}
	}
}