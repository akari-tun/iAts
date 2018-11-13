#include <Arduino.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <inttypes.h>
#include <math.h>
#include <EEPROM.h>
#include "config.h"
#include "servos.h"
#include "eeprom_functions.h"
#include "MPU6050.h"
#include "IMU.h"
#ifdef COMPASS_HMC5983L
    #include "HMC5983L.h"
#endif
#ifdef COMPASS_QMC5883L
    #include "QMC5883L.h"
#endif

void IMU::init()
{
    delay(100); // Wait for sensors to get ready
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    mpu6050.init();
    compass.init(magGain);

    mpu6050.calibration(); //执行校准

    delay(100);

    measure();
    updatePitchRoll();
    updateYaw();

    kalmanX.setAngle(roll); // First set roll starting angle
    kalmanY.setAngle(pitch); // Then pitch
    kalmanZ.setAngle(yaw); // And finally yaw

    timer = micros(); // Initialize the timer

    for (uint8_t axis = 0; axis < 3; axis++)
    {
      magOffset[axis] = LoadFromEEPROM_u16(axis * 2);
    }
}

double IMU::getHeading() {
  /* Update all the IMU values */
  measure();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // /* Roll and pitch estimation */
  updatePitchRoll();
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    // compAngleX = roll;
    kalAngleX = roll;
    // gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  /* Yaw estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    // compAngleZ = yaw;
    kalAngleZ = yaw;
    // gyroZangle = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

  /* Estimate angles using gyro only */
  // gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  // gyroYangle += gyroYrate * dt;
  // gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
  //gyroYangle += kalmanY.getRate() * dt;
  //gyroZangle += kalmanZ.getRate() * dt;

  /* Estimate angles using complimentary filter */
  // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  // compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  // if (gyroXangle < -180 || gyroXangle > 180)
  //   gyroXangle = kalAngleX;
  // if (gyroYangle < -180 || gyroYangle > 180)
  //   gyroYangle = kalAngleY;
  // if (gyroZangle < -180 || gyroZangle > 180)
  //   gyroZangle = kalAngleZ;

  return kalAngleZ;
}

void IMU::measure()
{
    mpu6050.read(accGyr);
    accX = accGyr[0];
    accY = -accGyr[1];
    accZ = accGyr[2];
    tempRaw = accGyr[3];
    gyroX = -accGyr[4];
    gyroY = accGyr[5];
    gyroZ = -accGyr[6];

		compass.read(magADC);
    magX = magADC[0];
    magZ = magADC[1];
    magY = magADC[2];
}

void IMU::updatePitchRoll() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
}

void IMU::updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  // magX *= magGain[0];
  // magZ *= magGain[1];
  // magY *= magGain[2];

  // magX -= magOffset[0];
  // magZ -= magOffset[1];
  // magY -= magOffset[2];

  magX = magGain[0] * (magX - magOffset[0]);
  magZ = magGain[1] * (magZ - magOffset[1]);
  magY = magGain[2] * (magY - magOffset[2]);

  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);

  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
  yaw *= -1;
}

void IMU::calibrate_compass(uint16_t pan_center)
{
	uint16_t pwm = 2000;
	static int16_t magZeroTempMin[3];
	static int16_t magZeroTempMax[3];
	byte axis = 0;

	while (!compass.read(magADC));

	for (axis = 0; axis < 3; axis++)
	{
		magOffset[axis] = 0;
		magZeroTempMin[axis] = magADC[axis];
		magZeroTempMax[axis] = magADC[axis];
	}
	
	unsigned long cal_timer = millis();
	SET_PAN_SERVO_SPEED(pwm);

	while (millis() - cal_timer < 5000)
	{
		while (!compass.read(magADC));

		for (axis = 0; axis < 3; axis++)
		{
			if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
			if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
		}

		delay(14);
	}

	uint16_t per = (pwm - pan_center) / 50;

	while (pwm > pan_center)
	{
		pwm = pwm - per;
		SET_PAN_SERVO_SPEED(pwm);
		delay(20);
	}

	delay(1000);
	pwm = 1000;
	cal_timer = millis();
	SET_PAN_SERVO_SPEED(pwm);

	while (millis() - cal_timer < 5000)
	{
		while (!compass.read(magADC));

		for (axis = 0; axis < 3; axis++)
		{
			if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
			if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
		}
		delay(14);
	}

	per = (pan_center - pwm) / 50;

	while (pwm < pan_center)
	{
		pwm = pwm + per;
		SET_PAN_SERVO_SPEED(pwm);
		delay(20);
	}

	for (axis = 0; axis < 3; axis++)
	{
		magOffset[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
		StoreToEEPROM_u16((uint16_t)magOffset[axis], axis * 2);

    // if (axis == 0) {
    //   magGain[axis] = 1.0;
    // } else {
    //   magGain[axis] = (magZeroTempMax[0] - magZeroTempMin[0]) / (magZeroTempMin[axis] - magZeroTempMax[axis]);
    // }
    // Serial.print("Offset[");Serial.print(axis);Serial.print("]:");Serial.print(magOffset[axis]); Serial.print("      Gain[");Serial.print(axis);Serial.print("]:");Serial.println(magGain[axis]); 
	}
}