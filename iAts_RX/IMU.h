#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <Kalman.h>
#include "MPU6050.h"
#ifdef COMPASS_HMC5983L
    #include "HMC5983L.h"
#endif
#ifdef COMPASS_QMC5883L
    #include "QMC5883L.h"
#endif


class IMU
{
    public:

    void init();
    double getHeading();
    void calibrate_compass(uint16_t pan_center);

    private:

    /* IMU Data */
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    double magX, magY, magZ;
    int16_t tempRaw;

    double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
    double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

    uint32_t timer;
    Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

    MPU6050 mpu6050;
#ifdef COMPASS_HMC5983L
    HMC5983L compass;
#endif
#ifdef COMPASS_QMC5883L
    QMC5883L compass;
#endif

    int accGyr[7];
    int magADC[3];
    int magOffset[3];
    double magGain[3];

    void measure();
    void updatePitchRoll();
    void updateYaw();
};

#endif