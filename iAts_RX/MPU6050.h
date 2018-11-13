#ifndef MPU6050_h
#define MPU6050_h

#include <arduino.h>

#define MPU6050_Address           0x68

class MPU6050
{
    public:

    void init();
    void read(int *pVals);
    void calibration();

    private:

    void write(int nReg, unsigned char nVal);
    void rectify(int *pReadout, float *pRealVals);
};

#endif