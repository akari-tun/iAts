#ifndef QMC5883L_h
#define QMC5883L_h

#include <Arduino.h>

#define QMC5883L_Address           0x0D
#define SET_RESET_Period_Register  0x0B
#define Control_Register           0x09
#define Data_Register_Begin        0x00
#define QMC_POS_BIAS               0x01
#define QMC_NEG_BIAS               0x02

class QMC5883L {
    public:

    void init(double *pMagGain);
    bool read(int *pVals);

    private:

    uint8_t bias_collect(uint8_t bias);
    void write(int address, int data);
};

#endif