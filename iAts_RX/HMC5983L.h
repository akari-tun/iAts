#ifndef HMC5983L_h
#define HMC5983L_h

#include <Arduino.h>

#define HMC5983L_Address           0x1E
#define ModeRegister               0x02
#define DataRegisterBegin          0x03
#define HMC59X3_R_CONFA            0x00
#define HMC59X3_R_CONFB            0x01
#define HMC59X3_R_MODE             0x02
#define HMC59X3_X_SELF_TEST_GAUSS  (+1.16)   //!< X axis level when bias current is applied.
#define HMC59X3_Y_SELF_TEST_GAUSS  (+1.16)   //!< Y axis level when bias current is applied.
#define HMC59X3_Z_SELF_TEST_GAUSS  (+1.08)   //!< Y axis level when bias current is applied.
#define HMC_POS_BIAS               0x01
#define HMC_NEG_BIAS               0x02

class HMC5983L {
    public:

    void init(double *pMagGain);
    bool read(int *pVals);

    private:

    uint8_t bias_collect(uint8_t bias);
    void write(int address, int data);
};

#endif