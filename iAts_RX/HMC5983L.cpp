
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "HMC5983L.h"

//static float magGain[3] = {1.0, 1.0, 1.0};
//int magZero[3];
static int32_t xyz_total[3] = {0, 0, 0};

void HMC5983L::init(double *pMagGain)
{
  bool bret = true;

  write(HMC59X3_R_CONFB, 2 << 5);
  write(HMC59X3_R_MODE, 1);
  delay(100);

  //get one sample and discard it
  //while (!read());

  if (bias_collect(0x010 + HMC_POS_BIAS))
    bret = false;
  if (bias_collect(0x010 + HMC_NEG_BIAS))
    bret = false;

  if (bret)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
      pMagGain[axis] = 820.0 * HMC59X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[axis]; // note: xyz_total[axis] is always positive
  }
  else
  {
    pMagGain[0] = 1.0;
    pMagGain[1] = 1.0;
    pMagGain[2] = 1.0;
  }

  write(HMC59X3_R_CONFA, 0x78); //Configuration Register A  -- output rate: 75Hz ; normal measurement mode
  write(HMC59X3_R_CONFB, 0x20); //Configuration Register B  -- configuration gain 1.3Ga
  write(HMC59X3_R_MODE, 0x00);  //Mode register             -- continuous Conversion Mode
  delay(100);
}

bool HMC5983L::read(int *pMagAdc)
{
  Wire.beginTransmission(HMC5983L_Address);
  Wire.write(DataRegisterBegin);
  Wire.requestFrom(HMC5983L_Address, 6);
  if (Wire.available() == 6)
  {
    //X
    pMagAdc[0] = (Wire.read() << 8) | Wire.read();
    //Z
    pMagAdc[1] = (Wire.read() << 8) | Wire.read();
    //Y
    pMagAdc[2] = (Wire.read() << 8) | Wire.read();
  }

  Wire.endTransmission();

  if (pMagAdc[0] == -4096 || pMagAdc[1] == -4096 || pMagAdc[2] == -4096)
  {
    // no valid data available
    return false;
  }
  return true;
}

uint8_t HMC5983L::bias_collect(uint8_t bias)
{
  int16_t abs_magADC;
  int magADC[3];
    
  write(HMC59X3_R_CONFA, bias); // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
  
  for (uint8_t i = 0; i < 10; i++)
  { 
    // Collect 10 samples
    write(HMC59X3_R_MODE, 1);

    delay(100);
    while (!read(magADC)); // Get the raw values in case the scales have already been changed.

    for (uint8_t axis = 0; axis < 3; axis++)
    {
      abs_magADC = abs(magADC[axis]);
      xyz_total[axis] += abs_magADC; // Since the measurements are noisy, they should be averaged rather than taking the max.
      if ((int16_t)(1 << 12) < abs_magADC)
        return false; // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  return true;
}

void HMC5983L::write(int address, int data)
{
  Wire.beginTransmission(HMC5983L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}