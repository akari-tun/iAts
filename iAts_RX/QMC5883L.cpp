#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "QMC5883L.h"

//static float magGain[3] = {1.0, 1.0, 1.0};
//int magZero[3];
static int32_t xyz_total[3] = {0, 0, 0};

void QMC5883L::init(double *pMagGain)
{
  bool bret = true;

  write(SET_RESET_Period_Register, 0x01);
  write(0x20, 0x40);
  write(0x21, 0x01);
  //控制寄存器 8个位
  // 00 采样率 （OSR） 512
  // 00 量程    +/- 2Gauss)
  // 10 输出速率 100hz
  // 01 连续测量模式
  write(Control_Register, 0x09);
  delay(100);

  int tmpVal[3];
  //get one sample and discard it
  while (!read(tmpVal));

  if (bias_collect(0x010 + QMC_POS_BIAS))
    bret = false;
  if (bias_collect(0x010 + QMC_NEG_BIAS))
    bret = false;

  if (bret)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
      pMagGain[axis] = 820.0 * 1.16 * 2.0 * 10.0 / xyz_total[axis]; // note: xyz_total[axis] is always positive
  }
  else
  {
    pMagGain[0] = 1.0;
    pMagGain[1] = 1.0;
    pMagGain[2] = 1.0;
  }
}

bool QMC5883L::read(int *pMagAdc)
{
  Wire.beginTransmission(QMC5883L_Address);
  Wire.write(Data_Register_Begin);
  Wire.requestFrom(QMC5883L_Address, 6);
  if (Wire.available() == 6)
  {
    int buff[6];
    buff[0] = Wire.read();
    buff[1] = Wire.read();
    buff[2] = Wire.read();
    buff[3] = Wire.read();
    buff[4] = Wire.read();
    buff[5] = Wire.read();

    pMagAdc[0] = buff[1] << 8 | buff[0]; //Combine MSB and LSB of X Data output register  最高有效位
    pMagAdc[2] = buff[3] << 8 | buff[2]; //Combine MSB and LSB of Y Data output register
    pMagAdc[1] = buff[5] << 8 | buff[4]; //Combine MSB and LSB of Z Data output register
  }

  Wire.endTransmission();

  if (pMagAdc[0] == -4096 || pMagAdc[1] == -4096 || pMagAdc[2] == -4096)
  {
    // no valid data available
    return false;
  }
  return true;
}

uint8_t QMC5883L::bias_collect(uint8_t bias)
{
  int16_t abs_magADC;
  int magADC[3];
  for (uint8_t i = 0; i < 10; i++)
  { 
    // Collect 10 samples
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

void QMC5883L::write(int address, int data)
{
  Wire.beginTransmission(QMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}