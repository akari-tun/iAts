/* #### 电压测量 ####
 *
 *  Arduino针脚供电最大5V，电流最大40毫安左右，所以测量12V电压需要进行分压。
 *  用电阻串联可以将电压分散，两个阻值一样的电阻串联，则每个电阻两端的电压为总电压的一半，那么一大一小两个电阻串联，小电阻两端电压 = （V*(R2/R1))
 *  
 */

#include <Arduino.h>
#include "defines.h"
#include "battery.h"
#include "config.h"

uint16_t Bat_ADC_Last[BATTERYMONITORING_AVERAGE];
float Bat_denominator = (float)BATTERYMONITORING_RESISTOR_2 / (float)BATTERYMONITORING_RESISTOR_1;

void getBatVoltage(uint16_t *voltage)
{
  int n = 0;
  uint16_t Bat_ADC = (uint16_t)analogRead(VOLTAGEDIVIDER); //Hole Wert
  uint32_t Bat_AVG = (uint32_t)Bat_ADC;

  for (n = 0; n < BATTERYMONITORING_AVERAGE; n++)
  {
    Bat_AVG += (uint32_t)Bat_ADC_Last[n];
  }

  Bat_AVG /= BATTERYMONITORING_AVERAGE + 1;

  for (n = 0; n < BATTERYMONITORING_AVERAGE - 1; n++)
  {
    Bat_ADC_Last[n] = Bat_ADC_Last[n + 1];
  }

  Bat_ADC_Last[BATTERYMONITORING_AVERAGE - 1] = Bat_ADC;
  *voltage = (uint16_t)((((float)Bat_AVG / 1024.0) * BATTERYMONITORING_VREF / Bat_denominator * BATTERYMONITORING_CORRECTION) * 100);
}