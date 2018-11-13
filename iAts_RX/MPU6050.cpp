#include <Arduino.h>
#include <Wire.h>
#include <Math.h>
#include "MPU6050.h"

const int nValCnt = 7; //一次读取寄存器的数量
const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据

void MPU6050::init() {
  write(0x6B, 0); //启动MPU6050设备

  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x19);     // Write to all four registers at once
  Wire.write(7);        // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  Wire.write(0x00);     // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  Wire.write(0x00);     // Set Gyro Full Scale Range to ±250deg/s
  Wire.write(0x00);     // Set Accelerometer Full Scale Range to ±2g
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x6B);
  Wire.write(0x03);     // PLL with Z axis gyroscope reference and disable sleep mode
  Wire.endTransmission();

  calibration(); //执行校准
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void MPU6050::write(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void MPU6050::read(int *pVals) {
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(0x3B);
  Wire.requestFrom(MPU6050_Address, nValCnt * 2);
  Wire.endTransmission();

  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }

  for (int i = 0; i < 3; ++i) {
    pVals[i] = pVals[i] - calibData[i];
  }

  for (int i = 4; i < 7; ++i) {
    pVals[i] = pVals[i] - calibData[i];
  }
}

//对大量读数进行统计，校准平均偏移量
void MPU6050::calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    read(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void MPU6050::rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}