#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Arduino.h>
#include<MadgwickAHRS.h>
//黄色線:SDL
//緑線:SDA
#define MPU6050_ADDR 0x68
#define MPU6050_AX  0x3B
#define MPU6050_AY  0x3D
#define MPU6050_AZ  0x3F
#define MPU6050_TP  0x41    //  data not used
#define MPU6050_GX  0x43
#define MPU6050_GY  0x45
#define MPU6050_GZ  0x47

#define AX_OFFSET 210
#define AY_OFFSET -73
#define AZ_OFFSET 302
#define GX_OFFSET -440
#define GY_OFFSET -111
#define GZ_OFFSET -239


//#define MADGWICK

class IMU
{
  private:
    short int AccX, AccY, AccZ;
    short int Temp;
    short int GyroX, GyroY, GyroZ;
       
    Madgwick MadgwickFilter;

  public:
    double acc_x,acc_y,acc_z;
    double gyro_x,gyro_y,gyro_z;
    double roll,pitch,yaw;
    IMU();
    void IMU_Init();
    void IMU_sense();
};

#endif
