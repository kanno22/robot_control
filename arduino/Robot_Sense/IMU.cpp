#include "IMU.h"

IMU::IMU()
{
  AccX=0;
  AccY=0;
  AccZ=0;
  Temp=0;
  GyroX=0;
  GyroY=0;
  GyroZ=0;
   
  acc_x=0.0;
  acc_y=0.0;
  acc_z=0.0;
  gyro_x=0.0;
  gyro_y=0.0;
  gyro_z=0.0;
  
  roll=0.0;
  pitch=0.0;
  yaw=0.0;
}

void IMU::IMU_Init()
{
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

    //  range of accel
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);  //  0x00:2g, 0x08:4g, 0x10:8g, 0x18:16g
  Wire.endTransmission();

  #ifndef MADGWICK
    MadgwickFilter.begin(100);
  #endif
}

void IMU::IMU_sense()
{
    //  send start address
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_AX);
  Wire.endTransmission();  
  //  request 14bytes (int16 x 7)
  Wire.requestFrom(MPU6050_ADDR, 14);
  //  get 14bytes
  AccX = Wire.read() << 8;  AccX |= Wire.read();
  AccY = Wire.read() << 8;  AccY |= Wire.read();
  AccZ = Wire.read() << 8;  AccZ |= Wire.read();
  Temp = Wire.read() << 8;  Temp |= Wire.read();  //  (Temp-12421)/340.0 [degC]
  GyroX = Wire.read() << 8; GyroX |= Wire.read();
  GyroY = Wire.read() << 8; GyroY |= Wire.read();
  GyroZ = Wire.read() << 8; GyroZ |= Wire.read();

  acc_x=(AccX-AX_OFFSET)/4096.0;//16384.0:2g, 8192:4g, 4096:8g, 2048:16g
  acc_y=(AccY-AY_OFFSET)/4096.0;
  acc_z=(AccZ-AZ_OFFSET)/4096.0;

  gyro_x=(GyroX-GX_OFFSET)/131.0;
  gyro_y=(GyroY-GY_OFFSET)/131.0;
  gyro_z=(GyroZ-GZ_OFFSET)/131.0;

  #ifndef MADGWICK
    MadgwickFilter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);//フィルタリンク
    //ヨー・ロール・ピッチ角の算出
    roll  = MadgwickFilter.getRollRadians();
    pitch = MadgwickFilter.getPitchRadians();
    yaw   = MadgwickFilter.getYawRadians();  
  #endif
}
