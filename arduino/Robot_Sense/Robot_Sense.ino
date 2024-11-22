#include "IMU.h"
#include "Encode.h"

//PCから受け取る値
int Receive_flag=0.0;//PCから受信したフラグ
bool Receive=false;//true：受信 false：受信していない

//エンコーダ
Encode encoder;

//IMU
IMU imu;

//
long Count=0;
float Time_f=0.0;
float dt=0.01;//10ms
unsigned long Current_time=0;
//unsigned long Time=0;
unsigned long Old_time=0;
unsigned long DT=0;
unsigned long Start_time=0;

void setup() 
{
  Serial.begin(115200);

  encoder.Encode_Init();
  imu.IMU_Init();

}

void loop() 
{
  Current_time=millis();//micros()
  encoder.Encode_count();
  imu.IMU_sense();//ロール・ピッチ・ヨー角の算出
  

  serial_read();
  Control();
}

void serial_read()
{
  
  if(Serial.available() >0)
  {
     Receive_flag = Serial.parseInt();
     Receive=true;
    
    while(Serial.available() > 0)
    {
      char t=Serial.read();//受信バッファクリア
    }

  }

}

void serial_write()
{
  Serial.print((float)((Current_time-Start_time)/1000.0),4);
  Serial.print(",");
  Serial.print(encoder.count_R);
  Serial.print(",");
  Serial.print(encoder.count_L);
  Serial.print(",");
  Serial.print(imu.acc_x,4);
  Serial.print(",");
  Serial.print(imu.acc_y,4);
  Serial.print(",");
  Serial.print(imu.acc_z,4);
  Serial.print(",");
////  Serial.print(imu.gyro_x,4);
////  Serial.print(",");
////  Serial.print(imu.gyro_y,4);
////  Serial.print(",");
////  Serial.println(imu.gyro_z,4);
//  
  Serial.print(imu.yaw,4);
  Serial.print(",");
  Serial.print(imu.roll,4);
  Serial.print(",");
  Serial.println(imu.pitch,4);
}

void Control()
{
  if(Receive==true)
  { 
    serial_write();
    
    if(Count==0)
    {
      Start_time=millis();
    }
    
    Counter();
    
    //Receive=false;     
  }
  
  
}

void Counter()
{

  Count++;
  //Time+=dt;
}
