#include "IMU.h"

#define RPINA 2
#define RPINB 8
#define LPINA 3
#define LPINB 9

#define Rinterrupt 0//2番ピン
#define Linterrupt 1//3番ピン

#define PINION_RADIUS 11.2//[mm]
#define PPR 1024//512*2

#define ENCODE//encoderのONOFF

//PCから受け取る値
int Receive_flag=0.0;//PCから受信したフラグ
bool Receive=false;//true：受信 false：受信していない

//エンコーダ
volatile long Rcount;
volatile long Lcount;
float Rdisp;//脚の長さの測定値
float Ldisp;

//IMU
IMU imu;

void setup() 
{
  Serial.begin(115200);

  imu.IMU_Init();

  #ifndef ENCODE
    Init_Encode();
    attachInterrupt(Rinterrupt, REncode, CHANGE);
    attachInterrupt(Linterrupt, LEncode, CHANGE);
  #endif

}

void loop() 
{
  imu.IMU_sense();//ロール・ピッチ・ヨー角の算出

  #ifndef ENCODE  
    CallDisp();//右、左足のストロークの算出
  #endif

  serial_read();

  if(Receive==true)
  { 
        
    serial_write();
    
    Receive=false;
         
  }
  
}

void CallDisp()
{
  Rdisp=((float)Rcount/PPR)*2*PI*PINION_RADIUS;
  Ldisp=((float)Lcount/PPR)*2*PI*PINION_RADIUS;
}

void Init_Encode(void) 
{
  pinMode(RPINA, INPUT);
  pinMode(RPINB, INPUT);
  pinMode(LPINA, INPUT);
  pinMode(LPINB, INPUT);

  Rcount=0;
  Lcount=0;
}

void REncode()  //右足側エンコード
{
  int Astate = digitalRead(RPINA);
  int Bstate = digitalRead(RPINB);
  if (Astate == 1)  //A is HIGH
  {
    if (Bstate == 1)  //B is HIGH
    {
      Rcount--;
    } else  //B is LOW
    {
      Rcount++;
    }
  } else  //A is LOW
  {
    if (Bstate == 1) 
    {
      Rcount++;
    } else 
    {
      Rcount--;
    }
  }
}

void LEncode()  //左足側エンコード
{
  int Astate = digitalRead(LPINA);
  int Bstate = digitalRead(LPINB);
  if (Astate == 1)  //A is HIGH
  {
    if (Bstate == 1)  //B is HIGH
    {
      Lcount--;
    } else  //B is LOW
    {
      Lcount++;
    }
  } else  //A is LOW
  {
    if (Bstate == 1) 
    {
      Lcount++;
    } else 
    {
      Lcount--;
    }
  }
}

void serial_read()
{
  
  if(Serial.available() >0)
  {
     Receive_flag = Serial.parseFloat();
     Receive=true;
    
    while(Serial.available() > 0)
    {
      char t=Serial.read();//受信バッファクリア
    }

  }

}

void serial_write()
{
  Serial.print(imu.acc_x);
  Serial.print(imu.acc_y);
  Serial.println(imu.acc_z);
  /*
  Serial.print(imu.yaw*(180/PI));
  Serial.print(imu.roll*(180/PI));
  Serial.println(imu.pitch*(180/PI));
  */
}
