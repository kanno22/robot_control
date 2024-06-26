#include "IMU.h"
#include "CAN.h"
#include "Inputgenerator.h"

#define RPINA 2
#define RPINB 8
#define LPINA 3
#define LPINB 9

#define Rinterrupt 0//2番ピン
#define Linterrupt 1//3番ピン

#define PINION_RADIUS 11.2//[mm]
#define GEAR_RADIUS 0.008//[mm]

#define PPR 1024//512*2

//シリアル
float RlegDisp_ref=0.0;//PCから受信した目標位置
float LlegDisp_ref=0.0;
float RlegDisp_ref_kari=0.0;
float LlegDisp_ref_kari=0.0;


bool Receive=false;

//EPOS
int RlegAngle_ref=0;//EPOSへ送信する目標角度
int LlegAngle_ref=0;
const int EposNodeID_R=1;
const int EposNodeID_L=2;

//エンコーダ
volatile long Rcount;
volatile long Lcount;

float Rdisp;//脚の長さの測定値
float Ldisp;

IMU imu;
CAN epos;
//Inputgenerator inputgenerator;

//時間計測
unsigned long starttime,endtime;

void setup() 
{
  char sbuf[60];
  Serial.begin(115200);
  
 // imu.IMU_Init();
 // Init_Encode();
  epos.CAN_init(EposNodeID_R,EposNodeID_L);
/////////////////右足EPOSをPPMモードに設定
  sprintf(sbuf, "SetPPM ID:%d, rtn:%d",EposNodeID_R,  epos.SetPPM(EposNodeID_R) );
  Serial.println(sbuf);
  
  sprintf(sbuf, "IsFault rtn:%d", epos.IsFault(EposNodeID_R) );//0正常
  Serial.println(sbuf);
/////////////////左足EPOSをPPMモードに設定
 /* sprintf(sbuf, "SetPPM ID:%d, rtn:%d",EposNodeID_L,  epos.SetPPM(EposNodeID_L) );
  Serial.println(sbuf);
  
  sprintf(sbuf, "IsFault rtn:%d", epos.IsFault(EposNodeID_L) );
  Serial.println(sbuf);*/
/////////////////  
 // attachInterrupt(Rinterrupt, REncode, CHANGE);
  attachInterrupt(Linterrupt, LEncode, CHANGE);

////////////////
  pinMode(7,OUTPUT);
////////////////

}

void loop() 
{
  //imu.IMU_sense();//ロール・ピッチ・ヨー角の算出
  //CallDisp();//右、左足のストロークの算出
  serial_read();

  if(Receive==true)
  { 
    
    Input();
    serial_write();
    Receive=false;
         
  }
  
}

void Input()//BLDCへの指令値印加
{

  RlegAngle_ref=(int)((24/PI)*(RlegDisp_ref/GEAR_RADIUS));//1qc=7.5deg　正：上
  //LlegAngle_ref=(int)(-1*(24/PI)*(LlegDisp_ref/GEAR_RADIUS));//正：下->マイナスを×

  //
 //  Serial.println("RlegAngle_ref=%d",RlegAngle_ref);
  // Serial.println("LlegAngle_ref=%d",LlegAngle_ref);
  //

  epos.SetTargetPosition(EposNodeID_R,RlegAngle_ref);
  epos.StartPositioningPPM(EposNodeID_R,0,0);

  //epos.SetTargetPosition(EposNodeID_L,LlegAngle_ref);
  //epos.StartPositioningPPM(EposNodeID_L,0,0);
  //inputgenerator.Counter();
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
     RlegDisp_ref = Serial.parseFloat();
   //  LlegDisp_ref  = Serial.parseFloat();

     Receive=true;
    
    while(Serial.available() > 0)
    {
      char t=Serial.read();//受信バッファクリア
    }

  }

}

void serial_write()
{
  Serial.println(Lcount);
  /*
  Serial.print(imu.yaw*(180/PI));
  Serial.print(imu.roll*(180/PI));
  Serial.println(imu.pitch*(180/PI));
  */
}
