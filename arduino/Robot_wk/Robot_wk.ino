#include "IMU.h"
#include "Inputgenerator.h"
#include<MsTimer2.h>

#define RLEG_PIN 5
#define LLEG_PIN 6
#define RLEG_ENABLE 4
#define LLEG_ENABLE 7

#define RPINA 2
#define RPINB 8
#define LPINA 3
#define LPINB 9

#define Rinterrupt 0//2番ピン
#define Linterrupt 1//3番ピン

#define PINION_RADIUS 11.2//[mm]
#define GEAR_RADIUS 0.008//[mm]

#define PPR 1024//512*2

int Rleg_disp=0;
int Lleg_disp=0;

int Rleg_input=0;//0~255
int Lleg_input=0;

volatile long Rcount;
volatile long Lcount;

double Rdisp;
double Ldisp;

bool Start;
bool Estart;


const double ulimit=127.5;//255/2

IMU imu;
Inputgenerator inputgenerator;

void setup() 
{
  imu.IMU_Init();
  Init_Encode();
  Init_Input();

  Serial.begin(115200);

  MsTimer2::set(10,Input);
  MsTimer2::start();
  
  attachInterrupt(Rinterrupt, REncode, CHANGE);
  attachInterrupt(Linterrupt, LEncode, CHANGE);
  Enable_Input(HIGH,LOW,LOW);
}

void loop() 
{
  imu.IMU_sense();//ロール・ピッチ・ヨー角の算出
  CallDisp();//右、左足のストロークの算出
  serial_read();

  
}

void Init_Input()
{
  pinMode(RLEG_ENABLE, OUTPUT);
  pinMode(LLEG_ENABLE, OUTPUT);
  //pinMode(RLEG_PIN, OUTPUT);
  //pinMode(LLEG_PIN, OUTPUT);

  digitalWrite(RLEG_ENABLE, LOW);
  digitalWrite(LLEG_ENABLE, LOW);
}

void Enable_Input(bool i,bool j,bool k)
{
  digitalWrite(RLEG_ENABLE, i);
  digitalWrite(LLEG_ENABLE, i);

  Start=j;
  Estart=k;
}

void Input()//BLDCへの指令値印加
{

//  inputgenerator.Forcegene(Start,Estart);
//  Rleg_input=(int)(127.5+limit(538.206*GEAR_RADIUS*inputgenerator.Rforce_ref));//a=1/Kt(2.568/51)
//  Lleg_input=(int)(127.5+limit(538.206*GEAR_RADIUS*inputgenerator.Lforce_ref));

  Rleg_input=(int)(127.5+limit((51/19.2)*(24/PI)*(Rleg_disp/80)));//1qc=7.5degLleg_disp/80
  Lleg_input=(int)(127.5+limit((51/19.2)*(24/PI)*(Lleg_disp/80)));//1qc=7.5degLleg_disp/80
//
//inputgenerator.dispgene();
//  Rleg_input=(int)(127.5+limit((51/19.2)*(24/PI)*(inputgenerator.Rdisp_ref*10000/80)));//1qc=7.5degLleg_disp/80
//  Lleg_input=(int)(127.5+limit((51/19.2)*(24/PI)*(inputgenerator.Ldisp_ref/80)));//1qc=7.5degLleg_disp/80



  analogWrite(RLEG_PIN,Rleg_input);
  analogWrite(LLEG_PIN,Lleg_input);

  inputgenerator.Counter();

}

double limit(double u)
{
  if (abs(u) > ulimit) {
    if (u > 0) {
      return ulimit;
    } else {
      return -ulimit;
    }
  } else {
    return u;
  }
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

void CallDisp()
{
  Rdisp=((double)Rcount/PPR)*2*PI*PINION_RADIUS;
  Ldisp=((double)Lcount/PPR)*2*PI*PINION_RADIUS;
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
  
  if((Serial.available() >= sizeof('R') + sizeof(int)))
  {
     //ヘッダの確認
    if ( Serial.read() == 'R' )
    {
      int high = Serial.read(); // 上位バイトの読み取り
      int low = Serial.read(); // 下位バイトの読み取り
      Rleg_disp = (high<<8)|low; // 上位バイトと下位バイトを合体させてint型データを復元
    }

//      if(Rleg_disp==100)
//      {
//        Serial.println("OK1");
//      
//      }
//      else
//      {
//        Serial.println("異常1");
//      }
  }


  if((Serial.available() >= sizeof('L') + sizeof(int)))
  {
     //ヘッダの確認
    if ( Serial.read() == 'L' )
    {
      int high = Serial.read(); // 上位バイトの読み取り
      int low = Serial.read(); // 下位バイトの読み取り
      Lleg_disp = (high<<8)|low; // 上位バイトと下位バイトを合体させてint型データを復元
    }

//      if(Lleg_disp==100)
//      {
//        Serial.println("OK2");
//      
//      }
//      else
//      {
//        Serial.println("異常2");
//      }
  }
}

void serial_w()
{
  /*
  Serial.print(imu.yaw*(180/PI));
  Serial.print(imu.roll*(180/PI));
  Serial.println(imu.pitch*(180/PI));
  */
  Serial.print(((double)Lcount/PPR)*360*1.4);
  Serial.print(" ");
    Serial.println(Lleg_input);
//    Serial.print(inputgenerator.Rforce_ref);
//  Serial.print(" ");
//  Serial.println(inputgenerator.Lforce_ref);//Rleg_input
}

void serial_test()
{
    if(Serial.available()>0)
  {
    Enable_Input(LOW,LOW,LOW);
    //int rd=Serial.read();
    byte rd=Serial.read();

    switch(rd)
    {
      case '1':
        Enable_Input(HIGH,LOW,LOW);
        rd=Serial.read();//シリアルの改行を消化
        break;
          
      case '2':
        Enable_Input(HIGH,HIGH,LOW);
        rd=Serial.read();//シリアルの改行を消化
        break;
        
      case '3':
        Enable_Input(HIGH,LOW,HIGH);
        rd=Serial.read();//シリアルの改行を消化
        break;      
    }
  }
}
