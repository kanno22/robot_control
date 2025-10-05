#include "CAN.h"

#define PINION_RADIUS 0.0112//[mm]
#define GEAR_RADIUS 0.008//[mm]

//PC->Arduino
float RlegDisp_ref=0.0;//PCから受信した目標位置
bool Receive=false;

//Arduino->EPOS
int RlegAngle_ref=0;//EPOSへ送信する目標角度
const int EposNodeID_R=1;

CAN epos;

void setup() 
{
  char sbuf[60];
  Serial.begin(115200);

  epos.CAN_init(EposNodeID_R);
/////////////////右足EPOSをPPMモードに設定
  sprintf(sbuf, "SetPPM ID:%d, rtn:%d",EposNodeID_R,  epos.SetPPM(EposNodeID_R) );
  Serial.println(sbuf);
  
  sprintf(sbuf, "IsFault rtn:%d", epos.IsFault(EposNodeID_R) );//0正常
  Serial.println(sbuf);

}

void loop() 
{
  serial_read();

  if(Receive==true)
  { 
    Input();
        
    Receive=false;         
  }
  
}

void Input()//BLDCへの指令値印加
{
  
  RlegAngle_ref=(int)((4096.0/PI)*(RlegDisp_ref/PINION_RADIUS));//8192qc/turn　正：上

  epos.SetTargetPosition(EposNodeID_R,RlegAngle_ref);
  epos.StartPositioningPPM(EposNodeID_R,0,0);
}


void serial_read()
{
  
  if(Serial.available() >0)
  {
     RlegDisp_ref = Serial.parseFloat();

     Receive=true;
    
    while(Serial.available() > 0)
    {
      char t=Serial.read();//受信バッファクリア
    }

  }
}
