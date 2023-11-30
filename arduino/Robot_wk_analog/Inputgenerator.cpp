#include"Inputgenerator.h"

Inputgenerator::Inputgenerator()
{
  Rforce_ref=0.0;
  Lforce_ref=0.0;
  Rdisp_ref=0.0;
  Ldisp_ref=0.0;
  starttime=0.0;
  estarttime=0.0;

  Count=0;
  Time=0.0;
  Oldtime=0.0;
  dt=0.01;//10ms;
}

void Inputgenerator::Counter()//カウンター
{
  Count++;
  Time=(double)Count*dt;
}

void Inputgenerator::dispgene()
{
  if(Time<5)
  {
    Rdisp_ref=0.0;
    Ldisp_ref=0.0;
    starttime=Time;
  }
  else if(Time>=starttime&&Time<=(starttime+4))
  {
    Rdisp_ref=0.05*(Time-starttime);//トルク[Nm]
    Ldisp_ref=0.05*(Time-starttime);//トルク[Nm]

  }
  else if(Time>=(starttime+4)&&Time<=(starttime+8))  {
    Rdisp_ref=-0.05*(Time-starttime)+0.2;//トルク[Nm]
    Ldisp_ref=-0.05*(Time-starttime)+0.2;//トルク[Nm]
    
  }
  else
  {
    Rdisp_ref=0.0;//-18
    Ldisp_ref=0.0;
  }
}

void Inputgenerator::Forcegene(bool Start,bool Estart)
{
  if((Start==LOW)&&(Estart==LOW))
  {
    Rforce_ref=0.0;
    Lforce_ref=0.0;
    starttime=Time;
  }
  else if((Start==HIGH)&&(Estart==LOW)&&(Time>=starttime&&Time<=(starttime+2)))
  {
    Rforce_ref=-9*(Time-starttime);//トルク[Nm]
    Lforce_ref=-9*(Time-starttime);//トルク[Nm]

  }
  else if((Start==HIGH)&&(Estart==LOW)&&(Time>(starttime+2)))
  {
    Rforce_ref=-18;//トルク[Nm]0
    Lforce_ref=-18;
    estarttime=Time;
  }
  else if((Start==LOW)&&(Estart==HIGH)&&(Time>=estarttime&&Time<=(estarttime+2)))
  {
    Rforce_ref=23;//18;//トルク[Nm]自重を支えうる力 18N
    Lforce_ref=23;//18;//23 5A
  }
  else
  {
    Rforce_ref=0.0;//-18
    Lforce_ref=0.0;
  }
}
