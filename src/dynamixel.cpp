#include "dynamixel.h"

void Dynamixel::open()
{
  dev=DX2_OpenPort(_COMPORT, _BAUDRATE);

  if(dev != 0)
  {
    printf ("Successful opening of %s\n", _COMPORT);
    for (int i = 0; i < ID_NUMBER; i++) DXL_GetModelInfo (dev, ids[i]);
  }
  else
  {
    printf ("Failed to open %s\n", _COMPORT);
  }
}

void Dynamixel::close()
{
  if(dev != 0)
  {
    DX2_ClosePort (dev);
  }
}

void Dynamixel::set_LEDs(bool enable)
{
  if(dev != 0)
  {
    for(int i=0;i<ID_NUMBER;i++)
    {
      DXL_SetLED(dev,ids[i],enable);
    }
  }

}

void Dynamixel::set_OperatingModes()
{
  if(dev != 0)
  {
    setop=DXL_SetOperatingModesEquival(dev,ids,ID_NUMBER,OPMODE); 
  }
}

void Dynamixel::set_DriveModes()
{
  if(dev != 0)
  {
    DXL_SetDriveModesEquival(dev,ids,ID_NUMBER,OPMODE); 
  }

}


void Dynamixel::torque_enables(bool enable)
{
  if(dev != 0)
  {
    if(setop != 0)
    {
      DXL_SetTorqueEnablesEquival(dev,ids,ID_NUMBER,enable);
    }
  }

}

void Dynamixel::angle_time_writes2()
{
  if(dev != 0)
  {
    if(setop != 0)
    {
      DXL_SetGoalAnglesAndTime2(dev,ids,angle,ID_NUMBER,dt);
    }
  }

}

void Dynamixel::get_angles()
{
  if(dev != 0)
  {
    if(setop != 0)
    {
      DXL_GetPresentAngles(dev,ids,angle_g,ID_NUMBER);
    }
  }

}


void Dynamixel::get_angle()
{
  if(dev != 0)
  {
    if(setop != 0)
    {
      DXL_GetPresentAngle(dev,XMID1,&angle_g[4]);//左股ロール
      DXL_GetPresentAngle(dev,XMID2,&angle_g[7]);//左股ロール
    }
  }

}

void Dynamixel::get_current()
{
  if(dev != 0)
  {
    if(setop != 0)
    {
      DXL_GetPresentCurrent(dev,XMID1,&current_g[4]);//左股ロール
      DXL_GetPresentCurrent(dev,XMID2,&current_g[7]);//左股ロール
    }
  }

}
