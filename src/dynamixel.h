#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include  <stdio.h>

#include  "dx2lib.h"
#include  "dxmisc.h"

#define ID_NUMBER 8
#define OPMODE 4//Expand Position Control
#define XMID1 7//角度を取得したいdynamixelのID
#define XMID2 10//角度を取得したいdynamixelのID

class Dynamixel
{
  private:
    TDeviceID dev;
    bool setop;
    uint8_t ids[ID_NUMBER]={2,3,4,5,7,8,9,10};

  public:
    double angle[ID_NUMBER];//[deg]
    double angle_g[ID_NUMBER];
    double current_g[ID_NUMBER];//[mA]
    double dt;//=0.02;//20ms

    void open();
    void close();
    void set_LEDs(bool enable);
    void set_OperatingModes();
    void set_DriveModes();
    void torque_enables(bool enable);
    void angle_time_writes2();
    void get_angle();
    void get_angles();
    void get_current();
    
};

#endif