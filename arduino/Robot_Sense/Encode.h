#ifndef Encode_H
#define Encode_H

#include <SPI.h>
#include <Arduino.h>

//--
#define PINION_RADIUS 11.2//[mm]
#define PPR 2048//
#define SS_R 10//右脚
#define SS_L 9//左脚
//--

#define CLR B00000000
#define RD B01000000
#define WR B10000000
#define LOAD B11000000

#define MDR0 B00001000
#define MDR1 B00010000
#define DTR B00011000
#define CNTR B00100000
#define OTR B00101000
#define STR B00110000

// filter factor 1
// async index
// no index
// free-running
// 4x quadrature
#define MDR0_CONF B00000011

// no flag
// enabled
// 32 bits
#define MDR1_CONF B00000000

class Encode
{
  public:
    Encode();
    long count_R;
    long count_L;
    float disp_R;//脚の長さの測定値
    float disp_L;

    void Encode_Init();
    void Encode_count();
    void CallDisp();
};



#endif
