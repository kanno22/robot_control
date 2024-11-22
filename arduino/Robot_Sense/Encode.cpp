#include"Encode.h"
//白:SS 10
//灰色:SCLK 13
//紫:MISO 12
//青:MOSI 11

Encode::Encode()
{
  count_R=0;
  count_L=0;
  disp_R=0.0;//脚の長さの測定値
  disp_L=0.0;
}

void Encode::Encode_Init() 
{
  SPI.begin();
  //右脚
  pinMode(SS_R, OUTPUT);
  
  digitalWrite(SS_R, LOW);
  SPI.transfer(WR | MDR0);
  SPI.transfer(MDR0_CONF);
  digitalWrite(SS_R, HIGH);
  
  digitalWrite(SS_R, LOW);
  SPI.transfer(WR | MDR1);
  SPI.transfer(MDR1_CONF);
  digitalWrite(SS_R, HIGH);
  
  digitalWrite(SS_R, LOW);
  SPI.transfer(CLR | CNTR);
  digitalWrite(SS_R, HIGH);

  //左脚
  pinMode(SS_L, OUTPUT);
  
  digitalWrite(SS_L, LOW);
  SPI.transfer(WR | MDR0);
  SPI.transfer(MDR0_CONF);
  digitalWrite(SS_L, HIGH);
  
  digitalWrite(SS_L, LOW);
  SPI.transfer(WR | MDR1);
  SPI.transfer(MDR1_CONF);
  digitalWrite(SS_L, HIGH);
  
  digitalWrite(SS_L, LOW);
  SPI.transfer(CLR | CNTR);
  digitalWrite(SS_L, HIGH);

}

void Encode::Encode_count() 
{
  //右
  digitalWrite(SS_R, LOW);
  byte b_r = SPI.transfer((byte) RD | CNTR);
  count_R = SPI.transfer(0x00);
  count_R <<= 8;
  count_R |= SPI.transfer(0x00);
  count_R <<= 8;
  count_R |= SPI.transfer(0x00);
  count_R <<= 8;
  count_R |= SPI.transfer(0x00);
  digitalWrite(SS_R, HIGH);

  //左
  digitalWrite(SS_L, LOW);
  byte b_l = SPI.transfer((byte) RD | CNTR);
  count_L = SPI.transfer(0x00);
  count_L <<= 8;
  count_L |= SPI.transfer(0x00);
  count_L <<= 8;
  count_L |= SPI.transfer(0x00);
  count_L <<= 8;
  count_L |= SPI.transfer(0x00);
  digitalWrite(SS_L, HIGH);

}

void Encode::CallDisp()
{
  disp_R=((float)count_R/PPR)*2*PI*PINION_RADIUS;
  disp_L=((float)count_L/PPR)*2*PI*PINION_RADIUS;
}
