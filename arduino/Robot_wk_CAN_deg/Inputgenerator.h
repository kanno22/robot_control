#ifndef Inputgenerator_H
#define Inputgenerator_H

#include <Arduino.h>

class Inputgenerator
{
  private:
    double starttime;
    double estarttime;
    
  public:
    double Rforce_ref;
    double Lforce_ref;
    double Rdisp_ref;
    double Ldisp_ref;
    int Count;
    double Time;
    double Oldtime;//add
    double dt;

    Inputgenerator();
    void Forcegene(bool Start,bool Estart);//
    void dispgene();
    void Counter();
};

#endif
