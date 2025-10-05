#include "robot.h"

///Timer
Timer::Timer()
{
    dt = 0.0;
    t = 0.0;
    count=0;
}

void Timer::countup()
{
    t += dt;
    count++;

}