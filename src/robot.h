#pragma once

class Timer
{
    public:
        double dt;
        double t;
        int count;

        Timer();
        void countup();
};