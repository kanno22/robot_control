#ifndef Log
#define Log

#include<fstream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

#define FILE_NAME "./log/log.csv"

using namespace std;

class DataLog
{
    public:
        ofstream mylog;
        void log_init();
        void logging(RobotLink link[],walkingpatterngenerator gene);
};

#endif