#ifndef Log
#define Log

#include<fstream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

#define FILE_NAME "./log/log.csv"
#define FILE_NAME_2 "./log/log2.csv"


using namespace std;

class DataLog
{
    public:
        ofstream mylog;
        ofstream mylog2;
        void log_init();
        void log2_init();
        void logging(RobotLink link[],walkingpatterngenerator gene);
        void logging_2(RobotLink link[],walkingpatterngenerator gene);

};

#endif