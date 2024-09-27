#ifndef Log
#define Log

#include<fstream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

#define FILE_NAME "./log/log.csv"
#define FILE_NAME_2 "./log/log2.csv"
#define FILE_NAME_3 "./log/log3.csv"


using namespace std;

class DataLog
{
    public:
        ofstream mylog;
        ofstream mylog2;
        ofstream mylog3;
        void log_init();
        void log2_init();
        void log3_init();
        void logging(RobotLink link[],Robot robot,walkingpatterngenerator gene);
        void logging_2(RobotLink link[],walkingpatterngenerator gene);
        void logging_3(RobotLink link[]);
        

};

#endif