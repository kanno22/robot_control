#ifndef Log
#define Log

#include<fstream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

#define FILE_NAME "./log/log.csv"
#define FILE_NAME_2 "./log/log2.csv"
#define FILE_NAME_S "./log/log_sensor.csv"
#define FILE_NAME_I "./log/log_imu.csv"
#define FILE_NAME_C "./log/log_cog.csv"

#define DATANUM 396//360


using namespace std;

class DataLog
{
    public:
        ofstream mylog;
        ofstream mylog2;
        ofstream mylog_s;
        ofstream mylog_imu;
        ofstream mylog_cog;
        void log_init();
        void log2_init();
        void log_sensor_init();
        void log_imu_init();
        void log_cog_init();

        void logging(RobotLink link[],Robot robot,walkingpatterngenerator gene);
        void logging_2(RobotLink link[],walkingpatterngenerator gene);
        void logging_sensor(double (&link_get_q)[15][DATANUM],double (&link_get_c)[15][DATANUM],int w_count);
        void logging_imu(double (&IMU_acc)[3][DATANUM],double (&IMU_angle)[3][DATANUM],double (&IMU_time)[DATANUM],int w_count);
        void logging_cog(RobotLink link[],Robot robot,double t);
            

};

#endif