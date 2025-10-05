#ifndef Log
#define Log

#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>

#include<fstream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

#define FILE_NAME "log.csv"
#define FILE_NAME_2 "log2.csv"
#define FILE_NAME_S "log_sensor.csv"
#define FILE_NAME_I "log_imu.csv"
#define FILE_NAME_C "log_cog.csv"

#define DATANUM 162//189//162//216//324//396//360


class DataLog
{
    public:
        std::ofstream mylog;
        std::ofstream mylog2;
        std::ofstream mylog_s;
        std::ofstream mylog_imu;
        std::ofstream mylog_cog;

        std::filesystem::path log_dir;
        std::filesystem::path new_dir_path ;

        void log_init();
        void log2_init();
        void log_sensor_init();
        void log_imu_init();
        void log_cog_init();

        void create_dir();

        void logging(RobotLink link[],RobotLink foot_ref[],Robot robot,double t);
        void logging_2(RobotLink link[],double t);
        void logging_sensor(double (&link_get_q)[15][DATANUM],double (&link_get_c)[15][DATANUM],int w_count);
        void logging_imu(double (&IMU_acc)[3][DATANUM],double (&IMU_angle)[3][DATANUM],double (&IMU_time)[DATANUM],int w_count);
        void logging_cog(RobotLink link[],Robot robot,double t);
            

};

#endif