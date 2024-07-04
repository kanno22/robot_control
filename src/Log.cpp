#include"Log.h"

void DataLog::log_init()
{
    mylog.open(FILE_NAME,std::ios::trunc);//上書きモード

    mylog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"Pr_x\t"<<"Pr_y\t"<<"Pr_z\t"<<"Pl_x\t"<<"Pl_y\t"<<"Pl_z\t"<<"COG_Vx\t"<<"COG_Vy\t"<<endl;

}

void DataLog::logging(RobotLink link[],walkingpatterngenerator gene)
{
    mylog<<gene.t<<"\t"<<link[0].p(0)<<"\t"<<link[0].p(1)<<"\t"<<link[0].p(2)<<"\t"<<link[6].p(0)<<"\t"<<link[6].p(1)<<"\t"<<link[6].p(2)<<"\t"<<link[13].p(0)<<"\t"<<link[13].p(1)<<"\t"<<link[13].p(2)<<"\t"<<link[0].v(0)<<"\t"<<link[0].v(1)<<endl;
}

 void DataLog::log2_init()
 {
    mylog2.open(FILE_NAME_2,std::ios::trunc);//上書きモード

   // mylog2<<"time\t"<<"Rleg_l\t"<<"Lleg_l\t"<<endl;
   mylog2<<"time\t"<<"R_C_yaw\t"<<"R_C_roll\t"<<"R_C_pitch\t"<<"R_Linear\t"<<"R_A_pitch\t"<<"R_A_roll\t"<<"L_C_yaw\t"<<"L_C_roll\t"<<"L_C_pitch\t"<<"L_Linear\t"<<"L_A_pitch\t"<<"L_A_roll\t"<<endl;

 }

 void DataLog::logging_2(RobotLink link[],walkingpatterngenerator gene)
{
   // mylog2<<gene.t<<"\t"<<link[4].q<<"\t"<<link[11].q<<endl;
    mylog2<<gene.t<<"\t"<<link[1].q*(180/M_PI)<<"\t"<<link[2].q*(180/M_PI)<<"\t"<<link[3].q*(180/M_PI)<<"\t"<<link[4].q<<"\t"<<link[5].q*(180/M_PI)<<"\t"<<link[6].q*(180/M_PI)<<"\t"<<link[8].q*(180/M_PI)<<"\t"<<link[9].q*(180/M_PI)<<"\t"<<link[10].q*(180/M_PI)<<"\t"<<link[11].q<<"\t"<<link[12].q*(180/M_PI)<<"\t"<<link[13].q*(180/M_PI)<<"\t"<<endl;

}

 void DataLog::log3_init()
 {
    mylog3.open(FILE_NAME_3,std::ios::trunc);//上書きモード

   // mylog2<<"time\t"<<"Rleg_l\t"<<"Lleg_l\t"<<endl;
   mylog3<<"L_C_roll\t"<<"L_A_roll\t"<<"L_Linear\t"<<"L_C_roll_c\t"<<"L_A_roll_c\t"<<endl;

 }

 void DataLog::logging_3(RobotLink link[])
{
   // mylog2<<gene.t<<"\t"<<link[4].q<<"\t"<<link[11].q<<endl;
    mylog3<<link[9].get_q<<"\t"<<link[13].get_q<<"\t"<<link[11].get_q<<"\t"<<link[9].get_c<<"\t"<<link[13].get_c<<"\t"<<endl;

}
