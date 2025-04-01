#include"Log.h"

void DataLog::log_init()
{
    mylog.open(FILE_NAME,std::ios::trunc);//上書きモード

    mylog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"Pr_x\t"<<"Pr_y\t"<<"Pr_z\t"<<"Pl_x\t"<<"Pl_y\t"<<"Pl_z\t"<<"BODY_x\t"<<"BODY_y\t"<<"BODY_z\t"<<"BODY_vx\t"<<"BODY_vy\t"<<"BODY_vz\t"<<"BODY_ax\t"<<"BODY_ay\t"<<"BODY_az\t"<<"COG_vx\t"<<"COG_vy\t"<<"COG_vz\t"<<"COG_ax\t"<<"COG_ay\t"<<"COG_az\t"<<endl;

}

void DataLog::logging(RobotLink link[],Robot robot,walkingpatterngenerator gene)
{
    mylog<<gene.t<<"\t"<<robot.CoG(0)<<"\t"<<robot.CoG(1)-DWR<<"\t"<<robot.CoG(2)<<"\t"<<link[6].p(0)<<"\t"<<link[6].p(1)-DWR<<"\t"<<link[6].p(2)<<"\t"<<link[13].p(0)<<"\t"<<link[13].p(1)-DWR<<"\t"<<link[13].p(2)<<"\t"<<link[0].p(0)<<"\t"<<link[0].p(1)-DWR<<"\t"<<link[0].p(2)<<"\t"<<link[0].v(0)<<"\t"<<link[0].v(1)<<"\t"<<link[0].v(2)<<"\t"<<link[0].acc(0)<<"\t"<<link[0].acc(1)<<"\t"<<link[0].acc(2)<<"\t"<<robot.dCoGref(0)<<"\t"<<robot.dCoGref(1)<<"\t"<<robot.dCoGref(2)<<"\t"<<robot.ddCoGref(0)<<"\t"<<robot.ddCoGref(1)<<"\t"<<robot.ddCoGref(2)<<"\t"<<endl;
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

 void DataLog::log_sensor_init()
 {
    mylog_s.open(FILE_NAME_S,std::ios::trunc);//上書きモード
//（変位）左股ロール・ピッチ、左直動、左足首ピッチ・ロール、右股ロール・ピッチ、右直動、右足首ピッチ・ロール、（電流）左股ロール・ピッチ、左足首ピッチ・ロール、右股ロール・ピッチ、右足首ピッチ・ロール
   mylog_s<<"L_C_roll_angle\t"<<"L_C_pitch_angle\t"<<"L_linear_dis\t"<<"L_A_pitch_angle\t"<<"L_A_roll_angle\t"<<"R_C_roll_angle\t"<<"R_C_pitch_angle\t"<<"R_linear_dis\t"<<"R_A_pitch_angle\t"<<"R_A_roll_angle\t"<<"L_C_roll_current\t"<<"L_C_pitch_current\t"<<"L_A_pitch_current\t"<<"L_A_roll_current\t"<<"R_C_roll_current\t"<<"R_C_pitch_current\t"<<"R_A_pitch_current\t"<<"R_A_roll_current\t"<<endl;

 }

 void DataLog::logging_sensor(double (&link_get_q)[15][DATANUM],double (&link_get_c)[15][DATANUM],int w_count)
{
   // mylog2<<gene.t<<"\t"<<link[4].q<<"\t"<<link[11].q<<endl;
    mylog_s<<link_get_q[2][w_count]*(180/M_PI)<<"\t"<<link_get_q[3][w_count]*(180/M_PI)<<"\t"<<link_get_q[4][w_count]<<"\t"<<link_get_q[5][w_count]*(180/M_PI)<<"\t"<<link_get_q[6][w_count]*(180/M_PI)<<"\t"<<link_get_q[9][w_count]*(180/M_PI)<<"\t"<<link_get_q[10][w_count]*(180/M_PI)<<"\t"<<link_get_q[11][w_count]<<"\t"<<link_get_q[12][w_count]*(180/M_PI)<<"\t"<<link_get_q[13][w_count]*(180/M_PI)<<"\t"<<link_get_c[2][w_count]<<"\t"<<link_get_c[3][w_count]<<"\t"<<link_get_c[5][w_count]<<"\t"<<link_get_c[6][w_count]<<"\t"<<link_get_c[9][w_count]<<"\t"<<link_get_c[10][w_count]<<"\t"<<link_get_c[12][w_count]<<"\t"<<link_get_c[13][w_count]<<"\t"<<endl;

}

void DataLog::log_cog_init()
{
    mylog_cog.open(FILE_NAME_C,std::ios::trunc);//上書きモード

    mylog_cog<<"time\t"<<"COG_x\t"<<"COG_y\t"<<"COG_z\t"<<"COG_xx\t"<<"COG_yy\t"<<"COG_zz\t"<<"p_x\t"<<"p_y\t"<<"p_z\t"<<"BODY_x\t"<<"BODY_y\t"<<"BODY_z\t"<<"BODY_vx\t"<<"BODY_vy\t"<<"BODY_vz\t"<<"BODY_ax\t"<<"BODY_ay\t"<<"BODY_az\t"<<"COG_vx\t"<<"COG_vy\t"<<"COG_vz\t"<<"COG_ax\t"<<"COG_ay\t"<<"COG_az\t"<<endl;

}

void DataLog::logging_cog(RobotLink link[],Robot robot,double t)
{
    mylog_cog<<t<<"\t"<<robot.CoG_O_GND[1](0)<<"\t"<<robot.CoG_O_GND[1](1)<<"\t"<<robot.CoG_O_GND[1](2)<<"\t"<<robot.CoG_gnd(0)<<"\t"<<robot.CoG_gnd(1)<<"\t"<<robot.CoG_gnd(2)<<"\t"<<robot.p_O_GND(0)<<"\t"<<robot.p_O_GND(1)<<"\t"<<robot.p_O_GND(2)<<"\t"<<robot.p_body_GND[1](0)<<"\t"<<robot.p_body_GND[1](1)<<"\t"<<robot.p_body_GND[1](2)<<"\t"<<robot.dp_body_GND[1](0)<<"\t"<<robot.dp_body_GND[1](1)<<"\t"<<robot.dp_body_GND[1](2)<<"\t"<<robot.ddp_body_GND(0)<<"\t"<<robot.ddp_body_GND(1)<<"\t"<<robot.ddp_body_GND(2)<<"\t"<<robot.dCoG_O_GND[1](0)<<"\t"<<robot.dCoG_O_GND[1](1)<<"\t"<<robot.dCoG_O_GND[1](2)<<"\t"<<robot.ddCoG_O_GND(0)<<"\t"<<robot.ddCoG_O_GND(1)<<"\t"<<robot.ddCoG_O_GND(2)<<"\t"<<robot.dCoG_O_GND[0](0)<<"\t"<<robot.dCoG_O_GND[0](1)<<"\t"<<robot.dCoG_O_GND[0](2)<<"\t"<<endl;
}

//  void DataLog::log3_init()1
//  {
//     mylog3.open(FILE_NAME_3,std::ios::trunc);//上書きモード

//    // mylog2<<"time\t"<<"Rleg_l\t"<<"Lleg_l\t"<<endl;
//    mylog3<<"L_C_roll\t"<<"L_A_roll\t"<<"L_Linear\t"<<"L_C_roll_c\t"<<"L_A_roll_c\t"<<endl;

//  }

//  void DataLog::logging_3(RobotLink link[])
// {
//    // mylog2<<gene.t<<"\t"<<link[4].q<<"\t"<<link[11].q<<endl;
//     mylog3<<link[9].get_q<<"\t"<<link[13].get_q<<"\t"<<link[11].get_q<<"\t"<<link[9].get_c<<"\t"<<link[13].get_c<<"\t"<<endl;

// }
