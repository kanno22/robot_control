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