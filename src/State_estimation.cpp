#include"State_estimation.h"
#include<math.h>

using namespace Eigen;
using namespace std;

State_estimation::State_estimation()
{
        stepcount=0;
        tcount=0;
        sup_dub=true;//true:片足支持、false:両足支持
        switching=0;//0:定常、1:右->左へ切り替えした瞬間、2:左->右へ切り替えした瞬間
        dt=0.01*DTNUM;
        t=0.0;
}

//各リンクのエンコーダ値（+IMU）の値が更新され次第実行（つまり毎ループ）
void State_estimation::Centroid_estimation(RobotLink link[],Robot &robot)
{
    Vector3d M_cog_self={0.0,0.0,0.0};
    Vector3d dp={0.0,-DWR,0.0};
    Vector3d p_rl={0.0,0.0,0.0};
    int tofrom=6;

    //1.自己座標系から見た時の腰リンク姿勢を求める
    link[0].R_self<<1.0,0.0,0.0,
                     0.0,1.0,0.0,
                     0.0,0.0,1.0;     //とりあえず単位行列とする　後にIMUからの姿勢を反映
    link[0].p_self={0.0,0.0,0.0}; 

    //2.自己座標系から見た時の重心位置を推定
    
    ForwardKinematics_self(link, tofrom, link[1].ID);
    ForwardKinematics_self(link, tofrom, link[8].ID);

    for(int i=0; i<LINKNUM; i++)//各リンクの絶対重心位置を算出
    {
        link[i].c_self=link[i].p_self+link[i].R_self*link[i].c_;
        M_cog_self+=link[i].m*link[i].c_self;
    }
    robot.CoG_self=M_cog_self/robot.M; //重心位置算出

    //3.自己座標系から見た時の地面座標系の位置と姿勢
    if(sign()==1)//右足が支持脚
    {
        robot.p_s_gnd=link[6].p_self;//+dp;
        robot.R_s_gnd=link[6].R_self;//IMUの値を無視している間は足リンク姿勢=地面姿勢とする（自己座標系は”斜める”ものとする

    }
    else//左足が支持脚
    {
        robot.p_s_gnd=link[13].p_self;
        robot.R_s_gnd=link[13].R_self;
    }

    //4.地面座標系から見た時の自己座標系の位置と姿勢
    robot.R_g_self=robot.R_s_gnd.inverse();
    robot.p_g_self=-robot.R_g_self*robot.p_s_gnd;
    
    //5.地面座標系から見た時の重心位置の推定
    robot.CoG_gnd=robot.p_g_self+robot.R_g_self*robot.CoG_self;
    //6.歩行開始時からの重心・ボディ位置の推定

    if((stepcount==0)&&(tcount==0)&&(sup_dub==true))//t=0
    {
     robot.CoG_O_GND[0]=robot.p_O_GND+robot.CoG_gnd;
     robot.dCoG_O_GND[0]={0.0,0.0,0.0};
     robot.p_body_GND[0]=robot.p_O_GND+robot.p_g_self;
     robot.dp_body_GND[0]={0.0,0.0,0.0};
     //cout<<"状態推定_第0回目"<<endl;
    }

     robot.CoG_O_GND[1]=robot.p_O_GND+robot.CoG_gnd;
     robot.dCoG_O_GND[1]=(robot.CoG_O_GND[1]-robot.CoG_O_GND[0])/dt;//速度
     robot.ddCoG_O_GND=(robot.dCoG_O_GND[1]-robot.dCoG_O_GND[0])/dt;//加速度

     robot.p_body_GND[1]=robot.p_O_GND+robot.p_g_self;//ボディ位置
     robot.dp_body_GND[1]=(robot.p_body_GND[1]-robot.p_body_GND[0])/dt;
     robot.ddp_body_GND=(robot.dp_body_GND[1]-robot.dp_body_GND[0])/dt;
    
     robot.CoG_O_GND[0]=robot.CoG_O_GND[1];
     robot.dCoG_O_GND[0]=robot.dCoG_O_GND[1];
     robot.p_body_GND[0]=robot.p_body_GND[1];
     robot.dp_body_GND[0]=robot.dp_body_GND[1];
     
    
    if(switching==1)//右->左へ切り替え
    {
        p_rl=link[13].p_self-robot.p_s_gnd;
        robot.p_O_GND+=robot.R_g_self*p_rl;
    }
    else if(switching==2)//右->左へ切り替え
    {
        p_rl=link[6].p_self-robot.p_s_gnd;
        robot.p_O_GND+=robot.R_g_self*p_rl;
    }

   
}

//自己座標系における順運動学計算
void State_estimation::ForwardKinematics_self(RobotLink link[], int tofrom, int start)
{
    //i=0はルートリンク
    for (int i = 0; i < tofrom; i++) //int i = start; i < start+tofrom; i++
    {
        RobotLink L;
        L = link[start+i];

        if(i==3)//直動関節
        {
            link[start+i].p_self = link[link[start+i].parentID].R_self * (link[start+i].b+link[start+i].a*link[start+i].get_q) + link[link[start+i].parentID].p_self;

            link[start+i].R_self = link[link[start+i].parentID].R_self;
        }
        else//回転関節
        {
            link[start+i].p_self = link[link[start+i].parentID].R_self * link[start+i].b + link[link[start+i].parentID].p_self;

            link[start+i].R_self = link[link[start+i].parentID].R_self * Rodrigues_self(L);
        }
    }
}

Matrix3d State_estimation::Rodrigues_self(RobotLink link)
{
    Matrix3d A, E, R;

    A << 0.0, -link.a(2), link.a(1), link.a(2), 0.0, -link.a(0), -link.a(1),
        link.a(0), 0.0;  //ひずみ対称行列

    E << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;  //単位行列

    R = E + A * sin(link.get_q) + A * A * (1 - cos(link.get_q));  //ロドリゲスの式

    return R;
}

int State_estimation::sign()
{
        if(((stepcount % 2)==0)||stepcount==0)//右足が始め
        {
            return 1;//右足支持なら1
        }
        else
        {
            return -1;//左足支持なら-1
        }
};
