
#include "Kinematics.h"
#include "State_estimation.h"

#include "Serial.h"
#include "RSservo.h"
#include "dynamixel.h"
#include"Log.h"

/// timer
#include <stdio.h>
#include <stdlib.h>//exit()
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
///timer
#define MESSAGE "timer_handler() is called.\n"
#define INIT_WAIT_SEC 2
#define INTERVAL_SEC 0
#define INTERVAL_MICROSEC 10000*DTNUM//*1.5//*2//10ms=10000μs
#define NR_TIMER_INTERRUPTS 360//468//360//432//360//900//720//900//450//600//900//1625///2//625/2//1100/2//950/2//840/2//560//繰り返し回数 0.8*7 

static int remaining = NR_TIMER_INTERRUPTS;
struct sigaction action, old_action;
struct itimerval timer, old_timer;
///

#define START 0.00081//0.00079//
#define WX 0.0//0.015//0.033
#define WY 0.02
#define DWY 0.045//0.0465//両足支持期の増分
#define TSUP 0.5
#define TDBL 0.5//0.3

#define TIMER
#define TEST
#define KINETEST
#define ONE_LEG
#define MOVE_TEST
#define INITTIME 1000000//1s

#define PPR 2048
#define PINION_RADIUS 0.0112//[m]


///
#define RSNUM 2

using namespace Eigen;
using namespace std;

void LinkInit(RobotLink LINK[], int linknum);
void RSInit(Servo_data RS405CB[]);
void WPInit(walkingparameters wp[]);
void RSInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial);
void RSInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count);
void ECmotorInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],serial& Arduino,serial& Arduino2,int w_count);
void XMInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS]);
void XMInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],int w_count);

void Flag_send();
void Data_sense(int w_count);//センサ値取得
void State_estimate();

void timer_init();
void timer_close();
void timer_handler(int signam);//タイマに呼び出される関数

double calcTime();//時間計測

    //////////////////////////////////
    RobotLink LINK[15];//各リンクの情報
    Robot robot;//ロボットの質量、重心の情報
    RobotLink linkref[2];
    double link_q[15][NR_TIMER_INTERRUPTS];//目標関節変位を格納する配列
    double link_get_q[15][NR_TIMER_INTERRUPTS];//取得した関節変位を格納する配列
    double link_get_c[15][NR_TIMER_INTERRUPTS];//取得した電流を格納する配列
    string linear_get_q[NR_TIMER_INTERRUPTS];
    string ddx[NR_TIMER_INTERRUPTS];//取得した生の加速度を格納する配列
    string ddy[NR_TIMER_INTERRUPTS];//取得した生の加速度を格納する配列
    string ddz[NR_TIMER_INTERRUPTS];//取得した生の加速度を格納する配列
    vector<float> arduino_sense_data;//arduino nanoから送られてきたIMU、エンコーダの値を格納する配列

    Kinematics kine;
    State_estimation state;
    walkingparameters wp[18];//numpsteps
    walkingpatterngenerator gene;

    serial Arduino,Arduino2,Arduino_s;
    Servo_data RS405CB[RSNUM];
    Servo_serial RS_serial;
    Dynamixel XM_serial;

    DataLog datalog;

    int linknum=15;//13
    int tofrom=6;
    int numsteps=9;//17
    int w_count=0;
    //
    int starttime;
    int endtime;
    int t;
    int oldt;
    int dt;
    //目標ボディ速度・加速度算出用
    Vector3d p_body_old;
    Vector3d dp_body_old;
    Vector3d ddp_body_old;
    //////////////////////////////////

int main()
{
    LinkInit(LINK,linknum);

#ifndef KINETEST
    kine.CalcMass(LINK,robot);
    cout<<"Mass="<<robot.M<<endl;

  //  LINK[0].p={0.0,(WY+DWY)/2,ZC};//{0.0,0.02,0.385};
    LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    linkref[0].p={0.0,DWR,0.0};//右足{0.0,0.0,0.0}
    linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    linkref[1].p={0.0,WY+DWY+DWL,0.01};//右足{0.0,0.196,0.0}
    linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  // kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
   // kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);

    robot.CoGref={0.0,0.0325,ZC-DZC};
    LINK[0].p=robot.CoGref;
    cout<<"CoGref="<<robot.CoGref<<endl;

    kine.ModiCoG(LINK,robot,linkref,tofrom);

    cout<<"CoG="<<robot.CoG<<endl;
    cout<<"BODY="<<LINK[0].p<<endl;

    for(int i=0; i<LINKNUM; i++)//
    {
        LINK[i].get_q=LINK[i].q;
    }
    cout<<"test="<<LINK[6].p-LINK[0].p<<endl;
    cout<<"P_="<<LINK[6].p<<endl;
    state.Centroid_estimation(LINK,robot);
    cout<<"P_GND="<<LINK[6].p_self<<endl;
    cout<<"C_self="<<robot.CoG_self<<endl;
    cout<<"P_GND_test="<<robot.p_s_gnd<<endl;
    cout<<"CoG_gnd="<<robot.CoG_gnd<<endl;
    cout<<"p_g_self="<<robot.p_g_self<<endl;
#else

    //RSInit(RS405CB);

    datalog.log_init();
    datalog.log2_init();
    datalog.log_sensor_init();
    datalog.log_cog_init();
    //////////////////////////////////
    
    Arduino.s_open(serial::kB115200,SERIAL_PORT);
    Arduino2.s_open(serial::kB115200,SERIAL_PORT_2);
    Arduino_s.s_open(serial::kB115200,SERIAL_PORT_S);
    //RS_serial.rs_open();
    XM_serial.open();

    /*
    for(int i=0;i<RSNUM;i++)
    {
        RS405CB[i].tq_mode=1;
        RS_serial.rs_torque(RS405CB[i]);
        cout<<"RS_torque on"<<endl;
    }*/

    XM_serial.set_OperatingModes();
    XM_serial.set_DriveModes();
    XM_serial.set_LEDs(true);
    XM_serial.torque_enables(true);
    cout<<"XM_torque on"<<endl;

#ifndef TEST
    link_q[4][0]=0.0;
    link_q[11][0]=0.0;
    link_q[1][0]=0*(M_PI/1800);
    link_q[7][0]=-0*(M_PI/1800);
    link_q[2][0]=10*(M_PI/180);//右股ロール
    link_q[3][0]=0*(M_PI/180);//右股ピッチ
    link_q[5][0]=0*(M_PI/180);//右足首ピッチ
    link_q[6][0]=10*(M_PI/180);//右足首ロール
    link_q[9][0]=10*(M_PI/180);//左股ロール
    link_q[10][0]=0*(M_PI/180);//左股ピッチ
    link_q[12][0]=0*(M_PI/180);//右足首ピッチ
    link_q[13][0]=10*(M_PI/180);//右足首ロール
    ECmotorInput(link_q,Arduino,Arduino2,0);
    //RSInput_init(link_q,RS405CB,RS_serial);
    XMInput_init(link_q);
    usleep(INITTIME);
    link_q[4][0]=0.0;
    link_q[11][0]=0.0;
    link_q[1][0]=0*(M_PI/1800);
    link_q[7][0]=0*(M_PI/1800);
    link_q[2][0]=-10*(M_PI/180);//右股ロール
    link_q[3][0]=0*(M_PI/180);//右股ピッチ
    link_q[5][0]=0*(M_PI/180);//右足首ピッチ
    link_q[6][0]=-10*(M_PI/180);//右足首ロール
    link_q[9][0]=-10*(M_PI/180);//左股ロール
    link_q[10][0]=0*(M_PI/180);//左股ピッチ
    link_q[12][0]=0*(M_PI/180);//右足首ピッチ
    link_q[13][0]=-10*(M_PI/180);//右足首ロール
    ECmotorInput(link_q,Arduino,Arduino2,0);
    //RSInput_init(link_q,RS405CB,RS_serial);
    XMInput_init(link_q);
    usleep(INITTIME );
#else
       //////////////////////////////////
   // LINK[0].p={0.0,(WY+DWY)/2,ZC};//{0.0,0.02,0.385};
    LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    linkref[0].p={0.0,DWR,0.0};//右足{0.0,0.0,0.0}
    linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    linkref[1].p={0.0,WY+DWY+DWL,0.0};//右足{0.0,0.196,0.0}
    linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    robot.CoGref={0.0,(WY+DWY)/2,ZC-DZC};
    LINK[0].p=robot.CoGref;
    kine.ModiCoG(LINK,robot,linkref,tofrom);

 //   kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
 //   kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);
    
    for(int j=0;j<15;j++)
    {
        link_q[j][w_count]=LINK[j].q;
    } 
    w_count=0;//初期姿勢
    cout<<"直立姿勢"<<endl;  
    //RSInput_init(link_q,RS405CB,RS_serial);
    ECmotorInput(link_q,Arduino,Arduino2,w_count);
    XMInput_init(link_q);
    usleep( INITTIME );
    //////////////////////////////////
      
    #ifndef MOVE_TEST

        for(int i=0;i<160;i++)//3s
        {
            LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

            if(i<80)
            {
                linkref[0].p={0.0,0.0,0.0};//右足{0.0,0.0,0.0}
                linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
                linkref[1].p={0.0,0.1,0.0};//左足{0.0,0.196,0.0}
                linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

               // robot.CoGref={0.0,0.05,0.3};//左足へ
                robot.CoGref={0.0,((0.1-((0.1)/2.0))/2.0)*gene.t+((0.1)/2.0),0.3};//右足へ
                //robot.CoGref={0.0,((Right-((WY+DWY)/2))/2.0)*gene.t+((WY+DWY)/2),ZC-DZC+0.1};//左足へ
                            }
            else
            {              
                linkref[0].p={0.0,0.0,0.0};//右足{0.0,0.0,0.0}
                linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
                
                linkref[1].p={0.0,0.1,0.0};//左足{0.0,0.196,0.0}
                linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

                robot.CoGref={0.0,0.1,0.3};//左足へ
            }

            LINK[0].p=robot.CoGref;
            //kine.ModiCoG(LINK,robot,linkref,tofrom);
            kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
            kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);
            cout<<"BODY="<<LINK[0].p<<endl;
            cout<<"COG="<<robot.CoGref<<endl;
            cout<<"i="<<i<<endl;
            cout<<"gene.t="<<gene.t<<endl;
            cout<<"----"<<endl;

            gene.t=gene.t+0.025;

            datalog.logging(LINK,robot,gene);
            datalog.logging_2(LINK,gene);
            for(int j=0;j<15;j++)
            {
                link_q[j][i]=LINK[j].q;
            }
            
        }
        gene.t=0.0;
        cout<<"動作テスト開始"<<endl;  
        for(int i=0;i<160;i++)
        {   
            //RSInput(link_q,RS405CB,RS_serial,w_count);
            ECmotorInput(link_q,Arduino,Arduino2,w_count);
            XMInput(link_q,w_count);
            
            w_count++;    
            usleep( 25000 );//10000000　
        }  
        usleep( 50000000 );//25s
    #else

    for(int i=0;i<80;i++)//2s 80
    {
      //  LINK[0].p={0.0,((START-((WY+DWY)/2))/2.0)*gene.t+((WY+DWY)/2),ZC};//{0.0,0.02,0.385};
        LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
      //  kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
      //  kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);
        robot.CoGref={0.0,((START-((WY+DWY)/2))/2.0)*gene.t+((WY+DWY)/2),ZC-DZC};
        LINK[0].p=robot.CoGref;
        kine.ModiCoG(LINK,robot,linkref,tofrom);

        gene.t=gene.t+0.01*DTNUM;
        for(int j=0;j<15;j++)
        {
            link_q[j][i]=LINK[j].q;
        }
        
    }
    gene.t=0.0;
    cout<<"初期姿勢へ以降"<<endl;  
    for(int i=0;i<80;i++)
    {   
        //RSInput(link_q,RS405CB,RS_serial,w_count);
        ECmotorInput(link_q,Arduino,Arduino2,w_count);
        XMInput(link_q,w_count);
        
        w_count++;    
        usleep( 25000 );//10000000　
    }  
    //////////////////////////////////
    WPInit(wp);

    for(int i=0;i<NR_TIMER_INTERRUPTS;i++)
    {
        #ifndef ONE_LEG
        if(i<=50)//50 90
        {
            gene.PatternGenerator(LINK,robot,linkref,wp,linkref[0].p,linkref[1].p,numsteps);
            kine.ModiCoG(LINK,robot,linkref,tofrom);
          //  kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
          //  kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);

        }
        #else
        gene.PatternGenerator(LINK,robot,linkref,wp,linkref[0].p,linkref[1].p,numsteps);
        kine.ModiCoG(LINK,robot,linkref,tofrom);
        //目標ボディ速度・加速度算出

        if(i==0)
        {
            p_body_old=LINK[0].p;
            dp_body_old={0.0,0.0,0.0};
            ddp_body_old={0.0,0.0,0.0};
        }

        if(gene.t==0.0)
        {
          LINK[0].v=dp_body_old;
          LINK[0].acc=ddp_body_old;
        }
        else
        {
          LINK[0].v=(LINK[0].p-p_body_old)/(0.01*DTNUM);
          LINK[0].acc=(LINK[0].v-dp_body_old)/(0.01*DTNUM);
        }

        p_body_old=LINK[0].p;
        dp_body_old=LINK[0].v;
        ddp_body_old=LINK[0].acc;
        //
        //kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
        //kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);
        
        ////
        /*
        for(int i=0; i<LINKNUM; i++)//
        {
            LINK[i].get_q=LINK[i].q;
        }
        State_estimate();*/
        /////
        
        #endif //ONE_LEG

        datalog.logging(LINK,robot,gene);
        datalog.logging_2(LINK,gene);
        //datalog.logging_cog(LINK,robot,state.t);
        for(int j=0;j<15;j++)
        {
            link_q[j][i]=LINK[j].q;
        }
    }

    w_count=0;//初期姿勢
    cout<<"0歩目_遊脚移動"<<endl;  
    //RSInput_init(link_q,RS405CB,RS_serial);
    ECmotorInput(link_q,Arduino,Arduino2,w_count);
    XMInput_init(link_q);
    usleep( INITTIME );

    cout<<"歩行_開始"<<endl;
    Flag_send();//IMU、エンコーダ値を要求(0.5s間待機)
    #ifdef TIMER
    timer_init();
    while(remaining>0)
    {

    }

    //////////////////////////////////
    timer_close();
    #endif //TIMER

    #endif //MOVE_TEST

#endif //TEST

    // for(int i=0;i<RSNUM;i++)
    // {
    //     RS405CB[i].tq_mode=0;
    //     RS_serial.rs_torque(RS405CB[i]);
    //     cout<<"torque off"<<endl;
    // }
    XM_serial.torque_enables(false);
    XM_serial.set_LEDs(false);

    XM_serial.close();
    //RS_serial.rs_close();
    Arduino_s.s_close();
    Arduino2.s_close();
    Arduino.s_close();
    
     for(int i=0;i<NR_TIMER_INTERRUPTS;i++)
     {
        for(int j=0; j<LINKNUM; j++)//
        {
            LINK[j].get_q=link_get_q[j][i];
        }
       State_estimate();
       datalog.logging_cog(LINK,robot,state.t);
       datalog.logging_sensor(link_get_q,link_get_c,i);
     }
    
   //////////////////////////////////
#endif //KINETEST

    return 0;
}

void LinkInit(RobotLink LINK[], int linknum)
{

    for (int i = 0; i < linknum; i++) {
        LINK[i].ID = i;  //配列のインデックスに合わせる

    }
    //もしロボットのモデルを変更するならここで関節軸ベクトル、相対位置ベクトルを変更すればよい

    LINK[0].a = {0.0, 0.0, 0.0};  //ルートリンク
    LINK[0].b = {0.0, 0.0, 0.0};
    LINK[0].c_ ={0.0085, 0.001, 0.0987};//{0.00775, 0.0, 0.062};//{0.0085, 0.001, 0.0945};//
    LINK[0].m=0.575;//;//0.318;//0.575;//

/**/
    //右足
    LINK[1].parentID = 0;//右足ヨー
    LINK[1].a = {0.0, 0.0, 1.0};
    //LINK[1].b = {0.00, -0.03, 0.07};//重心の前後方向の位置を調節するならここ 0.085
    LINK[1].b = {0.00, -0.03, 0.0};//重心の前後方向の位置を調節するならここ 0.085

    LINK[1].c_ ={0.0, 0.0, 0.0};
    LINK[1].m=0.0;

    LINK[2].parentID = 1;//右足ロール
    LINK[2].a = {1.0, 0.0, 0.0};
    LINK[2].b = {0.00775, 0.0, 0.0};
    LINK[2].c_ ={-0.017, 0.0, -0.012};
    LINK[2].m=0.201;

    LINK[3].parentID = 2;//右足ピッチ
    LINK[3].a = {0.0, 1.0, 0.0};
    LINK[3].b = {0.0, 0.0, 0.0};
    LINK[3].c_ ={-0.023, -0.061, -0.032};
    LINK[3].m=0.311;


    LINK[4].parentID = 3;//右足直動
    LINK[4].a = {0.0, 0.0, 1.0};
    LINK[4].b = {0.0, 0.0, 0.0};
    LINK[4].c_ ={0.0065, -0.0445, -0.0927};
    LINK[4].m=0.267;


    LINK[5].parentID = 4;//右足ピッチ
    LINK[5].a = {0.0, 1.0, 0.0};
    LINK[5].b = {0.0, 0.0, -0.3}; //-0.367//足リンクなし-0.382　重心高さ0.4m -0.327 重心高さ0.45m -0.367
    LINK[5].c_ ={-0.017, 0.0, 0.012};
    LINK[5].m=0.201;


    LINK[6].parentID = 5;//右足ロール
    LINK[6].a = {1.0, 0.0, 0.0};
    LINK[6].b = {0.0, 0.0, 0.0};
    LINK[6].c_ ={0.0, -0.011, -0.0225};
    LINK[6].m=0.069;


    LINK[7].parentID = 6;//右足先 仮想の回転関節だとする
    LINK[7].a = {0.0, 0.0, 0.0};
    LINK[7].b = {0.0, 0.0, -0.0265};
    LINK[7].c_ ={0.0, 0.0, 0.0};
    LINK[7].m=0.0;


    //左足
    LINK[8].parentID = 0;
    LINK[8].a = {0.0, 0.0, 1.0};
//    LINK[8].b = {0.00, 0.03, 0.07};
    LINK[8].b = {0.00, 0.03, 0.0};
    LINK[8].c_ ={0.0, 0.0, 0.0};
    LINK[8].m=0.0;


    LINK[9].parentID = 8;
    LINK[9].a = {1.0, 0.0, 0.0};
    LINK[9].b = {0.00775, 0.0, 0.0};
    LINK[9].c_ ={-0.017, 0.0, -0.012};
    LINK[9].m=0.201;


    LINK[10].parentID = 9;
    LINK[10].a = {0.0, 1.0, 0.0};
    LINK[10].b = {0.0, 0.0, 0.0};
    LINK[10].c_ ={-0.023, 0.061, -0.032};
    LINK[10].m=0.311;

    LINK[11].parentID = 10;
    LINK[11].a = {0.0, 0.0, 1.0};
    LINK[11].b = {0.0, 0.0, 0.0};
    LINK[11].c_ ={0.0065, 0.0445, -0.0927};
    LINK[11].m=0.267;

    LINK[12].parentID = 11;
    LINK[12].a = {0.0, 1.0, 0.0};
    LINK[12].b = {0.0, 0.0, -0.3};
    LINK[12].c_ ={-0.017, 0.0, 0.012};
    LINK[12].m=0.201;

    LINK[13].parentID = 12;
    LINK[13].a = {1.0, 0.0, 0.0};
    LINK[13].b = {0.0, 0.0, 0.0};
    LINK[13].c_ ={0.0, 0.011, -0.0225};
    LINK[13].m=0.069;

    LINK[14].parentID = 13;//仮想の回転関節
    LINK[14].a = {0.0, 0.0, 0.0};
    LINK[14].b = {0.0, 0.0, -0.0265};
    LINK[14].c_ ={0.0, 0.0, 0.0};
    LINK[14].m=0.0;

}

void WPInit(walkingparameters wp[])
{  
    wp[0].Cpi={0.0,START};//初期重心位置 右寄りに設定
    wp[0].Cvi={0.0,0.0};//初期重心速度
    wp[0].Pref={0.0,0.0};//目標着地位置
    wp[0].P=wp[0].Pref;//修正着地位置
    wp[0].S={0.0,0.0};//歩行パラメータ
    wp[0].Sz=0.0;//足上げ高さ
    wp[0].Tsup=TSUP;//歩行周期
    wp[0].Tdbl=TDBL;//両足支持期 0歩目は両足支持期は無し

    wp[1].S={0.0,WY};//歩行パラメータ
    wp[1].Sz=0.0;//足上げ高さ
    wp[1].Tsup=TSUP;//歩行周期
    wp[1].Tdbl=TDBL;//両足支持期

    wp[2].S={0.0,WY};//歩行パラメータ
    wp[2].Sz=0.0;//0.008;//足上げ高さ
    wp[2].Tsup=TSUP;//歩行周期
    wp[2].Tdbl=TDBL;//両足支持期

    wp[3].S={WX/4,WY};//歩行パラメータ
    wp[3].Sz=0.0;//0.008;//足上げ高さ
    wp[3].Tsup=TSUP;//歩行周期
    wp[3].Tdbl=TDBL;//両足支持期

    wp[4].S={WX/2,WY};//歩行パラメータ
    wp[4].Sz=0.0;//0.008;//足上げ高さ
    wp[4].Tsup=TSUP;//歩行周期
    wp[4].Tdbl=TDBL;//両足支持期

    wp[5].S={WX,WY};//歩行パラメータ
    wp[5].Sz=0.0;//0.008;//足上げ高さ
    wp[5].Tsup=TSUP;//歩行周期
    wp[5].Tdbl=TDBL;//両足支持期

    wp[6].S={WX,WY};//歩行パラメータ
    wp[6].Sz=0.0;//0.008;//足上げ高さ
    wp[6].Tsup=TSUP;//歩行周期
    wp[6].Tdbl=TDBL;//両足支持期

    wp[7].S={WX/2,WY};//歩行パラメータ
    wp[7].Sz=0.0;//0.008;//足上げ高さ
    wp[7].Tsup=TSUP;//歩行周期
    wp[7].Tdbl=TDBL;//両足支持期

    wp[8].S={0.0,WY};//歩行パラメータ
    wp[8].Sz=0.0;//0.008;//足上げ高さ
    wp[8].Tsup=TSUP;//歩行周期
    wp[8].Tdbl=TDBL;//両足支持期

    wp[9].S={0.0,WY};//歩行パラメータ
    wp[9].Sz=0.0;//0.0;//足上げ高さ
    wp[9].Tsup=TSUP;//歩行周期
    wp[9].Tdbl=TDBL;//両足支持期
/*
    wp[8].S={0.10,0.08};//歩行パラメータ
    wp[8].Tsup=1.0;//歩行周期
    wp[8].Tdbl=0.24;//両足支持期

    wp[9].S={0.10,0.08};//歩行パラメータ
    wp[9].Tsup=1.0;//歩行周期
    wp[9].Tdbl=0.24;//両足支持期

    wp[10].S={0.10,0.08};//歩行パラメータ
    wp[10].Tsup=1.0;//歩行周期
    wp[10].Tdbl=0.24;//両足支持期

    wp[11].S={0.10,0.08};//歩行パラメータ
    wp[11].Tsup=1.0;//歩行周期
    wp[11].Tdbl=0.24;//両足支持期

    wp[12].S={0.10,0.08};//歩行パラメータ
    wp[12].Tsup=1.0;//歩行周期
    wp[12].Tdbl=0.24;//両足支持期

    wp[13].S={0.10,0.08};//歩行パラメータ
    wp[13].Tsup=1.0;//歩行周期
    wp[13].Tdbl=0.24;//両足支持期

    wp[14].S={0.10,0.08};//歩行パラメータ
    wp[14].Tsup=1.0;//歩行周期
    wp[14].Tdbl=0.24;//両足支持期

    wp[15].S={0.10,0.08};//歩行パラメータ
    wp[15].Tsup=1.0;//歩行周期
    wp[15].Tdbl=0.24;//両足支持期

    wp[16].S={0.0,0.08};//歩行パラメータ
    wp[16].Tsup=1.0;//歩行周期
    wp[16].Tdbl=0.24;//両足支持期

    wp[17].S={0.0,0.0};//歩行パラメータ
    wp[17].Tsup=1.0;//歩行周期
    wp[17].Tdbl=0.24;//両足支持期
*/
}

//actuate
void RSInit(Servo_data RS405CB[])
{
    RS405CB[0].ID=1;//右足ヨー
    RS405CB[1].ID=6;//左足ヨー
}

void RSInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial)
{
    cout<<"RSInput_init"<<endl;

    RS405CB[0].Angle_ref=-50+link_q[1][0]*(1800/M_PI); 
    RS405CB[1].Angle_ref=link_q[7][0]*(1800/M_PI); 
    //送信
    for(int i=0;i<RSNUM;i++)
    {
        RS405CB[i].dt_ref=100;//1s
        RS_serial.rs_move(RS405CB[i]);
    }
    
}

void RSInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count)
{
    RS405CB[0].Angle_ref=-50+link_q[1][w_count]*(1800/M_PI); 
    RS405CB[1].Angle_ref=link_q[7][w_count]*(1800/M_PI); 
    //送信
    for(int i=0;i<RSNUM;i++)
    {
        RS405CB[i].dt_ref=1*DTNUM;//*1.5;//*2;//10ms
        RS_serial.rs_move(RS405CB[i]);
    }
}


void ECmotorInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],serial& Arduino,serial& Arduino2,int w_count)
{
    string Angle_str;

    Angle_str=to_string(link_q[4][w_count]);
    if (Arduino.s_write(Angle_str)) 
    {
      cout << "Send : " << Angle_str << endl;
    } 
    else 
    {
      cout << "Send Error"  << endl;
    }    
    // //////////////////////
    Angle_str=to_string(link_q[11][w_count]);
    if (Arduino2.s_write(Angle_str)) 
    {
      cout << "Send : " << Angle_str << endl;
    } 
    else 
    {
      cout << "Send Error"  << endl;
    }

}

void XMInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS])
{
    cout<<"XMInput_init"<<endl;

   XM_serial.angle[0]=-1*link_q[2][0]*(180/M_PI);//右股ロール
   XM_serial.angle[1]=-1*link_q[3][0]*(180/M_PI);//右股ピッチ
   XM_serial.angle[2]=-2.0+link_q[5][0]*(180/M_PI);//右足首ピッチ
   XM_serial.angle[3]=link_q[6][0]*(180/M_PI);//右足首ロール
   XM_serial.angle[4]=-1.5+-1*link_q[9][0]*(180/M_PI);//左股ロール
   XM_serial.angle[5]=link_q[10][0]*(180/M_PI);//左股ピッチ
   XM_serial.angle[6]=-1*link_q[12][0]*(180/M_PI);//右足首ピッチ
   XM_serial.angle[7]=link_q[13][0]*(180/M_PI);//右足首ロール

   XM_serial.dt=1.0;//1s
   XM_serial.angle_time_writes2();
}

void XMInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],int w_count)
{
   XM_serial.angle[0]=-1*link_q[2][w_count]*(180/M_PI);//右股ロール
   XM_serial.angle[1]=-1*link_q[3][w_count]*(180/M_PI);//右股ピッチ
   XM_serial.angle[2]=-2.0+link_q[5][w_count]*(180/M_PI);//右足首ピッチ
   XM_serial.angle[3]=link_q[6][w_count]*(180/M_PI);//右足首ロール
   XM_serial.angle[4]=-1.5+-1*link_q[9][w_count]*(180/M_PI);//左股ロール
   XM_serial.angle[5]=link_q[10][w_count]*(180/M_PI);//左股ピッチ
   XM_serial.angle[6]=-1*link_q[12][w_count]*(180/M_PI);//右足首ピッチ
   XM_serial.angle[7]=link_q[13][w_count]*(180/M_PI);//右足首ロール

   XM_serial.dt=0.01*DTNUM;//*1.5;//*2;//10ms
   
   XM_serial.angle_time_writes2();//目標値送信

}

//sense
void Data_sense(int w_count)
{
   //dynamixelのデータ取得
   XM_serial.get_angles();
   XM_serial.get_currents();
   //IMU、直動部エンコーダの値取得
   Arduino_s.s_read(arduino_sense_data);

   link_get_q[2][w_count]=-1*XM_serial.angle_g[0]*(M_PI/180);//右股ロール
   link_get_q[3][w_count]=-1*XM_serial.angle_g[1]*(M_PI/180);//右股ピッチ
   link_get_q[4][w_count]=-1*(arduino_sense_data[1]/PPR)*2*M_PI*PINION_RADIUS;//右足直動
   link_get_q[5][w_count]=(2.0+XM_serial.angle_g[2])*(M_PI/180);//右足首ピッチ
   link_get_q[6][w_count]=XM_serial.angle_g[3]*(M_PI/180);//右足首ロール
   link_get_q[9][w_count]=(-1.5-1*XM_serial.angle_g[4])*(M_PI/180);//左股ロール
   link_get_q[10][w_count]=XM_serial.angle_g[5]*(M_PI/180);//左股ピッチ
   link_get_q[11][w_count]=(arduino_sense_data[2]/PPR)*2*M_PI*PINION_RADIUS;//左足直動-
   link_get_q[12][w_count]=-1*XM_serial.angle_g[6]*(M_PI/180);//右足首ピッチ
   link_get_q[13][w_count]=XM_serial.angle_g[7]*(M_PI/180);//右足首ロール
   //
   link_get_c[2][w_count]=XM_serial.current_g[0];//右股ロール
   link_get_c[3][w_count]=XM_serial.current_g[1];//右股ピッチ
   link_get_c[5][w_count]=XM_serial.current_g[2];//右足首ピッチ
   link_get_c[6][w_count]=XM_serial.current_g[3];//右足首ロール
   link_get_c[9][w_count]=XM_serial.current_g[4];//左股ロール
   link_get_c[10][w_count]=XM_serial.current_g[5];//左股ピッチ
   link_get_c[12][w_count]=XM_serial.current_g[6];//右足首ピッチ
   link_get_c[13][w_count]=XM_serial.current_g[7];//右足首ロール
}

void Flag_send()
{
    string Send_flag="1";

    if (Arduino_s.s_write(Send_flag)) 
    {
      cout << "Send : " << Send_flag << endl;
    } 
    else 
    {
      cout << "Send Error"  << endl;
    }  

    usleep(500000);
}

void State_estimate()
{
        if((state.stepcount==0)&&(state.tcount==0)&&(state.sup_dub==true))//n=1の着地位置を算出 定常歩行stepcount==2
        {
            state.switching=0;
            state.sup_dub=true;//片足支持

            state.Centroid_estimation(LINK,robot);
            state.tcount++;
 
            cout<<"センシング_歩行開始"<<endl;

        }
        else if((state.tcount>=wp[state.stepcount].Tsup*(1/state.dt))&&(state.sup_dub==true))//両足支持に移行
        {
            if(wp[state.stepcount].Tdbl<=0.001)
            {
                if(state.sign()==1)//右足支持
                {
                    state.switching=1;
                }
                else
                {
                    state.switching=2;
                }
                state.Centroid_estimation(LINK,robot);
                state.tcount=0;
                state.stepcount++;
                
            }
            else
            {
                state.Centroid_estimation(LINK,robot);
                state.tcount=0;
                state.sup_dub=false;//両足支持期に移行
                 cout<<"センシング_両足支持期に移行"<<endl;

            }
            
        }
        else if((state.tcount>=wp[state.stepcount].Tdbl*(1/state.dt))&&(state.sup_dub==false))//片足支持に移行 //n+1 //支持脚切り替えの瞬間だけ計算すればよい
        {
            if(state.sign()==1)//右足支持
            {
                state.switching=1;
            }
            else
            {
                state.switching=2;
            }
            state.Centroid_estimation(LINK,robot);
            state.tcount=0;
            state.stepcount++;
            state.sup_dub=true;//片足支持に移行
            state.switching=0;
            cout<<"センシング_片足支持に移行"<<endl;

        }
        else
        {
            state.Centroid_estimation(LINK,robot);
            state.tcount++;
            cout<<"センシング_定常"<<endl;
        }

        state.t+=state.dt;
}

//timer
void timer_handler(int signam)
{
    if(remaining==NR_TIMER_INTERRUPTS)
    {
        starttime=calcTime();
        oldt=starttime;
        cout<<"0 s"<<endl;
    }
    else if(remaining==1)
    {
        endtime=calcTime();
        cout<<"歩行終了時間"<<endtime-starttime<<"ms"<<endl;
    }

    t=calcTime();
    dt=t-oldt;
    cout<<dt<<"ms"<<endl;
    oldt=t;
    
    //RSInput(link_q,RS405CB,RS_serial,w_count);
    ECmotorInput(link_q,Arduino,Arduino2,w_count);
    XMInput(link_q,w_count);
    Data_sense(w_count);
    
    w_count++;

    remaining--;
    
}

void timer_init()
{
  memset(&action, 0, sizeof(action));
  memset(&old_action, 0, sizeof(struct sigaction));
 
  action.sa_handler = timer_handler;//呼び出す関数名
  action.sa_flags = SA_RESTART;
 
  if (sigaction(SIGALRM, &action, &old_action) == -1)
  {
    perror("sigaction");
    exit(1);//プログラムの異常終了
  }

 ///タイマの設定
  timer.it_value.tv_sec = INIT_WAIT_SEC;
  timer.it_value.tv_usec = 0;
  timer.it_interval.tv_sec = INTERVAL_SEC;//タイマ割り込みの間隔[second]
  timer.it_interval.tv_usec = INTERVAL_MICROSEC;//[microseconds]
 
  if (setitimer(ITIMER_REAL, &timer, &old_timer) == -1)     
  {
    perror("setitimer");
    exit(2);
  }
 
}

void timer_close()
{
  if (setitimer(ITIMER_REAL, &old_timer, NULL) == -1)
  {
    perror("setitimer");
    exit(4);
  }
 
  if (sigaction(SIGALRM, &old_action, NULL) == -1) 
  {
    perror("sigaction");
    exit(5);
  }
}

double calcTime()
{
    struct::timespec getTime;
    clock_gettime(CLOCK_MONOTONIC, &getTime);
    return (getTime.tv_sec + getTime.tv_nsec*1e-9) *1000;
}


