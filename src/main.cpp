
// #include<iostream>
//#include "Link.h"
#include "Kinematics.h"
#include "WalkingPatternGenerator.h"
#include "Serial.h"
#include "RSservo.h"
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
#define INTERVAL_MICROSEC 10000*2//10ms=10000μs
#define NR_TIMER_INTERRUPTS 840/2//560//繰り返し回数 0.8*7 

static int remaining = NR_TIMER_INTERRUPTS;
struct sigaction action, old_action;
struct itimerval timer, old_timer;
///

#define START 0.0004//0.0025 0.8
//#define TIMER
#define INITTIME 50000000

using namespace Eigen;
using namespace std;

void LinkInit(RobotLink LINK[], int linknum);
void ServoInit(Servo_data RS405CB[]);
void WPInit(walkingparameters wp[]);
void ServoInput_init(double (&link_q)[13][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count);
void ServoInput(double (&link_q)[13][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count);
void ECmotorInput(double (&link_q)[13][NR_TIMER_INTERRUPTS],serial& Arduino,int w_count);

void timer_init();
void timer_close();
void timer_handler(int signam);//タイマに呼び出される関数

    //////////////////////////////////
    RobotLink LINK[13];//
    RobotLink linkref[2];
    double link_q[13][NR_TIMER_INTERRUPTS];//目標関節変位を格納する配列

    Kinematics kine;
    walkingparameters wp[9];//numpsteps
    walkingpatterngenerator gene;

    serial Arduino;
    Servo_data RS405CB[6];
    Servo_serial RS_serial;

    DataLog datalog;

    int linknum=13;
    int tofrom=6;
    int numsteps=9;
    int w_count=0;
    int starttime;
int endtime;
    //////////////////////////////////

int main()
{

    LinkInit(LINK,linknum);
    ServoInit(RS405CB);
    datalog.log_init();
    //////////////////////////////////
    

    Arduino.s_open();
    RS_serial.rs_open();

    for(int i=0;i<6;i++)
    {
        RS405CB[i].tq_mode=1;
        RS_serial.rs_torque(RS405CB[i]);
        cout<<"torque on"<<endl;

    }

       //////////////////////////////////
    LINK[0].p={0.0,0.065,0.4};//{0.0,0.02,0.385};
    LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    linkref[0].p={0.0,0.0,0.0};//右足{0.0,0.0,0.0}
    linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    linkref[1].p={0.0,0.13,0.0};//右足{0.0,0.196,0.0}
    linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  //  kine.ForwardKinematics(link,tofrom,link[7].ID);
   
    kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
    kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[7].ID);
    w_count=0;
    for(int j=0;j<13;j++)
    {
        link_q[j][w_count]=LINK[j].q;
    }

    //////////////////////////////////
    
    // for(int j=0;j<13;j++)
    // {
    //     link_q[j][0]=LINK[j].q;
    //     if(j==4||j==10)
    //     {
    //         cout<<"リンク"<<LINK[j].ID<<"変位"<<LINK[j].q<<"[m]"<<endl;
    //     }
    //     else
    //     {
    //         cout<<"リンク"<<LINK[j].ID<<"変位"<<LINK[j].q*(180/M_PI)<<"[deg]"<<endl;
    //     }
        
    // }
    ServoInput_init(link_q,RS405CB,RS_serial,w_count);
    ECmotorInput(link_q,Arduino,w_count);
    usleep( 10000000 );        

    //////////////////////////////////
    /*
    LINK[0].p={0.0,START,0.4};//{0.0,0.02,0.385};
    LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    linkref[0].p={0.0,0.0,0.0};//右足{0.0,0.0,0.0}
    linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    linkref[1].p={0.0,0.13,0.0};//右足{0.0,0.196,0.0}
    linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  //  kine.ForwardKinematics(link,tofrom,link[7].ID);
   
    kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
    kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[7].ID);*/
    //////////////////////////////////


cout<<"初期姿勢へ以降"<<endl;  
      
    for(int i=0;i<100;i++)//2s
    {
        LINK[0].p={0.0,((START-0.065)/2.0)*gene.t+0.065,0.4};//{0.0,0.02,0.385};
        LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
        kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[7].ID);

        datalog.logging(LINK,gene);
        gene.t=gene.t+0.02;
        for(int j=0;j<13;j++)
        {
            link_q[j][i]=LINK[j].q;
        }
        
    }
    gene.t=0.0;


    for(int i=0;i<100;i++)
    {   
        ServoInput(link_q,RS405CB,RS_serial,w_count);
        ECmotorInput(link_q,Arduino,w_count);
        
        w_count++;    
        usleep( 20000 );//10000000
    }  
    cout<<"初期姿勢へ以降"<<endl;   
    usleep( INITTIME );
    //////////////////////////////////
#ifdef TIMER
    datalog.log_init(); 
    WPInit(wp);

    for(int i=0;i<NR_TIMER_INTERRUPTS;i++)
    {
        gene.PatternGenerator(LINK,linkref,wp,linkref[0].p,linkref[1].p,numsteps);
        kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
        kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[7].ID);

        datalog.logging(LINK,gene);
        for(int j=0;j<13;j++)
        {
            link_q[j][i]=LINK[j].q;
        }

    }

    w_count=0;//初期姿勢


    timer_init();
    while(remaining>0)
    {

    }
//    for(int i=0;i<560;i++)//4s
//    {
//     gene.PatternGenerator(link,linkref,wp,linkref[0].p,linkref[1].p,numsteps);
//     kine.InverseKinematics(link,linkref[0].p,linkref[0].R,tofrom,link[1].ID);
//     kine.InverseKinematics(link,linkref[1].p,linkref[1].R,tofrom,link[7].ID);
//     // cout<<"右[m]"<<link[4].q<<endl;
//     // cout<<"左[m]"<<link[10].q<<endl;
//     #ifdef SERIAL
//     ServoInput(link,RS405CB,RS_serial);
//     ECmotorInput(link,Arduino);
    
//     usleep( 10000 );
//     #endif
//    }
    //////////////////////////////////
    timer_close();
    #endif
    for(int i=0;i<6;i++)
    {
        if(i==1)
        {
            RS405CB[i].Angle_ref=-50;    
        }
        else if(i==3)
        {
            RS405CB[i].Angle_ref=100;            
        }
        else
        {
            RS405CB[i].Angle_ref=000;
        }
        RS405CB[i].dt_ref=100;
        RS_serial.rs_move(RS405CB[i]);
        cout<<"initial posture"<<endl;
    }
    usleep( 1000000 );
    for(int i=0;i<6;i++)
    {
        RS405CB[i].tq_mode=0;
        RS_serial.rs_torque(RS405CB[i]);
        cout<<"torque off"<<endl;
    }


    RS_serial.rs_close();
    Arduino.s_close();
    

   //////////////////////////////////
    cout<<"wp1.pref=\n"<<wp[1].Pref <<endl;
    cout<<"wp1.p=\n"<<wp[1].P <<endl;
    cout<<"wp2.pref=\n"<<wp[2].Pref <<endl;
    cout<<"wp2.p=\n"<<wp[2].P <<endl;
    cout<<"wp3.pref=\n"<<wp[3].Pref <<endl;
    cout<<"wp3.p=\n"<<wp[3].P <<endl;
    cout<<"wp4.pref=\n"<<wp[4].Pref <<endl;
    cout<<"wp4.p=\n"<<wp[4].P <<endl;
    cout<<"wp5.pref=\n"<<wp[5].Pref <<endl;
    cout<<"wp5.p=\n"<<wp[5].P <<endl;
    cout<<"wp6.pref=\n"<<wp[6].Pref <<endl;
    cout<<"wp6.p=\n"<<wp[6].P <<endl;
    cout<<"wp7.pref=\n"<<wp[7].Pref <<endl;
    cout<<"wp7.p=\n"<<wp[7].P <<endl;

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

    //右足
    LINK[1].parentID = 0;//右足ヨー
    LINK[1].a = {0.0, 0.0, 1.0};
    LINK[1].b = {0.0, -0.051, -0.018};//x12.71mm -0.0125, -0.051, 0.0 

    LINK[2].parentID = 1;//右足ロール
    LINK[2].a = {1.0, 0.0, 0.0};
    LINK[2].b = {0.0, 0.0, 0.0};

    LINK[3].parentID = 2;//右足ピッチ
    LINK[3].a = {0.0, 1.0, 0.0};
    LINK[3].b = {0.0127, 0.0, -0.035};//0.0125, 0.0, -0.035

    LINK[4].parentID = 3;//右足直動
    LINK[4].a = {0.0, 0.0, 1.0};
    LINK[4].b = {0.0, -0.048, 0.0};//y48.076mm 0.0, -0.047, 0.0

    LINK[5].parentID = 4;//仮想の回転関節だとする
    LINK[5].a = {0.0, 1.0, 0.0};
    LINK[5].b = {0.0, 0.0, -0.355};//0.0, 0.0, -0.35

    LINK[6].parentID = 5;//右足先 仮想の回転関節だとする
    LINK[6].a = {1.0, 0.0, 0.0};
    LINK[6].b = {0.0, 0.0, 0.0};

    //左足
    LINK[7].parentID = 0;
    LINK[7].a = {0.0, 0.0, 1.0};
    LINK[7].b = {0.0, 0.051, -0.018};

    LINK[8].parentID = 7;
    LINK[8].a = {1.0, 0.0, 0.0};
    LINK[8].b = {0.0, 0.0, 0.0};

    LINK[9].parentID = 8;
    LINK[9].a = {0.0, 1.0, 0.0};
    LINK[9].b = {0.0127, 0.0, -0.035};

    LINK[10].parentID = 9;
    LINK[10].a = {0.0, 0.0, 1.0};
    LINK[10].b = {0.0, 0.048, 0.0};

    LINK[11].parentID = 10;//仮想の回転関節
    LINK[11].a = {0.0, 1.0, 0.0};
    LINK[11].b = {0.0, 0.0, -0.355};

    LINK[12].parentID = 11;//仮想の回転関節
    LINK[12].a = {1.0, 0.0, 0.0};
    LINK[12].b = {0.0, 0.0, 0.0};

}

void ServoInit(Servo_data RS405CB[])
{
    for(int i=0;i<6;i++)
    {
        RS405CB[i].ID=i+1;
    }
}

void WPInit(walkingparameters wp[])
{  
    wp[0].Cpi={0.0,START};//初期重心位置 右寄りに設定
    wp[0].Cvi={0.0,0.0};//初期重心速度
    wp[0].Pref={0.0,0.0};//目標着地位置
    wp[0].P=wp[0].Pref;//修正着地位置
    wp[0].S={0.0,0.0};//歩行パラメータ
    wp[0].Tsup=1.2;//歩行周期
    wp[0].Tdbl=0.24;//両足支持期

    wp[1].S={0.0,0.13};//歩行パラメータ
    wp[1].Tsup=1.2;//歩行周期
    wp[1].Tdbl=0.24;//両足支持期

    wp[2].S={0.1,0.13};//歩行パラメータ
    wp[2].Tsup=1.2;//歩行周期
    wp[2].Tdbl=0.24;//両足支持期

    wp[3].S={0.1,0.13};//歩行パラメータ
    wp[3].Tsup=1.2;//歩行周期
    wp[3].Tdbl=0.24;//両足支持期

    wp[4].S={0.1,0.13};//歩行パラメータ
    wp[4].Tsup=1.2;//歩行周期
    wp[4].Tdbl=0.24;//両足支持期

    wp[5].S={0.1,0.13};//歩行パラメータ
    wp[5].Tsup=1.2;//歩行周期
    wp[5].Tdbl=0.24;//両足支持期

    wp[6].S={0.1,0.13};//歩行パラメータ
    wp[6].Tsup=1.2;//歩行周期
    wp[6].Tdbl=0.24;//両足支持期

    wp[7].S={0.0,0.13};//歩行パラメータ
    wp[7].Tsup=1.2;//歩行周期
    wp[7].Tdbl=0.24;//両足支持期

    wp[8].S={0.0,0.0};//歩行パラメータ
    wp[8].Tsup=1.2;//歩行周期
    wp[8].Tdbl=0.24;//両足支持期

}

void ServoInput_init(double (&link_q)[13][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count)
{
    for(int i=0;i<6;i++)//サーボの数だけ
    {
        if(i<3)
        {
            if(i==1)//ID2
            {
                RS405CB[i].Angle_ref=-50+link_q[i+1][w_count]*(1800/M_PI);
            }
            else if(i==3)//ID4
            {
                RS405CB[i].Angle_ref=100+link_q[i+1][w_count]*(1800/M_PI);
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+1][w_count]*(1800/M_PI);
            }
           // RS405CB[i].Angle_ref=LINK[i+1].q*(1800/M_PI);//1~3
           
        }
        else if(i==5)
        {
             RS405CB[i].Angle_ref=-link_q[i+4][w_count]*(1800/M_PI);
        }
        else
        {
           // RS405CB[i].Angle_ref=LINK[i+4].q*(1800/M_PI);//7~9
           RS405CB[i].Angle_ref=link_q[i+4][w_count]*(1800/M_PI);
        }
        RS405CB[i].dt_ref=100;//1s

        //送信
        RS_serial.rs_move(RS405CB[i]);
    } 
}

void ServoInput(double (&link_q)[13][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count)
{
    for(int i=0;i<6;i++)//サーボの数だけ
    {
        if(i<3)
        {
            if(i==1)//ID2
            {
                RS405CB[i].Angle_ref=-50+link_q[i+1][w_count]*(1800/M_PI);
            }
            else if(i==3)//ID4
            {
                RS405CB[i].Angle_ref=100+link_q[i+1][w_count]*(1800/M_PI);
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+1][w_count]*(1800/M_PI);
            }
           // RS405CB[i].Angle_ref=LINK[i+1].q*(1800/M_PI);//1~3
           
        }
        else if(i==5)
        {
             RS405CB[i].Angle_ref=-link_q[i+4][w_count]*(1800/M_PI);
        }
        else
        {
           // RS405CB[i].Angle_ref=LINK[i+4].q*(1800/M_PI);//7~9
           RS405CB[i].Angle_ref=link_q[i+4][w_count]*(1800/M_PI);
        }
        RS405CB[i].dt_ref=1*2;//10ms

        //送信
        RS_serial.rs_move(RS405CB[i]);
    } 
}

void ECmotorInput(double (&link_q)[13][NR_TIMER_INTERRUPTS],serial& Arduino,int w_count)
{
    int Angle;

    //Angle=LINK[4].q*10000;//4 10
    Angle=link_q[4][w_count]*10000;

    Arduino.buf_w='R';//ヘッダ
    Arduino.s_write();
    Arduino.buf_w=Angle>>8;//上位バイト
    Arduino.s_write();
    Arduino.buf_w=Angle&0x00FF;//下位バイト
    Arduino.s_write();    
    Arduino.s_read();
    //////////////////////
 //  Angle=LINK[10].q*10000;//100;//Link[10].q*10000;//4 10
   Angle=link_q[10][w_count]*10000;
    Arduino.buf_w='L';//ヘッダ
    Arduino.s_write();
    Arduino.buf_w=Angle>>8;//上位バイト
    Arduino.s_write();
    Arduino.buf_w=Angle&0x00FF;//下位バイト
    //cout<<((Angle>>8)|(Angle&0x00FF))<<endl;
    Arduino.s_write();  
    Arduino.s_read();  

}

void timer_handler(int signam)
{
    if(remaining==560/2)
    {
        starttime=clock();
        cout<<"0 s"<<endl;
    }
    else if(remaining==1)
    {
        endtime=clock()-starttime;
        cout<<endtime<<endl;
    }
    ServoInput(link_q,RS405CB,RS_serial,w_count);
    ECmotorInput(link_q,Arduino,w_count);
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


