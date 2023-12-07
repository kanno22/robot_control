
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
#define NR_TIMER_INTERRUPTS 1625/2//625/2//1100/2//950/2//840/2//560//繰り返し回数 0.8*7 

static int remaining = NR_TIMER_INTERRUPTS;
struct sigaction action, old_action;
struct itimerval timer, old_timer;
///

#define START 0.03//0.05//0.0004 0.0025 0.8 
#define TIMER
#define TEST
#define INITTIME 1000000//10000000

using namespace Eigen;
using namespace std;

void LinkInit(RobotLink LINK[], int linknum);
void ServoInit(Servo_data RS405CB[]);
void WPInit(walkingparameters wp[]);
void ServoInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count);
void ServoInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count);
void ECmotorInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],serial& Arduino,int w_count);

void timer_init();
void timer_close();
void timer_handler(int signam);//タイマに呼び出される関数

    //////////////////////////////////
    RobotLink LINK[15];//
    RobotLink linkref[2];
    double link_q[15][NR_TIMER_INTERRUPTS];//目標関節変位を格納する配列

    Kinematics kine;
    walkingparameters wp[18];//numpsteps
    walkingpatterngenerator gene;

    serial Arduino;
    Servo_data RS405CB[10];
    Servo_serial RS_serial;

    DataLog datalog;

    int linknum=15;//13
    int tofrom=6;
    int numsteps=17;//9;
    int w_count=0;
    //
    int starttime;
    int endtime;
    int t;
    int oldt;
    int dt;
    //////////////////////////////////

int main()
{
    LinkInit(LINK,linknum);
    ServoInit(RS405CB);
    datalog.log_init();
    //////////////////////////////////
    
    Arduino.s_open(serial::kB115200);
    RS_serial.rs_open();

    for(int i=0;i<10;i++)
    {
        RS405CB[i].tq_mode=1;
        RS_serial.rs_torque(RS405CB[i]);
        cout<<"torque on"<<endl;
    }
    link_q[4][0]=0.04;
    link_q[11][0]=0.04;
    ECmotorInput(link_q,Arduino,0);
    usleep(2*INITTIME );
    link_q[4][0]=0.0;
    link_q[11][0]=0.0;
    ECmotorInput(link_q,Arduino,0);
    usleep(INITTIME );

 #ifdef TEST
       //////////////////////////////////
    LINK[0].p={0.0,0.055,ZC};//{0.0,0.02,0.385};
    LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    linkref[0].p={0.0,0.0,0.0};//右足{0.0,0.0,0.0}
    linkref[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    linkref[1].p={0.0,0.11,0.0};//右足{0.0,0.196,0.0}
    linkref[1].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
    kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);
    
    w_count=0;
    for(int j=0;j<15;j++)
    {
        link_q[j][w_count]=LINK[j].q;
    } 
    w_count=0;//初期姿勢
    cout<<"直立姿勢"<<endl;  
    ServoInput_init(link_q,RS405CB,RS_serial,w_count);
    ECmotorInput(link_q,Arduino,w_count);
    usleep( INITTIME );
    //////////////////////////////////
      
    for(int i=0;i<100;i++)//2s
    {
        LINK[0].p={0.0,((START-0.055)/2.0)*gene.t+0.055,ZC};//{0.0,0.02,0.385};
        LINK[0].R<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
        kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);

        // kine.ForwardKinematics(LINK,7,LINK[1].ID);
        // kine.ForwardKinematics(LINK,7,LINK[8].ID);
      //  datalog.logging(LINK,gene);
        gene.t=gene.t+0.02;
        for(int j=0;j<15;j++)
        {
            link_q[j][i]=LINK[j].q;
        }
        
    }
    gene.t=0.0;
    cout<<"初期姿勢へ以降"<<endl;  
    for(int i=0;i<100;i++)
    {   
        ServoInput(link_q,RS405CB,RS_serial,w_count);
        ECmotorInput(link_q,Arduino,w_count);
        
        w_count++;    
        usleep( 20000 );//10000000　
    }  
    //////////////////////////////////

   //datalog.log_init(); 
    WPInit(wp);

    for(int i=0;i<NR_TIMER_INTERRUPTS;i++)
    {
        gene.PatternGenerator(LINK,linkref,wp,linkref[0].p,linkref[1].p,numsteps);
        kine.InverseKinematics(LINK,linkref[0].p,linkref[0].R,tofrom,LINK[1].ID);
        kine.InverseKinematics(LINK,linkref[1].p,linkref[1].R,tofrom,LINK[8].ID);

        datalog.logging(LINK,gene);
        for(int j=0;j<15;j++)
        {
            link_q[j][i]=LINK[j].q;
        }
    }

    w_count=0;//初期姿勢
    cout<<"0歩目遊脚移動"<<endl;  
    ServoInput_init(link_q,RS405CB,RS_serial,w_count);
    ECmotorInput(link_q,Arduino,w_count);
    usleep( INITTIME );

    cout<<"歩行開始"<<endl;


    #ifdef TIMER
    timer_init();
    while(remaining>0)
    {

    }

    //////////////////////////////////
    timer_close();
    #endif
    for(int i=0;i<10;i++)
    {
        if(i==1)
        {
            RS405CB[i].Angle_ref=-50;    
        }
        else if(i==5)
        {
            RS405CB[i].Angle_ref=100;            
        }
        else if((i==8)||(i==9))
        {
            RS405CB[i].Angle_ref=-50;            
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
#endif

    for(int i=0;i<10;i++)
    {
        RS405CB[i].tq_mode=0;
        RS_serial.rs_torque(RS405CB[i]);
        cout<<"torque off"<<endl;
    }

    RS_serial.rs_close();
    Arduino.s_close();
    
   //////////////////////////////////

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
    LINK[1].b = {0.01, -0.051, 0.083};//-0.018//重心の前後方向の位置を調節するならここ 45+35+3

    LINK[2].parentID = 1;//右足ロール
    LINK[2].a = {1.0, 0.0, 0.0};
    LINK[2].b = {0.0, 0.0, 0.0};

    LINK[3].parentID = 2;//右足ピッチ
    LINK[3].a = {0.0, 1.0, 0.0};
    LINK[3].b = {0.0127, 0.0, -0.035};

    LINK[4].parentID = 3;//右足直動
    LINK[4].a = {0.0, 0.0, 1.0};
    LINK[4].b = {0.0, -0.048, 0.0};

    LINK[5].parentID = 4;//右足ピッチ
    LINK[5].a = {0.0, 1.0, 0.0};
    LINK[5].b = {0.0, 0.0, -0.327}; //-0.367//足リンクなし-0.382　重心高さ0.4m -0.327 重心高さ0.45m -0.367

    LINK[6].parentID = 5;//右足ロール
    LINK[6].a = {1.0, 0.0, 0.0};
    LINK[6].b = {0.0, 0.0105, -0.035};//

    LINK[7].parentID = 6;//右足先 仮想の回転関節だとする
    LINK[7].a = {0.0, 0.0, 0.0};
    LINK[7].b = {0.0, 0.0, -0.0275};

    //左足
    LINK[8].parentID = 0;
    LINK[8].a = {0.0, 0.0, 1.0};
    LINK[8].b = {0.01, 0.051, 0.083};

    LINK[9].parentID = 8;
    LINK[9].a = {1.0, 0.0, 0.0};
    LINK[9].b = {0.0, 0.0, 0.0};

    LINK[10].parentID = 9;
    LINK[10].a = {0.0, 1.0, 0.0};
    LINK[10].b = {0.0127, 0.0, -0.035};

    LINK[11].parentID = 10;
    LINK[11].a = {0.0, 0.0, 1.0};
    LINK[11].b = {0.0, 0.048, 0.0};

    LINK[12].parentID = 11;
    LINK[12].a = {0.0, 1.0, 0.0};
    LINK[12].b = {0.0, 0.0, -0.327};

    LINK[13].parentID = 12;
    LINK[13].a = {1.0, 0.0, 0.0};
    LINK[13].b = {0.0, -0.0105, -0.035};

    LINK[14].parentID = 13;//仮想の回転関節
    LINK[14].a = {0.0, 0.0, 0.0};
    LINK[14].b = {0.0, 0.0, -0.0275};

}

void ServoInit(Servo_data RS405CB[])
{
    for(int i=0;i<10;i++)
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
    wp[0].Tsup=0.25;//歩行周期
    wp[0].Tdbl=0.24;//両足支持期

    wp[1].S={0.0,0.11};//歩行パラメータ
    wp[1].Tsup=1.0;//歩行周期
    wp[1].Tdbl=0.24;//両足支持期

    wp[2].S={0.10,0.11};//歩行パラメータ
    wp[2].Tsup=1.0;//歩行周期
    wp[2].Tdbl=0.24;//両足支持期

    wp[3].S={0.10,0.11};//歩行パラメータ
    wp[3].Tsup=1.0;//歩行周期
    wp[3].Tdbl=0.24;//両足支持期

    wp[4].S={0.10,0.11};//歩行パラメータ
    wp[4].Tsup=1.0;//歩行周期
    wp[4].Tdbl=0.24;//両足支持期

    wp[5].S={0.10,0.11};//歩行パラメータ
    wp[5].Tsup=1.0;//歩行周期
    wp[5].Tdbl=0.24;//両足支持期

    wp[6].S={0.10,0.11};//歩行パラメータ
    wp[6].Tsup=1.0;//歩行周期
    wp[6].Tdbl=0.24;//両足支持期

    wp[7].S={0.10,0.11};//歩行パラメータ
    wp[7].Tsup=1.0;//歩行周期
    wp[7].Tdbl=0.24;//両足支持期

    wp[8].S={0.10,0.11};//歩行パラメータ
    wp[8].Tsup=1.0;//歩行周期
    wp[8].Tdbl=0.24;//両足支持期

    wp[9].S={0.10,0.11};//歩行パラメータ
    wp[9].Tsup=1.0;//歩行周期
    wp[9].Tdbl=0.24;//両足支持期

    wp[10].S={0.10,0.11};//歩行パラメータ
    wp[10].Tsup=1.0;//歩行周期
    wp[10].Tdbl=0.24;//両足支持期

    wp[11].S={0.10,0.11};//歩行パラメータ
    wp[11].Tsup=1.0;//歩行周期
    wp[11].Tdbl=0.24;//両足支持期

    wp[12].S={0.10,0.11};//歩行パラメータ
    wp[12].Tsup=1.0;//歩行周期
    wp[12].Tdbl=0.24;//両足支持期

    wp[13].S={0.10,0.11};//歩行パラメータ
    wp[13].Tsup=1.0;//歩行周期
    wp[13].Tdbl=0.24;//両足支持期

    wp[14].S={0.10,0.11};//歩行パラメータ
    wp[14].Tsup=1.0;//歩行周期
    wp[14].Tdbl=0.24;//両足支持期

    wp[15].S={0.10,0.11};//歩行パラメータ
    wp[15].Tsup=1.0;//歩行周期
    wp[15].Tdbl=0.24;//両足支持期

    wp[16].S={0.0,0.11};//歩行パラメータ
    wp[16].Tsup=1.0;//歩行周期
    wp[16].Tdbl=0.24;//両足支持期

    wp[17].S={0.0,0.0};//歩行パラメータ
    wp[17].Tsup=1.0;//歩行周期
    wp[17].Tdbl=0.24;//両足支持期

}

void ServoInput_init(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count)
{
    cout<<"実行"<<endl;
    for(int i=0;i<10;i++)//サーボの数だけ
    {   
        if(i<3)
        {
            if(i==1)//ID2右ロール
            {
                RS405CB[i].Angle_ref=-50+link_q[i+1][w_count]*(1800/M_PI);    
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+1][w_count]*(1800/M_PI)<<endl;            
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+1][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+1][w_count]*(1800/M_PI)<<endl;
            }
           
        }
        else if((i>=3)&&(i<5))
        {
            if(i==4)
            {
                RS405CB[i].Angle_ref=-link_q[i+2][w_count]*(1800/M_PI);//CCW
                cout<<"サーボID"<<i+1<<"θ="<<-link_q[i+2][w_count]*(1800/M_PI)<<endl;
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+2][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+2][w_count]*(1800/M_PI)<<endl;
            }
            
        }
        else if((i>=5)&&(i<8))
        {
            if(i==6)//ID7　左ロール
            {
                RS405CB[i].Angle_ref=-20+link_q[i+3][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+3][w_count]*(1800/M_PI)<<endl;
            }
            else if(i==7)//ID8　左ピッチ
            {
                 RS405CB[i].Angle_ref=-30-link_q[i+3][w_count]*(1800/M_PI);
                 cout<<"サーボID"<<i+1<<"θ="<<link_q[i+3][w_count]*(1800/M_PI)<<endl;
            }  
            else
            {
                RS405CB[i].Angle_ref=link_q[i+3][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+3][w_count]*(1800/M_PI)<<endl;
            }  
        }
        else
        {
            if(i==8)
            {
                RS405CB[i].Angle_ref=-50-link_q[i+4][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<-link_q[i+4][w_count]*(1800/M_PI)<<endl;
            }
            else
            {
                RS405CB[i].Angle_ref=-50-link_q[i+4][w_count]*(1800/M_PI);
                cout<<"サーボID"<<i+1<<"θ="<<link_q[i+4][w_count]*(1800/M_PI)<<endl;
            }
        }
       //cout<<"サーボID"<<i+1<<"θ="<<RS405CB[i].Angle_ref<<endl;
        RS405CB[i].dt_ref=100;//1s

        //送信
        RS_serial.rs_move(RS405CB[i]);
    } 
}

void ServoInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],Servo_data RS405CB[],Servo_serial& RS_serial,int w_count)
{
    for(int i=0;i<10;i++)//サーボの数だけ
    {
        if(i<3)
        {
            if(i==1)//ID2右ロール
            {
                RS405CB[i].Angle_ref=-50+link_q[i+1][w_count]*(1800/M_PI);
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+1][w_count]*(1800/M_PI);
            }
           
        }
        else if((i>=3)&&(i<5))
        {
            if(i==4)
            {
                RS405CB[i].Angle_ref=-link_q[i+2][w_count]*(1800/M_PI);//CCW
            }
            else
            {
                RS405CB[i].Angle_ref=link_q[i+2][w_count]*(1800/M_PI);
            }
            
        }
        else if((i>=5)&&(i<8))
        {
            if(i==6)//ID7　左ロール
            {
                RS405CB[i].Angle_ref=-20+link_q[i+3][w_count]*(1800/M_PI);
    
            }
            else if(i==7)//ID8　左ピッチ
            {
                 RS405CB[i].Angle_ref=-30-link_q[i+3][w_count]*(1800/M_PI);
                
            }  
            else
            {
                RS405CB[i].Angle_ref=link_q[i+3][w_count]*(1800/M_PI);
            }  
        }
        else
        {
            if(i==8)
            {
                RS405CB[i].Angle_ref=-50-link_q[i+4][w_count]*(1800/M_PI);
            }
            else
            {
                RS405CB[i].Angle_ref=-50-link_q[i+4][w_count]*(1800/M_PI);
            }
             //   RS405CB[i].Angle_ref=-50-link_q[i+4][w_count]*(1800/M_PI);
        }

        RS405CB[i].dt_ref=1*2;//10ms

        //送信
        RS_serial.rs_move(RS405CB[i]);
    } 
}


void ECmotorInput(double (&link_q)[15][NR_TIMER_INTERRUPTS],serial& Arduino,int w_count)
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
    //////////////////////
    Angle_str=to_string(link_q[11][w_count]);
    if (Arduino.s_write(Angle_str)) 
    {
      cout << "Send : " << Angle_str << endl;
    } 
    else 
    {
      cout << "Send Error"  << endl;
    }

}


void timer_handler(int signam)
{
    if(remaining==NR_TIMER_INTERRUPTS)
    {
        starttime=clock();
        oldt=starttime;
        cout<<"0 s"<<endl;
    }
    else if(remaining==1)
    {
        endtime=clock()-starttime;
        cout<<"歩行終了時間"<<endtime<<"s"<<endl;
    }
    t=clock();
    dt=t-oldt;
    cout<<dt<<"ms"<<endl;
    oldt=t;
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


