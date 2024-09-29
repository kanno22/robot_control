#ifndef WalkingPatternGenerator
#define WalkingPatternGenerator

#include<eigen3/Eigen/Eigen>
#include<math.h>
#include "Link.h"
#include"WalkingParameters.h"

#include<iostream>

#define ZC 0.250//0.270//0.250
#define DZC 0.0265// 
#define DTNUM 2.5//1.0//2.0
#define DWR -0.008//-0.015//-0.008//-0.01//-0.005//-0.005//-0.01//-0.025//ZMPより足リンク位置は左右2.5cmほど外側に設定
#define DWL 0.008//0.015//0.008//0.01//0.005//0.005//0.01//0.025//ZMPより足リンク位置は左右2.5cmほど外側に設定


using namespace Eigen;

class walkingpatterngenerator
{
    private:
        Vector2d Cpref;//重心位置(目標ボディリンク位置)
        Vector2d Cvref;//重心速度
        Vector2d Caref;//重心加速度
        Vector3d prefr;//右足目標位置
        Vector3d prefl;//左足目標位置
        Matrix3d Rrefr;//右足目標姿勢
        Matrix3d Rrefl;//左足目標姿勢

        Vector3d dprefr;
        Vector3d dprefl;

        Vector2d Csum;//両足支持期の増分の総和
        
        double zc;       //重心高さ
        double g;        //重力加速度
        double Tc;
        double C;       //cosh(Tsup/Tc)
        double S;       //sinh(Tsup/Tc)

        //両足支持期における補間関数の係数
        Vector2d a0,a1,a2,a3,a4;

        //重み関数パラメータ
        double D;       
        double a;       
        double b;

        int stepcount;
        bool sup_dub;//true:片足支持、false:両足支持
    
       // double t;
        int tcount;
        double dt;

        int tofrom;
        int Rstart;
        int Lstart;

        double Rx, Ry, Rz;//サイクロイドの半径
        double w;//サイクロイド曲線の角速度(固有角振動数っていったほうがいいかな)

    public:
        double t;
        walkingpatterngenerator();
        void PatternPlanner(walkingparameters wp[]);
        void PatternGenerator(RobotLink link[],Robot &robot,RobotLink linkref[],walkingparameters wp[],Vector3d Prefr,Vector3d Prefl,int numsteps);
         int sign(int stepcount);
};

#endif