#ifndef WalkingPatternGenerator
#define WalkingPatternGenerator

#include<eigen3/Eigen/Eigen>
#include<math.h>
#include "Link.h"
#include"WalkingParameters.h"

#include<iostream>

#define ZC 0.215//0.165//0.33//0.4//0.45

using namespace Eigen;

class walkingpatterngenerator
{
    private:
        Vector2d Cpref;//重心位置(目標ボディリンク位置)
        Vector2d Cvref;//重心速度
        Vector3d prefr;//右足目標位置
        Vector3d prefl;//左足目標位置
        Matrix3d Rrefr;//右足目標姿勢
        Matrix3d Rrefl;//左足目標姿勢

        double zc;       //重心高さ
        double g;        //重力加速度
        double Tc;
        double C;       //cosh(Tsup/Tc)
        double S;       //sinh(Tsup/Tc)
        
        //重み関数パラメータ
        double D;       
        double a;       
        double b;

        int stepcount;
    
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
        void PatternGenerator(RobotLink link[],RobotLink linkref[],walkingparameters wp[],Vector3d Prefr,Vector3d Prefl,int numsteps);
         int sign(int stepcount);
};

#endif