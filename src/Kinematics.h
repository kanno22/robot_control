#ifndef kinematics
#define kinematics

#include<eigen3/Eigen/Eigen>
#include<iostream>
#include"Link.h"

#define TOFROM 6

using namespace Eigen;
using namespace std;

class Kinematics
{
    public:
        void ForwardKinematics(RobotLink link[], int tofrom, int start);  //順運動学計算
        void InverseKinematics(RobotLink link[], Vector3d pref, Matrix3d Rref, int tofrom, int start);//逆運動学計算
        Matrix3d Rodrigues(RobotLink link);  //ロドリゲスの式計算
        Matrix<double, 6, TOFROM> Jacobian(RobotLink link[], int tofrom, int start);  //指定されたリンクまでのヤコビ行列を計算
        Vector3d RotmattoAngvec(Matrix3d R);  //回転行列→角速度ベクトル
        double err(Vector3d p, Vector3d w);   //誤差計算

        void CalcMass(RobotLink link[], Robot &robot); //全質量を計算
        void CalcCoG(RobotLink link[],Robot &robot); //重心位置を計算

};

#endif
