#ifndef Link
#define Link

#include<eigen3/Eigen/Eigen>

#define LINKNUM 15

using namespace Eigen;

class RobotLink
{
    public:
        RobotLink();
        int ID;        //自分のID
        int parentID;  //親リンクのID
        
        Vector3d p;     //絶対位置
        Vector3d v;     //絶対速度
        Vector3d acc;     //絶対加速度
        Matrix3d R;     //絶対姿勢
        Vector3d a;     //関節軸ベクトル
        Vector3d b;     //相対位置ベクトル
        Vector3d c_;    //相対重心位置
        Vector3d c;     //絶対重心位置

        double m;      //質量[kg]
        double q;      //関節変位[rad]
        double qref;   //目標関節変位[rad]

        double get_q;
        double get_c;
};

class Robot
{
    public:
        Robot();

        Vector3d CoG;
        Vector3d M_cog; //重心モーメント
        double M;

};

#endif