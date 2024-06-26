#ifndef Link
#define Link

#include<eigen3/Eigen/Eigen>

using namespace Eigen;

class RobotLink
{
    public:
        RobotLink();
        int ID;        //自分のID
        int parentID;  //親リンクのID
        
        Vector3d p;     //絶対位置
        Vector3d v;     //絶対速度
        Matrix3d R;     //絶対姿勢
        Vector3d a;     //関節軸ベクトル
        Vector3d b;     //相対位置ベクトル
        double q;      //関節変位[rad]
        double qref;   //目標関節変位[rad]
        double get_q;
};

#endif