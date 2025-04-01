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
        //自己位置推定用変数
        Vector3d p_self; //自己座標系から見た時の位置
        Matrix3d R_self; //自己座標系から見た時の姿勢
        Vector3d p_gnd; //地面座標系から見た時の位置
        Matrix3d R_gnd; //地面座標系から見た時の姿勢
        Vector3d c_self;//自己座標系から見た時の各リンク重心位置
        double get_q;  //現在の関節変位[rad or m]
        double get_c;
};

class Robot
{
    public:
        Robot();

        Vector3d CoG;
        Vector3d CoGref;
        Vector3d dCoGref;
        Vector3d ddCoGref;

        Vector3d M_cog; //重心モーメント
        double M;
        //自己位置推定用変数
        Vector3d CoG_self;
        Vector3d CoG_gnd;
        Vector3d p_s_gnd; //自己座標系から見た時の地面座標系の位置
        Matrix3d R_s_gnd; //自己座標系から見た時の地面座標系の姿勢
        Vector3d p_g_self; //地面から見た時の自己座標系の位置
        Matrix3d R_g_self; //地面座標系から見た時の自己座標系の位置

        Vector3d p_O_GND;//歩き初めの位置からの見た時の地面座標系の位置
        Vector3d CoG_O_GND[2];//歩き初めの位置からの見た時の重心位置
        Vector3d dCoG_O_GND[2];
        Vector3d ddCoG_O_GND;
        Vector3d p_body_GND[2];//歩き始めの位置から見た時のボディ位置
        Vector3d dp_body_GND[2];
        Vector3d ddp_body_GND;
        //
};

#endif