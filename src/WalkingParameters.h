#ifndef WalkingParameters
#define WalkingParameters

#include<eigen3/Eigen/Eigen>
using namespace Eigen;

class walkingparameters
{
    public:
        walkingparameters();
        Vector2d Cpel;//歩行素片
        Vector2d Cvel;
        
        Vector2d Cpd;//目標最終重心位置
        Vector2d Cvd;//目標最終重心速度
        Vector2d Cpi;//初期重心位置
        Vector2d Cvi;//初期重心速度
        Vector2d Cai;//初期重心加速度
        Vector2d Cpf;//最終重心位置
        Vector2d Cvf;//最終重心速度
        Vector2d Caf;//最終重心加速度

        Vector2d dCp;//両足支持期の重心位置・接地位置の増加分

        Vector2d Pref;//目標着地位置
        Vector2d P;//修正着地位置

        Vector2d S;//歩行パラメータ
        double Sz;//足上げ高さ

        double Tsup;//歩行周期
        double Tdbl;//両足支持期

    

};

#endif