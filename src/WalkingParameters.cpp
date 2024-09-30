#include"WalkingParameters.h"

using namespace Eigen;

walkingparameters::walkingparameters()
{
        Cpd={0.0,0.0};//目標最終重心位置
        Cvd={0.0,0.0};//目標最終重心速度
        Cpi={0.0,0.0};//初期重心位置
        Cvi={0.0,0.0};//初期重心速度
        Cai={0.0,0.0};//初期重心加速度
        Cpf={0.0,0.0};//最終重心位置
        Cvf={0.0,0.0};//最終重心速度
        Caf={0.0,0.0};//最終重心加速度

        dCp={0.0, 0.0};

        Pref={0.0,0.0};//目標着地位置
        P={0.0,0.0};//修正着地位置

        S={0.0,0.0};//歩行パラメータ
        Sz=0.0;

        Tsup=0.0;//歩行周期
        Tdbl=0.0;//両足支持期
}