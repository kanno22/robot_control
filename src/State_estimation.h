#ifndef state_estimation
#define state_estimation

#include<eigen3/Eigen/Eigen>
#include<iostream>
#include"Link.h"
#include "WalkingPatternGenerator.h"

using namespace Eigen;
using namespace std;

class State_estimation
{
    private:
        void ForwardKinematics_self(RobotLink link[], int tofrom, int start);  //順運動学計算
        Matrix3d Rodrigues_self(RobotLink link);  //ロドリゲスの式計算

    public:
        int stepcount;
        int tcount;
        bool sup_dub;//true:片足支持、false:両足支持
        int switching;//0:定常、1:右->左へ切り替えした瞬間、2:左->右へ切り替えした瞬間
        double dt;
        double t;
        State_estimation();
        void Centroid_estimation(RobotLink link[],Robot &robot);
        int sign();//偶数なら-1、奇数なら-1を返す関数
 
};
#endif
