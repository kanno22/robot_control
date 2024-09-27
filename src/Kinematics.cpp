#include"Kinematics.h"
#include<math.h>

using namespace Eigen;
using namespace std;

const double tr = M_PI / 180;  //deg→rad

void Kinematics::ForwardKinematics(RobotLink link[],int tofrom,int start)
{
    //i=0はルートリンク
    for (int i = 0; i < tofrom; i++) //int i = start; i < start+tofrom; i++
    {
        RobotLink L;
        L = link[start+i];

        if(i==3)//直動関節
        {
            link[start+i].p = link[link[start+i].parentID].R * (link[start+i].b+link[start+i].a*link[start+i].q) + link[link[start+i].parentID].p;

            link[start+i].R = link[link[start+i].parentID].R;
        }
        else//回転関節
        {
            link[start+i].p = link[link[start+i].parentID].R * link[start+i].b + link[link[start+i].parentID].p;

            link[start+i].R = link[link[start+i].parentID].R * Rodrigues(L);
        }
    }
}

void Kinematics::InverseKinematics(RobotLink link[], Vector3d pref, Matrix3d Rref, int tofrom, int start)
{
    
    //Step2:ルートリンクから目標リンクまでの各関節角度を並べたベクトルを定義
    Vector3d dp, dw;
    Matrix<double, TOFROM,1> q, dq;
    Matrix<double, 6,1> dr;
    Matrix3d dR;
    Matrix<double, 6, TOFROM> J;      //ヤコビ行列
    Matrix<double, TOFROM, 6> pinvJ;  //疑似逆行列
    Matrix<double, TOFROM, TOFROM> A;
    double e = 1.0e-3;
    double k = 0.1;

    for (int i = 0; i < tofrom; i++)  //特異姿勢回避
    {
        if(i==3)//1+3,6+3
        {
            link[i + start].q =0.01;// 0.01;
        }
        else
        {
            link[i + start].q = 10.0 * tr;
        }
    }


    for (int i = 0; i < tofrom; i++) 
    {
        q(i) = link[i + start].q;
    }
   
    
    ForwardKinematics(link, tofrom, start);

    for (int i = 0; i < 100; i++)  //i=100で終了
    {
        
        //Step4:誤差Δp,ΔRを算出
        dp = pref - link[start + (tofrom - 1)].p;
        dR = link[start + (tofrom - 1)].R.transpose() * Rref;
        dw = RotmattoAngvec(dR);

        dr(0) = dp(0);
        dr(1) = dp(1);
        dr(2) = dp(2);
        dr(3) = dw(0);
        dr(4) = dw(1);
        dr(5) = dw(2);

        //Step5:誤差が十分小さいなら終了
        if (err(dp, dw) < e) 
        {
            break;
        }
        if(i==99)
        {
            cout<<"終わり"<<endl;
        } 
      
        //Step6:誤差が大きいなら位置・姿勢の誤差を小さくするような関節角度修正量Δqをニュートンラプソン法により算出
        J = Jacobian(link, tofrom, start);  //ヤコビ行列の算出
        A = J.transpose() * J;
        pinvJ = A.inverse() * J.transpose();  //ヤコビ行列の疑似逆行列を計算
        dq = pinvJ * dr;
        //Step7:qi+1を算出
        q = q + k * dq;
        
        for (int i = 0; i < tofrom; i++)
        {
            link[i + start].q = q(i);
        }
        //Step3:順運動学計算によりqiに対応する目標リンク位置・姿勢pi,Riを求める
        ForwardKinematics(link, tofrom, start);
    }
}

Matrix3d Kinematics::Rodrigues(RobotLink link)
{
    Matrix3d A, E, R;

    A << 0.0, -link.a(2), link.a(1), link.a(2), 0.0, -link.a(0), -link.a(1),
        link.a(0), 0.0;  //ひずみ対称行列

    E << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;  //単位行列

    R = E + A * sin(link.q) + A * A * (1 - cos(link.q));  //ロドリゲスの式

    return R;
}

Matrix<double, 6, TOFROM> Kinematics::Jacobian(RobotLink link[], int tofrom,int start)
{
    Matrix<double, 6, TOFROM> J;  
    Vector3d a, jp;

    for (int i = 0; i < tofrom; i++)  
    {
        if(i==3)//1+3,6+3
        {
            a = {0.0, 0.0, 0.0};  

            jp = link[(link[start+i].parentID)].R * link[start + i].a; //ワールド座標系における関節軸ベクトル
        }
        else
        {
            a = link[(link[start+i].parentID)].R * link[start + i].a;  //ワールド座標系における関節軸ベクトル

            jp = a.cross(link[start + (tofrom - 1)].p - link[start + i].p); 
        }
        
        J(0, i) = jp(0);
        J(1, i) = jp(1);
        J(2, i) = jp(2);
        J(3, i) = a(0);
        J(4, i) = a(1);
        J(5, i) = a(2);
    }

    return J;
}

Vector3d Kinematics::RotmattoAngvec(Matrix3d R)
{
    Matrix3d E;
    Vector3d w, r;
    double theta;

    E << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    if (R == E) 
    {
        w = {0.0, 0.0, 0.0};
    }
    else 
    {
        r = {R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};

        theta = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2);

        w = (theta / (2 * sin(theta))) * r;
    }


    return w;  //各速度ベクトル
}

double Kinematics::err(Vector3d p, Vector3d w)
{
    return p.norm() + w.norm();  //誤差
}

void Kinematics::CalcMass(RobotLink link[], Robot &robot)
{
    double M=0.0;

    for(int i=0; i<LINKNUM; i++)
    {
        M+=link[i].m;
    }

    robot.M=M;
}

void Kinematics::CalcCoG(RobotLink link[],Robot &robot)
{
    Vector3d M_cog={0.0,0.0,0.0};

    CalcMass(link,robot);

    for(int i=0; i<LINKNUM; i++)//各リンクの絶対重心位置を算出
    {
        link[i].c=link[i].p+link[i].R*link[i].c_;
        M_cog+=link[i].m*link[i].c;
    }

    robot.M_cog=M_cog;

    robot.CoG=robot.M_cog/robot.M; //重心位置算出
}

void Kinematics::ModiCoG(RobotLink link[],Robot &robot,RobotLink linkref[], int tofrom)
{
    Vector3d dC;
    Vector3d dw={0.0,0.0,0.0};
    double e = 1.0e-3;
    double k = 1.0;

    //Step1 目標足先位置・姿勢をもらう
    for(int i=0;i<100;i++) //100回繰り返す
    {
        //Step2　逆運動学計算（両足）
        InverseKinematics(link,linkref[0].p,linkref[0].R,tofrom,link[1].ID);//右脚
        InverseKinematics(link,linkref[1].p,linkref[1].R,tofrom,link[8].ID);//左脚

        //Step3 重心位置を計算
        CalcCoG(link,robot);
      //  cout<<"CoG="<<robot.CoG<<endl;
        //Step4 差を取る
        dC=robot.CoGref-robot.CoG;
       // cout<<i<<"回目"<<endl;
        //Step5 終了化否か
        
        // if(abs(dC(1)) < e)//err(dC,dw)<e
        // {
        //     break;
        // }

        if(err(dC,dw)<e)
        {
          //  cout<<"i="<<i<<endl;
            break;
        }

        if(i==99)
        {
            cout<<"重心計算終わり"<<endl;
        }

        //Step6 ボディリンクに修正分を足す
        link[0].p=link[0].p+k*dC;
    //    link[0].p(1)=link[0].p(1)+k*dC(1);
        //cout<<"BODY="<<link[0].p<<endl;
    }
    
}
