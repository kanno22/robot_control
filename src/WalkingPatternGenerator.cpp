#include"WalkingPatternGenerator.h"

using namespace Eigen;
using namespace std;


    walkingpatterngenerator::walkingpatterngenerator()
    {
        Cpref={0.0,0.0};
        Cvref={0.0,0.0};
        prefr={0.0,0.0,0.0};
        prefl={0.0,0.0,0.0};
        Rrefr<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        Rrefl<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

        zc=0.4;       //[m]
        g=9.81;        //[m/s^2]
        Tc=sqrt(zc/g);
        C=0.0;       
        S=0.0;       
        
        //重み関数パラメータ
        D=0.0;       
        a=10;       
        b=1;

        stepcount=0;//定常2
    
        t=0.0;
        tcount=0;
        dt=0.01*2;//10ms

        tofrom=0;
        Rstart=0;
        Lstart=0;

        Rx=0.0;
        Ry=0.0;
        Rz=0.02;//4cm足を上げる
        w=0.0;
    };

    void walkingpatterngenerator::PatternPlanner(walkingparameters wp[])//まずは歩行パターンと歩数をあらかじめ与えたverを考える
    {
        C=cosh(wp[stepcount+1].Tsup/Tc);
        S=sinh(wp[stepcount+1].Tsup/Tc);
        D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

        wp[stepcount].Cpf = (wp[stepcount].Cpi-wp[stepcount].P)*cosh(wp[stepcount].Tsup/Tc)+Tc*wp[stepcount].Cvi*sinh(wp[stepcount].Tsup/Tc)+wp[stepcount].P;//n=0では修正着地位置=目標着地位置              
        wp[stepcount].Cvf = ((wp[stepcount].Cpi-wp[stepcount].P)/Tc)*sinh(wp[stepcount].Tsup/Tc)+wp[stepcount].Cvi*cosh(wp[stepcount].Tsup/Tc);              
        wp[stepcount+1].Cpi = wp[stepcount].Cpf;  //n歩目の最終重心位置をn+1歩目の初期重心位置としている
        wp[stepcount+1].Cvi = wp[stepcount].Cvf;  //n歩目の最終重心速度をn+1歩目の初期重心速度としている


        //Step5 n+1の目標着地位置 
        wp[stepcount+1].Pref(0)=wp[stepcount].Pref(0)+wp[stepcount+1].S(0);//修正後の着地位置の方がいいのか？
        wp[stepcount+1].Pref(1)=wp[stepcount].Pref(1)-wp[stepcount+1].S(1)*sign(stepcount+1);
        
        //Step6 n+1の歩行素片
        wp[stepcount+1].Cpel(0)=wp[stepcount+2].S(0)/2;
        wp[stepcount+1].Cpel(1)=sign(stepcount+1)*wp[stepcount+2].S(1)/2;
        wp[stepcount+1].Cvel(0)=(C+1)*wp[stepcount+1].Cpel(0)/(Tc*S);
        wp[stepcount+1].Cvel(1)=(C-1)*wp[stepcount+1].Cpel(1)/(Tc*S);

        //Step7 n+1の目標最終重心位置・速度
        wp[stepcount+1].Cpd=wp[stepcount+1].Pref+wp[stepcount+1].Cpel;
        wp[stepcount+1].Cvd=wp[stepcount+1].Cvel;

        //Step8 n+1の修正着地位置
        wp[stepcount+1].P(0) = (-a * (C - 1) / D)* (wp[stepcount+1].Cpd(0)
                                    - C * wp[stepcount+1].Cpi(0)
                                    - Tc * S * wp[stepcount+1].Cvi(0))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount+1].Cvd(0)
                                      - S * wp[stepcount+1].Cpi(0) / Tc
                                      - C * wp[stepcount+1].Cvi(0));

        wp[stepcount+1].P(1) = (-a * (C - 1) / D)
                                 * (wp[stepcount+1].Cpd(1)
                                    - C * wp[stepcount+1].Cpi(1)
                                    - Tc * S * wp[stepcount+1].Cvi(1))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount+1].Cvd(1)
                                      - S * wp[stepcount+1].Cpi(1) / Tc
                                      - C * wp[stepcount+1].Cvi(1));

        // wp[stepcount+1].Cpf = (wp[stepcount+1].Cpi-wp[stepcount+1].P)*cosh(wp[stepcount+1].Tsup/Tc)+Tc*wp[stepcount+1].Cvi*sinh(wp[stepcount+1].Tsup/Tc)+wp[stepcount+1].P;//n=0では修正着地位置=目標着地位置              
        // wp[stepcount+1].Cvf = ((wp[stepcount+1].Cpi-wp[stepcount+1].P)/Tc)*sinh(wp[stepcount+1].Tsup/Tc)+wp[stepcount+1].Cvi*cosh(wp[stepcount+1].Tsup/Tc);              
       
        // cout<<"Cpf=\n"<<wp[stepcount+1].Cpf<<endl;
        // cout<<"Cvf=\n"<<wp[stepcount+1].Cvf<<endl;
        // cout<<"Cpd=\n"<<wp[stepcount+1].Cpd<<endl;
        // cout<<"Cvd=\n"<<wp[stepcount+1].Cvd<<endl;
        // cout<<"次"<<endl;
        
    };

    void walkingpatterngenerator::PatternGenerator(RobotLink link[],RobotLink linkref[],walkingparameters wp[],Vector3d Prefr,Vector3d Prefl,int numsteps)
    {
        //右足をn=0とする

        if((stepcount==0)&&(tcount==0))//n=1の着地位置を算出 定常歩行stepcount==2
        {
            wp[stepcount].P(0)=Prefr(0);
            wp[stepcount].P(1)=Prefr(1);

            //定常歩行
            // wp[stepcount-1].Pref={0.0,0.196};
            // wp[stepcount-1].P={0.0,0.196};
            // wp[stepcount].Pref={0.1,0.0};
            // wp[stepcount].P={0.1,0.0};
            // wp[stepcount-1].Cpel={wp[stepcount].S(0)/2,-wp[stepcount].S(1)/2};
            // wp[stepcount-1].Cvel={(cosh(wp[stepcount-1].Tsup/Tc)+1)*wp[stepcount-1].Cpel(0)/(Tc*sinh(wp[stepcount-1].Tsup/Tc)),(cosh(wp[stepcount-1].Tsup/Tc)-1)*wp[stepcount-1].Cpel(1)/(Tc*sinh(wp[stepcount-1].Tsup/Tc))};
            // wp[stepcount].Cpi=wp[stepcount-1].P+(wp[stepcount-1].Cpel);
            // wp[stepcount].Cvi=wp[stepcount-1].Cvel;
            //

            //Step5~8
            PatternPlanner(wp);
             cout<<"\n"<<stepcount<<"回目"<<endl;
            // cout<<"Cpi=\n"<<wp[stepcount].Cpi<<endl;
            // cout<<"Cvi=\n"<<wp[stepcount].Cvi<<endl;
            // cout<<"Cpf=\n"<<wp[stepcount].Cpf<<endl;
            // cout<<"Cvf=\n"<<wp[stepcount].Cvf<<endl;

            t+=dt;
            tcount++;
            

        }
        else if(tcount>=wp[stepcount].Tsup*(1/dt))//n+1 //接地した瞬間だけ計算すればよい
        {
            t=0.0;//リセット
            tcount=0;
            stepcount++;

            //Step5~8
            PatternPlanner(wp);
             cout<<"\n"<<stepcount<<"回目"<<endl;
        }
        else
        {
            t+=dt;
            tcount++;
        }

        //Step3 重心軌道生成
        Cpref = (wp[stepcount].Cpi-wp[stepcount].P)*cosh(t/Tc)+Tc*wp[stepcount].Cvi*sinh(t/Tc)+wp[stepcount].P;//n=0では修正着地位置=目標着地位置              
        Cvref = ((wp[stepcount].Cpi-wp[stepcount].P)/Tc)*sinh(t/Tc)+wp[stepcount].Cvi*cosh(t/Tc);        

        link[0].p(0)=Cpref(0);
        link[0].p(1)=Cpref(1);
        link[0].p(2)=zc;
        //遊脚軌道生成
        if(sign(stepcount)==1)//右足が支持脚
        {
            w=(2*M_PI)/wp[stepcount].Tsup;
            
            prefr(0)=wp[stepcount].P(0);//支持脚
            prefr(1)=wp[stepcount].P(1);
            prefr(2)=0.0;

            if(stepcount==0)
            {
                Rx=(wp[stepcount+1].P(0)-Prefl(0))/(2*M_PI);
                Ry=(wp[stepcount+1].P(1)-Prefl(1))/(2*M_PI);

                prefl(0)=Prefl(0)+Rx*(w*t-sin(w*t));//遊脚脚
                prefl(1)=Prefl(1)+Ry*(w*t-sin(w*t));
                prefl(2)=Rz*(1-cos(w*t));
            }
            else
            {
                Rx=(wp[stepcount+1].P(0)-wp[stepcount-1].P(0))/(2*M_PI);
                Ry=(wp[stepcount+1].P(1)-wp[stepcount-1].P(1))/(2*M_PI);

                prefl(0)=wp[stepcount-1].P(0)+Rx*(w*t-sin(w*t));//遊脚脚
                prefl(1)=wp[stepcount-1].P(1)+Ry*(w*t-sin(w*t));
                prefl(2)=Rz*(1-cos(w*t));
            }

        }
        else//左足が支持脚
        {
            w=(2*M_PI)/wp[stepcount].Tsup;
            
            prefl(0)=wp[stepcount].P(0);//支持脚
            prefl(1)=wp[stepcount].P(1);
            prefl(2)=0.0;
                
            Rx=(wp[stepcount+1].P(0)-wp[stepcount-1].P(0))/(2*M_PI);
            Ry=(wp[stepcount+1].P(1)-wp[stepcount-1].P(1))/(2*M_PI);

            prefr(0)=wp[stepcount-1].P(0)+Rx*(w*t-sin(w*t));//遊脚脚
            prefr(1)=wp[stepcount-1].P(1)+Ry*(w*t-sin(w*t));
            prefr(2)=Rz*(1-cos(w*t));

        }

        linkref[0].p=prefr;
        linkref[0].R=Rrefr;
        linkref[1].p=prefl;
        linkref[1].R=Rrefr;
    };

    int walkingpatterngenerator::sign(int stepcount)
    {
        if(((stepcount % 2)==0)||stepcount==0)//右足が始め
        {
            return 1;//右足支持なら1
        }
        else
        {
            return -1;//左足支持なら-1
        }
    };