#include"WalkingPatternGenerator.h"

using namespace Eigen;
using namespace std;


    walkingpatterngenerator::walkingpatterngenerator()
    {
        Cpref={0.0,0.0};
        Cvref={0.0,0.0};
        Caref={0.0,0.0};
        prefr={0.0,0.0,0.0};
        prefl={0.0,0.0,0.0};
        Rrefr<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        Rrefl<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

        dprefr={0.0,DWR,0.0};
        dprefl={0.0,DWL,0.0};

        Csum={0.0,0.0};
       
        zc=ZC;       //[m]
        g=9.81;        //[m/s^2]
        Tc=sqrt(zc/g);
        C=0.0;       
        S=0.0;       

         //両足支持期における補間関数の係数       
        a0={0.0, 0.0};
        a1={0.0, 0.0};
        a2={0.0, 0.0};
        a3={0.0, 0.0};
        a4={0.0, 0.0};


        //重み関数パラメータ
        D=0.0;       
        a=10;       
        b=1;

        stepcount=0;//定常2
        sup_dub=true;//
    
        t=0.0;
        tcount=0;
        dt=0.01*DTNUM;//*1.5;//*2;//10ms

        tofrom=0;
        Rstart=0;
        Lstart=0;

        Rx=0.0;
        Ry=0.0;
        Rz=0.008;//0.01;//0.002;//0.04;//4cm足を上げる
        w=0.0;
    };

    void walkingpatterngenerator::PatternPlanner(walkingparameters wp[])//まずは歩行パターンと歩数をあらかじめ与えたverを考える
    {
        C=cosh(wp[stepcount+1].Tsup/Tc);
        S=sinh(wp[stepcount+1].Tsup/Tc);
        D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

        wp[stepcount].Cpf = (wp[stepcount].Cpi-wp[stepcount].P)*cosh(wp[stepcount].Tsup/Tc)+Tc*wp[stepcount].Cvi*sinh(wp[stepcount].Tsup/Tc)+wp[stepcount].P;//n=0では修正着地位置=目標着地位置              
        wp[stepcount].Cvf = ((wp[stepcount].Cpi-wp[stepcount].P)/Tc)*sinh(wp[stepcount].Tsup/Tc)+wp[stepcount].Cvi*cosh(wp[stepcount].Tsup/Tc); 
        wp[stepcount].Caf = ((wp[stepcount].Cpi-wp[stepcount].P)/(Tc*Tc))*cosh(wp[stepcount].Tsup/Tc)+(wp[stepcount].Cvi/Tc)*sinh(wp[stepcount].Tsup/Tc);

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

    //両足支持期における補間関数の係数を算出
        wp[stepcount+1].Cai = ((wp[stepcount+1].Cpi-wp[stepcount+1].P)/(Tc*Tc));  //n+1歩目の初期重心加速度としている
       // cout<<wp[stepcount+1].Cai<<endl;

        if(wp[stepcount].Tdbl>=0.001)
        {
            a0=wp[stepcount].Cpf;
            a1=wp[stepcount].Cvf;
            a2=wp[stepcount].Caf/2;
            a3=-(2*wp[stepcount].Caf+wp[stepcount+1].Cai)/(3*wp[stepcount].Tdbl);
            a4=(wp[stepcount].Caf+wp[stepcount+1].Cai)/(4*wp[stepcount].Tdbl);
        }
        else
        {
            a0={0.0,0.0};
            a1={0.0,0.0};
            a2={0.0,0.0};
            a3={0.0,0.0};
            a4={0.0,0.0};
        }

        
        //両足指示期中の増加分の重心位置・接地位置
        wp[stepcount].dCp=a1*wp[stepcount].Tdbl+a2*wp[stepcount].Tdbl*wp[stepcount].Tdbl+a3*wp[stepcount].Tdbl*wp[stepcount].Tdbl*wp[stepcount].Tdbl+a4*wp[stepcount].Tdbl*wp[stepcount].Tdbl*wp[stepcount].Tdbl*wp[stepcount].Tdbl;
        cout<<wp[stepcount].dCp<<endl;

        Csum=Csum+wp[stepcount].dCp;
    };

    void walkingpatterngenerator::PatternGenerator(RobotLink link[],Robot &robot,RobotLink linkref[],walkingparameters wp[],Vector3d Prefr,Vector3d Prefl,int numsteps)
    {
        //右足をn=0とする

        if((stepcount==0)&&(tcount==0)&&(sup_dub==true))//n=1の着地位置を算出 定常歩行stepcount==2
        {
            wp[stepcount].P(0)=Prefr(0);
            wp[stepcount].P(1)=Prefr(1)-DWR;

            //Step5~8
            PatternPlanner(wp);
             cout<<"\n"<<stepcount<<"回目"<<endl;
             cout<<"\n"<<sign(stepcount)<<endl;

            t+=dt;
            tcount++;
            sup_dub=true;//片足支持

        }
        else if((tcount>=wp[stepcount].Tsup*(1/dt))&&(sup_dub==true))//両足支持に移行
        {
            if(wp[stepcount].Tdbl<=0.001)
            {
                cout<<"両足支持期無し"<<t<<endl;
                t=0.0;//リセット
                tcount=0;
                stepcount++;
                 //Step5~8
                PatternPlanner(wp);
                cout<<"\n"<<stepcount<<"回目"<<endl;
                
            }
            else
            {
                cout<<"両足支持期に移行"<<t<<endl;
                t=0.0;//リセット
                tcount=0;
                sup_dub=false;//両足支持期に移行

            }
            
        }
        else if((tcount>=wp[stepcount].Tdbl*(1/dt))&&(sup_dub==false))//片足支持に移行 //n+1 //支持脚切り替えの瞬間だけ計算すればよい
        {
            cout<<"片足支持期に移行"<<t<<endl;
            t=0.0;//リセット
            tcount=0;
            stepcount++;
            sup_dub=true;//片足支持に移行

            //Step5~8
            PatternPlanner(wp);
            cout<<"\n"<<stepcount<<"回目"<<endl;
        }
        else
        {
            t+=dt;
            tcount++;
        }

        if((sup_dub==true))//片足支持期
        {
            Cpref = (wp[stepcount].Cpi-wp[stepcount].P)*cosh(t/Tc)+Tc*wp[stepcount].Cvi*sinh(t/Tc)+wp[stepcount].P+(Csum-wp[stepcount].dCp);//n=0では修正着地位置=目標着地位置
            Cvref = ((wp[stepcount].Cpi-wp[stepcount].P)/Tc)*sinh(t/Tc)+wp[stepcount].Cvi*cosh(t/Tc);      
            Caref = ((wp[stepcount].Cpi-wp[stepcount].P)/(Tc*Tc))*cosh(t/Tc)+(wp[stepcount].Cvi/Tc)*sinh(t/Tc);  

            //Step3 重心軌道生成

                    //遊脚軌道生成
            if(sign(stepcount)==1)//右足が支持脚
            {
                w=(2*M_PI)/wp[stepcount].Tsup;

                if(stepcount==0)
                {   
                    // Rx=(wp[stepcount+1].P(0)-Prefl(0))/(2*M_PI);
                    // Ry=(wp[stepcount+1].P(1)-Prefl(1))/(2*M_PI);

                    // prefl(0)=Prefl(0)+Rx*(w*t-sin(w*t));//遊脚脚
                    // prefl(1)=Prefl(1)+Ry*(w*t-sin(w*t));
                    // prefl(2)=Rz*(1-cos(w*t));
                    
                    prefr(0)=wp[stepcount].P(0);//支持脚
                    prefr(1)=wp[stepcount].P(1);
                    prefr(2)=0.0;

                    prefl(0)=wp[stepcount+1].P(0)+wp[stepcount].dCp(0);//あらかじめ着地しておく
                    prefl(1)=wp[stepcount+1].P(1)+wp[stepcount].dCp(1);
                    prefl(2)=0.0;

                }
                else
                {

                    prefr(0)=wp[stepcount].P(0)+(Csum(0)-wp[stepcount].dCp(0));//支持脚
                    prefr(1)=wp[stepcount].P(1)+(Csum(1)-wp[stepcount].dCp(1));
                    prefr(2)=0.0;

                    Rx=(wp[stepcount+1].P(0)-(wp[stepcount-1].P(0)-wp[stepcount].dCp(0)-wp[stepcount-1].dCp(0)))/(2*M_PI);
                    Ry=(wp[stepcount+1].P(1)-(wp[stepcount-1].P(1)-wp[stepcount].dCp(1)-wp[stepcount-1].dCp(1)))/(2*M_PI);

                    prefl(0)=wp[stepcount-1].P(0)+(Csum(0)-wp[stepcount].dCp(0)-wp[stepcount-1].dCp(0))+Rx*(w*t-sin(w*t));//遊脚脚
                    prefl(1)=wp[stepcount-1].P(1)+(Csum(1)-wp[stepcount].dCp(1)-wp[stepcount-1].dCp(1))+Ry*(w*t-sin(w*t));  
                    //prefl(2)=Rz*(1-cos(w*t));
                    prefl(2)=wp[stepcount].Sz*(1-cos(w*t));
                }

            }
            else//左足が支持脚
            {

                w=(2*M_PI)/wp[stepcount].Tsup;
                
                prefl(0)=wp[stepcount].P(0)+(Csum(0)-wp[stepcount].dCp(0));//支持脚
                prefl(1)=wp[stepcount].P(1)+(Csum(1)-wp[stepcount].dCp(1));//支持脚
                prefl(2)=0.0;
                    
                Rx=(wp[stepcount+1].P(0)-(wp[stepcount-1].P(0)-wp[stepcount].dCp(0)-wp[stepcount-1].dCp(0)))/(2*M_PI);
                Ry=(wp[stepcount+1].P(1)-(wp[stepcount-1].P(1)-wp[stepcount].dCp(1)-wp[stepcount-1].dCp(1)))/(2*M_PI);
               
                prefr(0)=wp[stepcount-1].P(0)+(Csum(0)-wp[stepcount].dCp(0)-wp[stepcount-1].dCp(0))+Rx*(w*t-sin(w*t));//遊脚脚
                prefr(1)=wp[stepcount-1].P(1)+(Csum(1)-wp[stepcount].dCp(1)-wp[stepcount-1].dCp(1))+Ry*(w*t-sin(w*t));
                //prefr(2)=Rz*(1-cos(w*t));
                prefr(2)=wp[stepcount].Sz*(1-cos(w*t));

            }  
        }
        else//両足支持期
        {
            Cpref=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+(Csum-wp[stepcount].dCp);
            Cvref=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t;
            Caref=2*a2+6*a3*t+12*a4*t*t;

            if(sign(stepcount)==1)//前の支持脚が右
            {

                if(stepcount==0)
                {
                    prefr(0)=wp[stepcount].P(0);//支持脚
                    prefr(1)=wp[stepcount].P(1);
                    prefr(2)=0.0;

                    prefl(0)=wp[stepcount+1].P(0)+wp[stepcount].dCp(0);
                    prefl(1)=wp[stepcount+1].P(1)+wp[stepcount].dCp(1);
                    prefl(2)=0.0;

                }
                else
                {
                    prefr(0)=wp[stepcount].P(0)+(Csum(0)-wp[stepcount].dCp(0));//支持脚
                    prefr(1)=wp[stepcount].P(1)+(Csum(1)-wp[stepcount].dCp(1));
                    prefr(2)=0.0;

                    prefl(0)=wp[stepcount+1].P(0)+Csum(0);
                    prefl(1)=wp[stepcount+1].P(1)+Csum(1);
                    prefl(2)=0.0;

                }
            }
            else//前の支持脚が左
            {
                prefl(0)=wp[stepcount].P(0)+(Csum(0)-wp[stepcount].dCp(0));//支持脚
                prefl(1)=wp[stepcount].P(1)+(Csum(1)-wp[stepcount].dCp(1));//支持脚
                prefl(2)=0.0;

                prefr(0)=wp[stepcount+1].P(0)+Csum(0);
                prefr(1)=wp[stepcount+1].P(1)+Csum(1);
                prefr(2)=0.0;
            }
        }
        
        //link[0].p(0)=Cpref(0);
        //link[0].p(1)=Cpref(1);
        //link[0].p(2)=zc;

        robot.CoGref(0)=Cpref(0);
        robot.CoGref(1)=Cpref(1);
        robot.CoGref(2)=zc-DZC;
        link[0].p=robot.CoGref;

        link[0].v(0)=Cvref(0);
        link[0].v(1)=Cvref(1);
        link[0].v(2)=0.0;

        link[0].a(0)=Caref(0);
        link[0].a(1)=Caref(1);
        link[0].a(2)=0.0;


        linkref[0].p=prefr+dprefr;
        linkref[0].R=Rrefr;
        linkref[1].p=prefl+dprefl;
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