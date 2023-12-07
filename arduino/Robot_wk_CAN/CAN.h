#ifndef CAN_H
#define CAN_H

#include <mcp_can.h>
#include <SPI.h>
#include <Arduino.h>


class CAN
{
  private:
    char sbuf[60];
    int CanSend(int ID,char *Data,char len);//CAN Low Layer function
    int32_t SdoWriteObject(int NodeID, int Index, int SubIndex, int Value,int NumofByte);//Write Object Dictionary
    int32_t SdoReadObject(int NodeID,int Index, int SubIndex);//Read Object Dictionary
    int Disenable(int ID);//EPOSをDisenableにする．引数_Node ID　返り値_1:正常実行
    int Clearfault(int ID);//EPOSのfault stateをクリアする．引数_Node ID　返り値_1:正常実行
    int Enable(int ID);//EPOSをEnableにする．初期化処理もこれを使う．引数_Node ID　返り値_1:正常実行

  public:
    void CAN_init(int EposNodeID_R,int EposNodeID_L);
    
    //Maxon EposのApp関数
    int GetPosition(int ID);//現在の回転角 [qc]を取得．引数_Node ID　返り値_回転角 [qc]
    int GetVelocity(int ID);//現在の回転速度 [rpm]を取得．引数_Node ID　返り値_回転速度 [rpm]
    int GetVelocityEpos4(int ID);
    int GetCurrent(int ID);//現在の電流 [mA]を取得．引数_Node ID　返り値_電流 [mA]
    int GetTorqueEpos4(int ID);
    int QuickStop(int ID);//回転停止する．引数_Node ID　返り値_1:正常実行
    int IsTargetPosReach(int ID);//目標位置に到達しているか．引数_Node ID 返り値_1:到達している
    //Profile position mode////////////////////////////////////
    int SetPPM(int ID);//EposをProfile position modeに設定する．引数_Node ID 返り値_1:正常終了
    int SetTargetPosition(int ID,int Target);//Profile position modeの目標位置を設定する．引数_(Node ID，目標位置 [qc]) 返り値_1:正常終了
    int StartPositioningPPM(int ID,int Relative,int Immediate);//Profile position modeの制御開始する．引数_(Node ID, 1:現在位置から相対変位で目標値ぶんだけ回転 or 0:現在位置から絶対変位で目標値に移動,) 返り値_1:正常終了
    //Profile velocity mode///////////////////////////////////
    int SetPVM(int ID);////EposをProfile velocity modeに設定する．引数_Node ID 返り値_1:正常終了
    int SetTargetVelocity(int ID,int Target);//Profile velocity modeの目標値を設定する．引数_(Node ID，目標値 [rpm]) 返り値_1:正常終了
    int StartMovePVM(int ID);//Profile velocity modeの制御を開始する．引数_Node ID 返り値_1:正常終了
    //Current Mode//////////////////////////////////////////
    int SetCM(int ID);//EposをProfile current modeに設定する．引数_Node ID 返り値_1:正常終了
    int SetTargetCurrent(int ID,int Target);//Profile current modeの目標値を設定し，制御を!!開始!!する．引数_(Node ID, 目標値 [mA]) 返り値_1:正常終了
    int IsFault(int ID);//EposがFault state + 通信不能な状態 etcか確認する．人なら見ればいいけどマイコン用 引数_Node ID 返り値_0:Fault状態ではなく通信も正常．1:Falultなど正常でない状態全て

  
};

#endif
