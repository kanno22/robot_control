#include"CAN.h"

MCP_CAN CAN0(10);     // SSピンをD10に設定

void CAN::CAN_init(int EposNodeID)
{    
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
/////////////////////右足
  while( Enable(EposNodeID) !=1)
  {
    sprintf(sbuf, "Node%d 初期化\r\n",EposNodeID);
    Serial.println(sbuf);    
  }

  sprintf(sbuf, "Disenable %d:%d",EposNodeID, Disenable(EposNodeID) );
  Serial.println(sbuf);
  
  sprintf(sbuf, "Clearfault %d:%d",EposNodeID,  Clearfault(EposNodeID) );
  Serial.println(sbuf);
  
  sprintf(sbuf, "Enable %d:%d",EposNodeID,  Enable(EposNodeID) );
  Serial.println(sbuf);

  Serial.println("EPOS_R 初期化完了");
}

//CAN Low Layer function
int CAN::CanSend(int ID,char *Data,char len) 
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(ID, 0, len, Data);
  if(sndStat == CAN_OK)
  {
    return 1;
  } 
  else 
  {
    Serial.println("Error Sending Message...");
    return 0;
  }
}
//Write And Read Object Dictionary
int32_t CAN::SdoWriteObject(int NodeID, int Index, int SubIndex, int Value,int NumofByte)
{
    int ID=0;
    byte Data[8]={0,0,0,0,0,0,0,0};
    int len=8;
    int retry=0;
    int FunctionCode = 0b1111;//Fuction codeとして定義されていない値を使って初期化
    int ResponseNodeID = 129;//NodeID <127に含まれない値で初期化
    int32_t ErrorCode = 1;//Eposからの応答の(int32_t)Dataはerror codeになっている．

    ID=0x600+NodeID;
    
    switch(NumofByte)
    {
        case 1:
            Data[0]=0x2F;
            break;
        case 2:
            Data[0]=0x2B;
            break;
        case 4:
            Data[0]=0x23;
            break;
        default:
            Data[0]=0x22;
     }
     
    Data[1]=Index & 0x00FF;
    Data[2]=Index >> 8;
    Data[3]=SubIndex;
    Data[4]=(Value & 0xFF);
    Data[5]=(Value & 0xFF00)>>8;
    Data[6]=(Value & 0xFF0000)>>16;
    Data[7]=(Value & 0xFF000000)>>24;
    
    if(CanSend(ID, Data, len) !=1 )
    {    
      Serial.println("CanSendFailure");       
    }
    else
    {
      unsigned long RxId;
      byte RxLen;
      byte RxBuf[8];
            
      while(CAN0.checkReceive() != CAN_MSGAVAIL)
      {//CAN受信
        retry++;
        if(retry>100000)
        {
          Serial.println("SdoWrite No Reply\r\n");           }
        }
            
          //CAN受信
        CAN0.readMsgBuf(&RxId, &RxLen, RxBuf);
        //受信のCOB-IDの上位4bit:Function codeからSDO, PDOとかを識別
        FunctionCode = (RxId & 0b11110000000) >>7;
            
        if( FunctionCode == 0b0000 )
        {// NMT
          // NMTの詳細は勉強してない
          Serial.println("error FC:NMT");
                
        }
        else if( FunctionCode == 0b0001 )
        {// SYNC
          // SYNCの詳細は勉強してない
          //EMERGENCY
          ResponseNodeID = RxId -128;
          Serial.println("EMERGENCY error from");
          Serial.print(ResponseNodeID, DEC);
        }
        else if( FunctionCode == 0b0011 )
        {//PDO1 (tx) Epos -> Micon
          Serial.println("error FC:PDO1");  
        }
        else if( FunctionCode == 0b0101 )
        {//PDO2 (tx) Epos -> Micon
          Serial.println("error FC:PDO2");
        }
        else if( FunctionCode == 0b0111 )
        {//PDO3 (tx) Epos -> Micon
          Serial.println("error FC:PDO3");            
        }
        else if( FunctionCode == 0b1001 )
        {//PDO4 (tx) Epos -> Micon
          Serial.println("error FC:PDO4");
        }
        else if( FunctionCode == 0b1011 )
        {//SDO1 (tx) 0x580  Epos -> Micon
          if( NodeID != (RxId - 0x580 ))
          {
            Serial.println("SdoWrite Reply form:%");
            Serial.print(RxId - 0x580 ,DEC);       
           }
           else
           {//正常にSDOが帰ってきた
            ErrorCode=(int32_t)((RxBuf[7]<<24) + (RxBuf[6]<<16) + (RxBuf[5]<<8) + RxBuf[4]); 
            return ErrorCode;//正常終了ならerrocodeは0のはず
           } 
             
         }
         else
         {
          Serial.println("throw if else\r\n");
         }  
         
      ErrorCode=(int32_t)((RxBuf[7]<<24) + (RxBuf[6]<<16) + (RxBuf[5]<<8) + RxBuf[4]); 
      //Serial.println("ErrorCode:%x\r\n",ErrorCode);
      return ErrorCode;//正常終了ならerrocodeは0のはず
    }   
    
  return 0b1111;//いるのかこれ？210316 Canopenの返り値0はNo errorなのでべつので        
}

int32_t CAN::SdoReadObject(int NodeID,int Index, int SubIndex)
{
    int value=0;
    int ID=0;
    byte Data[8]={0,0,0,0,0,0,0,0};
    int len=8;
    int retry=0;
    int FunctionCode = 0b1111;//Fuction codeとして定義されていない値を使って初期化
    int ResponseNodeID = 129;//NodeID <127に含まれない値で初期化
    int32_t ErrorCode = 1;//Eposからの応答の(int32_t)Dataはerror codeになっている．
        
    ID=0x600+NodeID;
    Data[0]=0x40;
    Data[1]=Index & 0x00FF;
    Data[2]=Index >> 8;
    Data[3]=SubIndex;
    
    if(CanSend(ID, Data, len) !=1 )
    {    
      Serial.println("CanSendFailure\r\n");         
    }
    else
    {
      unsigned long RxId;
      byte RxLen;
      byte RxBuf[8];
            
      while(CAN0.checkReceive() != CAN_MSGAVAIL)
      {//CAN受信
        retry++;
        if(retry>100000)
        {
          Serial.println("SdoWrite No Reply\r\n");
        }
      }
            
      //CAN受信
      CAN0.readMsgBuf(&RxId, &RxLen, RxBuf);
      //受信のCOB-IDの上位4bit:Function codeからSDO, PDOとかを識別
      FunctionCode = (RxId & 0b11110000000) >>7;
            
      if( FunctionCode == 0b0000 )
      {// NMT
        Serial.println("error FC:NMT");
      }
      else if( FunctionCode == 0b0001 )
      {// SYNC
        //EMERGENCY
        ResponseNodeID = RxId -128;
      }
      else if( FunctionCode == 0b0011 )
      {//PDO1 (tx) Epos -> Micon
        Serial.println("error FC:PDO1");    
      }
      else if( FunctionCode == 0b0101 )
      {//PDO2 (tx) Epos -> Micon
        Serial.println("error FC:PDO2");
      }
      else if( FunctionCode == 0b0111 )
      {//PDO3 (tx) Epos -> Micon
        Serial.println("error FC:PDO3");
      }
      else if( FunctionCode == 0b1001 )
      {//PDO4 (tx) Epos -> Micon
        Serial.println("error FC:PDO4");
      }
      else if( FunctionCode == 0b1011 )
      {//SDO1 (tx) 0x580  Epos -> Micon
        if( NodeID != (RxId - 0x580 ))
        {}
        else
        {
          //正常にSDOを受信した
          value=(int32_t)((RxBuf[7]<<24) + (RxBuf[6]<<16) + (RxBuf[5]<<8) + RxBuf[4]); 
          //Serial.println("value:%x\r\n",value);
         return value;           
        }
      }
      else
      {}
            
      //正常にSDO受信しなかった場合ではエラーコードのはず
      ErrorCode=(int32_t)((RxBuf[7]<<24) + (RxBuf[6]<<16) + (RxBuf[5]<<8) + RxBuf[4]); 
      // Serial.println("ErrorCode:%x\r\n",ErrorCode);
       
     } 
      
     return 0b1010;//Canopenの返り値0はNo errorなのでべつので
    
}

int CAN::Disenable(int ID)//EPOSをDisenableにする．引数_Node ID　返り値_1:正常実行
{   
    if(SdoWriteObject(ID,0x6040 ,0x00,0x06,2) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int CAN::Clearfault(int ID)//EPOSのfault stateをクリアする．引数_Node ID　返り値_1:正常実行
{   
    if(SdoWriteObject(ID,0x6040 ,0x00,0x080,2) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    } 
}   

int CAN::Enable(int ID)//EPOSをEnableにする．初期化処理もこれを使う．引数_Node ID　返り値_1:正常実行
{
    if( SdoWriteObject(ID,0x6040 ,0x00,0x0F,2) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int CAN::GetPosition(int ID)
{//現在の回転角 [qc]を取得．引数_Node ID　返り値_回転角 [qc]
  return SdoReadObject(ID,0x6064,0x00);
}

int CAN::GetVelocity(int ID)
{//現在の回転速度 [rpm]を取得．引数_Node ID　返り値_回転速度 [rpm]
  return SdoReadObject(ID,0x2028,0x00);//EPOS2ではこちらが速度のオブジェクト[rpm]だった
}

int CAN::GetVelocityEpos4(int ID)
{//現在の回転速度 [rpm]を取得．引数_Node ID　返り値_回転速度 [rpm]
  return SdoReadObject(ID,0x30D3,0x01);//EPOS4ではこちらが速度のオブジェクト[rpm]になってる．．．
}
    

int CAN::GetCurrent(int ID)
{//現在の電流 [mA]を取得．引数_Node ID　返り値_電流 [mA]
  return SdoReadObject(ID,0x6078,0x00);
}

    
int CAN::GetTorqueEpos4(int ID)
{//現在のトルク [per thousand of “Motor rated torque”=ようは定格トルクの千倍値?]を取得．引数_Node ID　返り値_トルク[Nm*1000?]
  return SdoReadObject(ID,0x6077,0x00);
}
    
int CAN::QuickStop(int ID)
{//回転停止する．引数_Node ID　返り値_1:正常実行
    if( SdoWriteObject(ID,0x6040,0x00,0x02,2) == 0)
    {
      return 1;
    }
    else
    {
        printf("QuickStop SdoWriteObject error\r\n");
        return 0;
    }
}

    
int CAN::IsTargetPosReach(int ID)
{//目標位置に到達しているか．引数_Node ID 返り値_1:到達している
  return (SdoReadObject(ID,0x6041,0x00)&0b0000010000000000)>>10;
}

//Profile position mode////////////////////////////////////
int CAN::SetPPM(int ID)
{//EposをProfile position modeに設定する．引数_Node ID 返り値_1:正常終了
    
    if(SdoWriteObject(ID,0x6060 ,0x00,0x1,1)==0)
    {//profile position modeに設定
      //設定成功
      if(SdoReadObject(ID,0x6060,0x00)==1)
      {//設定されたか確認
        return 1;
      }
      else
      {
        printf("SetPPM SdoReadObject error\r\n");
        return 0;
      }     
    }
    else
    {
      printf("SetPPM SdoWriteObject error\r\n");
      return 0;
    }
}

int CAN::SetTargetPosition(int ID,int Target)
{//Profile position modeの目標位置を設定する．引数_(Node ID，目標位置 [qc]) 返り値_1:正常終了
    
  if( SdoWriteObject(ID,0x6040 ,0x00,0x000F,2)  == 0)
  {
        
  }
  else
  {
    printf("toggle “New Position error\r\n");
  }
    
  if( SdoWriteObject(ID,0x607A ,0x00,Target,4)==0 )
  {//set target position
    //設定成功 
    return 1;       
  }
  else
  {
    printf("SetTargetPosition SdoWriteObject error\r\n");
    return 0;
  }
    
}
    
int CAN::StartPositioningPPM(int ID,int Relative,int Immediate)
{//Profile position modeの制御開始する．引数_(Node ID, 1:現在位置から相対変位で目標値ぶんだけ回転 or 0:現在位置から絶対変位で目標値に移動,) 返り値_1:正常終了
  int32_t ErrorCode = 0;
  ErrorCode = SdoWriteObject(ID,0x6040 ,0x00,(((Relative<<2)+(Immediate<<1)+1)<<4)+0x0F,2);
  
    if( ErrorCode==0 )
    {
      //設定成功 
      return 1;       
    }
    else
    {
      printf("StartPositioningPPM SdoWriteObject error\r\n");
      printf("ErrorCode:%x\r\n",ErrorCode);
      return 0;
    }

}

//Profile velocity mode///////////////////////////////////
int CAN::SetPVM(int ID)
{////EposをProfile velocity modeに設定する．引数_Node ID 返り値_1:正常終了
    
  if(SdoWriteObject(ID,0x6060 ,0x00,0x03,1)==0)
  {//modeを設定定
    //設定成功
    if(SdoReadObject(ID,0x6060,0x00)==0x03)
    {//設定されたか確認
      return 1;
    }
    else
    {
      printf("SetPVM SdoReadObject error\r\n");
      return 0;
    }
        
  }
  else
  {
   printf("SetPVM SdoWriteObject error\r\n");
   return 0;
  }

}

int CAN::SetTargetVelocity(int ID,int Target)
{//Profile velocity modeの目標値を設定する．引数_(Node ID，目標値 [rpm]) 返り値_1:正常終了
  if(SdoWriteObject(ID,0x60FF ,0x00,(Target),4) == 0)
  {
    return 1;
  }
  else
  {
    printf("SetTargetVelocity SdoWriteObject error\r\n");
    return 0;
  }
}
    
int CAN::StartMovePVM(int ID)
{//Profile velocity modeの制御を開始する．引数_Node ID 返り値_1:正常終了
  if( SdoWriteObject(ID,0x6040 ,0x00,0x000F,2)  == 0)
  {
    return 1;
  }
  else
  {
    printf("StartMovePVM SdoWriteObject error\r\n");
    return 0;
  }
}    
    
//Current Mode//////////////////////////////////////////
int CAN::SetCM(int ID)
{//EposをProfile current modeに設定する．引数_Node ID 返り値_1:正常終了

  if(SdoWriteObject(ID,0x6060 ,0x00,0xFD,1)==0)
  {//modeを設定定
    //設定成功
    if(SdoReadObject(ID,0x6060,0x00)==0xFD)
    {//設定されたか確認
      return 1;
    }
    else
    {
      printf("SetCM SdoReadObject error\r\n");
      return 0;
    }
        
   }
   else
   {
    printf("SetCM SdoWriteObject error\r\n");
    return 0;
   }
}

int CAN::SetTargetCurrent(int ID,int Target)
{//Profile current modeの目標値を設定し，制御を!!開始!!する．引数_(Node ID, 目標値 [mA]) 返り値_1:正常終了
  if( SdoWriteObject(ID,0x2030 ,0x00,(Target),2) == 0)
  {//目標値設定後，即制御開始
    return 1;
  }else
  {
    printf("SetTargetCurrent SdoWriteObject error\r\n");
    return 0;
  }
}

int CAN::IsFault(int ID)
{//EposがFault state + 通信不能な状態 etcか確認する．人なら見ればいいけどマイコン用 引数_Node ID 返り値_0:Fault状態ではなく通信も正常．1:Falultなど正常でない状態全て

  uint16_t statusword = 0b1000;
  statusword = SdoReadObject(ID,0x6041,0x00);

  if( ((statusword >>3) & 0b0000000000000001) == 0 )
  {
    return 0;//(statusword >>3) & 0b0000000000000001
  }
  else
  {
    return 1;
  }
}
