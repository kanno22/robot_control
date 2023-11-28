#include "RSservo.h"

using namespace std;

Servo_data::Servo_data()
{
    ID=0;
    tq_mode=0;//トルクのon1 off0 
    Angle_ref=0;//目標角度 10=1.0deg
    dt_ref=0;//目標Δt 100=1s
}

int Servo_serial::rs_close()
{
    if( fd )
    {
		close( fd );
	}

	return 1;
}

int Servo_serial::rs_open()
{

	fd = open(SERVO_PORT, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0)
    {
        cout<<"RS open error\n"<<endl;
        return -1;
    }
    else
    {
        cout<<"RS connected\n"<<endl;
    }

	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

	newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	// clean the buffer and activate the settings for the port
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return fd;
}

int Servo_serial::rs_torque(Servo_data sr)
{
	// ハンドルチェック
	if( !fd )
    {
		return -1;
	}

	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)sr.ID;			// サーボID
	sendbuf[3]  = (unsigned char)0x00;				// フラグ
	sendbuf[4]  = (unsigned char)0x24;				// アドレス(0x24=36)
	sendbuf[5]  = (unsigned char)0x01;				// 長さ(4byte)
	sendbuf[6]  = (unsigned char)0x01;				// 個数
	sendbuf[7]  = (unsigned char)(sr.tq_mode&0x00FF);	// ON/OFFフラグ
	// チェックサムの計算
	sum = sendbuf[2];
	for(int i = 3; i < 8; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[8] = sum;								// チェックサム

	// 通信バッファクリア
	tcflush( fd, TCIFLUSH );

	// 送信
	ret = write( fd, &sendbuf, 9);

	return ret;
}

int Servo_serial::rs_move(Servo_data sr)
{

	// ハンドルチェック
	if( !fd ){
		return -1;
	}

	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)sr.ID;			    // サーボID
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ
	sendbuf[4]  = (unsigned char)0x1E;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04;				    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)0x01;				    // 個数
	sendbuf[7]  = (unsigned char)(sr.Angle_ref&0x00FF);		    // 位置
	sendbuf[8]  = (unsigned char)((sr.Angle_ref&0xFF00)>>8);	// 位置
	sendbuf[9]  = (unsigned char)(sr.dt_ref&0x00FF);	    // 時間
	sendbuf[10] = (unsigned char)((sr.dt_ref&0xFF00)>>8);	// 時間
	// チェックサムの計算
	sum = sendbuf[2];
	for(int i = 3; i < 11; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[11] = sum;								// チェックサム

	// 通信バッファクリア
	tcflush(fd, TCIFLUSH);

	// 送信
	ret = write( fd, &sendbuf, 12);

	return ret;
}
