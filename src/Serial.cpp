#include "Serial.h"

using namespace std;

int serial::s_open()
{
    fd = open(SERIAL_PORT,O_RDWR);//デバイスファイルを開く O_RDWR 読み書き可能
    if(fd<0)
    {
        cout<<"open error\n"<<endl;
        return -1;
    }
    else
    {
        cout<<"Serial connected\n"<<endl;
    }

    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );
    cfmakeraw(&tio);                    // RAWモード
    tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う
    ioctl(fd, TCSETA, &tio);            // ポートの設定を有効にする

    return 0;
}

void serial::s_read()
{
    int len = read(fd, buf, sizeof(buf));//len 受信したバイト数

    if (0 < len)
    {
        for(int i = 0; i < len; i++) 
        {
            std::cout << buf[i];         
        }
    }
}

int serial::s_write()
{
    int len = write(fd,&buf_w,1);

    if(len<0)
    {
      //  cout<<"write error"<<endl;
        return -1;
    }

    return 0;
}

void serial::s_close()
{
    close(fd);  
}