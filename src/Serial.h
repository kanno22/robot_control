#ifndef Serial
#define Serial

#include<iostream>
#include <sys/ioctl.h>
#include <termios.h>//cfsetispeed,
#include <unistd.h>
#include <fcntl.h>//open,

#define SERIAL_PORT "/dev/ttyS16"

class serial
{
    private:
        int fd;//ファイルディスクリプタ(ファイルの識別番号,失敗-1)
        unsigned char buf[255];//バッファ（一時的な記憶領域)
        struct termios tio;//シリアル通信設定
        int baudRate = B115200;;//一秒あたりのデータ数

    public:
        int buf_w;//送信時のバッファ
        int s_open();
        void s_read();
        int s_write();
        void s_close();
};

#endif