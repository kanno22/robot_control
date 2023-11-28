#ifndef RSservo
#define RSservo

#include <stdio.h>	
#include<iostream>			
#include <string.h>
#include <termios.h> //Baudrate
#include <unistd.h> //read write
#include <fcntl.h> // O_RDWR|O_NOCTTY|O_NONBLOCK)

#define SERVO_PORT	"/dev/ttyS15"	// 通信ポートの指定

class Servo_data
{
    public:
        Servo_data();
        int ID;
        short tq_mode;//トルクのon1 off0 
        short Angle_ref;//目標角度 10=1.0deg
        unsigned short dt_ref;//目標Δt 100=1s

};

class Servo_serial
{
    private:
        int fd;
        struct termios newtio;
        unsigned char sendbuf[28];
        unsigned char sum;
        int ret;

    public:
        int rs_close();
        int rs_open();
        int rs_torque(Servo_data sr);
        int rs_move(Servo_data sr);
};


#endif