#ifndef Serial
#define Serial

#include <fcntl.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <vector>
#include <sstream>

#define SERIAL_PORT "/dev/ttyACM0"
#define SERIAL_PORT_2 "/dev/ttyACM1"
#define SERIAL_PORT_S "/dev/ttyUSB1"


using namespace std;

class serial
{
    private:
        static const int kError;
        termios old_settings_;
        termios current_settings_;
        int fd;

        char buffer[256];
        string dataBuffer = "";


    public:
        enum BaudRate 
        {
            kB4800 = B4800,
            kB9600 = B9600,
            kB19200 = B19200,
            kB38400 = B38400,
            kB115200 = B115200           
        };

        serial();
        virtual ~serial();
        bool s_open(const BaudRate &baudrate,const char *port);
        bool s_write(const string &str);
        void s_read(vector<float> &sense_data);
        vector<float> parseData(const string& data);
        void s_close();
};

#endif