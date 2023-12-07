#include "Serial.h"

using namespace std;

const int serial::kError = -1;

serial::serial() 
{
  fd=kError;
}

serial::~serial() 
{
  if (fd != kError) {
    tcsetattr(fd, TCSANOW, &old_settings_);
    close(fd);
  }
}

bool serial::s_open(const BaudRate &rate) 
{
  fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == kError) {
    cout << "Device open error. device : \"" << SERIAL_PORT << "\""<< endl;
    return false;
  }

  if (tcgetattr(fd, &old_settings_) == kError) 
  {
    cout << "tcgetattr error."<< endl;
    close(fd);
    return false;
  }

  current_settings_ = old_settings_;

  if ((cfsetispeed(&current_settings_, rate) == kError) || (cfsetospeed(&current_settings_, rate) == kError)) 
  {
    cout << "cfsetispeed or cfsetospeed error."<< endl;
    close(fd);
    return false;
  }

  current_settings_.c_iflag = IGNPAR;
  current_settings_.c_oflag = 0;
  current_settings_.c_lflag = 0;
  current_settings_.c_cflag= (CS8 | CLOCAL | CREAD);

  if (tcsetattr(fd, TCSANOW, &current_settings_) == kError) 
  {
    cout << "tcsetattr error."<< endl;
    close(fd);
    return false;
  }

  return true;
}

bool serial::s_write(const string &str) 
{
  size_t send_size = str.size() + 1;
  cout<<"send_size="<<str.size()<<endl;

  char send_char[send_size];

  str.copy(send_char, str.size());

  send_char[str.size()] = '\0';

  return write(fd, send_char, send_size) == send_size;
}

string serial::s_read(const bool wait, const char terminate) 
{
  string receive_str;
  bool receving = false;
  char receive_char;

  while (true) 
  {
    int read_size = read(fd, &receive_char, 1);

    if (read_size > 0) 
    {
      receving = true;
      receive_str.append(1, receive_char);
      if (receive_char == terminate) 
      {
        break;
      }
    } 
    else 
    {
      if (!wait || receving) 
      {
        break;
      }
    }
  }
  return receive_str;
}

void serial::s_close()
{
    close(fd);  
}