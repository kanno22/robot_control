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

bool serial::s_open(const BaudRate &rate,const char *port) 
{
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == kError) {
    cout << "Device open error. device : \"" << port << "\""<< endl;
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

void serial::s_read(vector<float> &sense_data)
{
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);  // データを読み取る
        if (bytesRead > 0)
         {
            buffer[bytesRead] = '\0';  // 文字列の終端を設定
            dataBuffer += buffer;      // 受信したデータをバッファに追加

            // 改行で区切られた行ごとに処理する
            size_t pos;
            while ((pos = dataBuffer.find('\n')) != string::npos) 
            {
                string line = dataBuffer.substr(0, pos);
                dataBuffer.erase(0, pos + 1);

                // データを解析して浮動小数点数値に分割
                vector<float> sensorValues = parseData(line);

                // 解析されたセンサー値を出力
                cout << "Sensor values: ";
                for (float value : sensorValues) {
                    cout << value << " ";
                }
                cout << endl;
                sense_data=sensorValues;
            }
        }  
}

// 受信したデータをカンマで区切り、浮動小数点数値としてベクトルに変換する関数
vector<float> serial::parseData(const string& data) 
{
    vector<float> values;
    stringstream ss(data);
    string item;
    while (getline(ss, item, ',')) 
    {
        try 
        {
            values.push_back(stof(item));  // 文字列を浮動小数点に変換してベクトルに格納
        } 
        catch (const invalid_argument& e) 
        {
            cerr << "Invalid data: " << item << endl;
        }
    }
    return values;
}
void serial::s_close()
{
    close(fd);  
}