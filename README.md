二足歩行ロボットの実機を動かすプログラムです。

1.起動手順
P 最大 or 100000
D 9000 or 300
I 100


sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer

2.ダイナミクセルのシリアルポートが変わった場合
dxmisc.h 28行目
#define _COMPORT    "/dev/ttyUSB2" //に変更
#define _COMPORT    "/dev/ttyUSB3" //に変更

sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB2/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB2/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB2/latency_timer

sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB3/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB3/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB3/latency_timer