1. Install Arduino + Teensyduino (https://www.pjrc.com/teensy/td_download.html)  
```
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz  
wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linux64  
wget https://www.pjrc.com/teensy/00-teensy.rules  
sudo cp 00-teensy.rules /etc/udev/rules.d/  
tar -xf arduino-1.8.13-linux64.tar.xz  
chmod 755 TeensyduinoInstall.linux64  
./TeensyduinoInstall.linux64 --dir=arduino-1.8.13  
```

2. Install rosserial  
```
sudo apt-get install ros-kinetic-rosserial-arduino  
sudo apt-get install ros-kinetic-rosserial
```  

3. Instal ros_lib  
```
cd <sketchbook>/libraries  
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
