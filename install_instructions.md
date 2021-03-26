### 1. Install Arduino + Teensyduino (https://www.pjrc.com/teensy/td_download.html)  
```
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz  
wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linux64  
wget https://www.pjrc.com/teensy/00-teensy.rules  
sudo cp 00-teensy.rules /etc/udev/rules.d/  
tar -xf arduino-1.8.13-linux64.tar.xz  
chmod 755 TeensyduinoInstall.linux64  
./TeensyduinoInstall.linux64 --dir=arduino-1.8.13  
```

### 2. Install rosserial  
```
sudo apt-get install ros-kinetic-rosserial-arduino  
sudo apt-get install ros-kinetic-rosserial
```  

### 3. Instal ros_lib  
```
cd <sketchbook>/libraries  
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
### 4. Upload firmware to Teensy (if the Teensy doesn't already have it installed already)  
Open `main.ino` with Arduino and press upload. It should compile and then open the Teensyduino uploader. There should a status bar as the firmware is written. Make sure the settings for the board match those below (the port will vary but it's usually /usb/ttyACM0 on linux machines).
<img width="826" alt="image" src="https://user-images.githubusercontent.com/32364356/112694118-2d1be080-8e58-11eb-87a6-f3feb7064fae.png">

### 5. Run the pass-through node.  
Start `roscore` then run the rosserial client application that forwards your Arduino messages to the rest of ROS. Make sure to use the correct serial port:
  ```
  rosrun rosserial_python serial_node.py /dev/ttyACM0
  ```
  Once this node is running all topics that the Teensy publishes to are available to ros. `rostopic list` should reveal /F1/cam_time, /F1/toggle, etc.
