# ig_handle

This package contains all code related to ig_handle.

## Overview

"ig_handle" describes our current data collection platform.  The hardware includes a SLAM sensor kit, a Teensy 3.6 microcontroller, a USB hub, a network switch, and an Ubuntu 18 computer.  The sensor kit is mounted on an aluminum post and the rest of the hardware is contained in a box for portability.  The kit can be used with or without a robot and with or without additional sensors.

## Launch Files

Make sure you have a bag directory located at ~/bags before using the launch files.

### Raw data

Use collect_raw_data.launch to collect data (sensor messages) + time stamps (TimeReference messages, if applicable) for the core SLAM sensor kit.  This includes cameras F1, F2, and F3, lidar_h, and imu.

```
roslaunch ig_handle collect_raw_data.launch
```

This launch file will start the handle and record a bag file. Bags will be recorded in a timestamped folder as "raw.bag". Use "output" arg to specify the parent directory for the timestamped folder (default is ~/bags). For example:

```
roslaunch ig_handle collect_raw_data.launch output:=~/my_folder
```

This command will record data to "~/my_folder/2021_08_09_07_01pm/raw.bag" for example.

### Using robots

For each robot that ig_handle has been implemented on, there is a launch file that starts the handle plus the additional sensors.  These launch files are invoked with a launch argument called "robot" (i.e. robot:=husky).

The Heron adds the DT100 sonar, camera F4, and lidar_v.  For example:

```
roslaunch ig_handle collect_raw_data.launch robot:=heron
```

The Husky adds camera F4, lidar_v, and the Husky base and control packages.  The thermal camera is in there, but it is not tested yet.  For example:

```
roslaunch ig_handle collect_raw_data.launch robot:=husky
```

## Data

In order to save data, collect_raw_data.launch calls record_bag.sh found in ig_handle/scripts/.  By default, image_raw is recorded for each camera.  For lidars, velodyne_points and velodyne_packets are recorded.  Any changes desired can be made in record_bag.sh.

### Processing

Processing scripts for ig_handle data are stored in ig_handle/scripts/processing.  The python requirements for these scripts are stored in ig_handle/requirements.txt.  The pip install can be automated using install.bash found in ig_handle/scripts/.

process_raw_bag.py is used to combine the TimeReference and sensor messages collected during data capture.  The script replaces the sensor messages' default timestamps with the TimeReference timestamps using a simple first in first out queue at the moment.  This scripts also reserializes every message to its header timestamp (see additional notes for more info).  The interface for the script is best explained by reading the argsparse help messages.  The bagfile argument (-b) needs to be set every time to find the input bag.  The values for data and time topics (-d, -t) are set correctly by default, so only specify those arguments if you've changed the data collection process.

stamp_from_serialization.py can be used on lidar data where the PPS/$GPRMC synchronization method failed.  If PPS/$GPRMC synchronization did not fail, do not use this script.  When it fails, the lidar data's header timestamps will not make sense, i.e. they will not start at 0 (no synchronization at all), and they will not match the rest of the bag (synchronized to the ROS clock). In this case, lidar message serialization time can be used since ethernet can convey the small messages very quickly.   Use this script before process_raw_bag.py.

If velodyne scans are recorded rather than the velodyne points and packets then unpacking them is the first step to perform:

```./path_to_catkin_ws/build/rosbag_tools/unpack_velodyne_scans_main -bag_file_path /path_to_bag/raw.bag  -aggregate_packets=true```

The following two steps are to take the raw.bag to a bag ready for SLAM:
```
python2 process_raw_bag.py -b [path_to_raw_bag] -d [list_of_data_topics] -t [list_of_time_reference_topics_respectively]
```

```
./debayer_downsample.sh [path_to_output_bag] [raw_image_topic] [list_of_other_topics]
```

## Documentation

### Networking

Everything is currently configured to expecting a 192.168.1.XXX subnet (255.255.255.0 aka /24 mask).  The lidars are currently configured as 192.168.1.201 (horizontal) and 192.168.1.202 (vertical).  The network switch inside of the handle box connects the lidars, the handle computer, and the outboard ethernet ports mounted on the handle box.  If you are using this switch alone, it is important to set all computers' IPs and netmasks statically.

The handle's ROS computer is usually statically assigned 192.168.1.150.  In order to have this set automatically at every startup, copy 01-ig2_netplan.yaml from ig_handle/config/ to /etc/netplan/.  Next, replace "enp0s31f6" with the name of the ethernet adapter that you are using.  You can find the name (eth0, enp0s1, etc) with the command ifconfig (may have to install).  If you connect the handle to an internet connected router using one of the outboard ethernet ports, the handle computer will gain internet access for cloning and installing software.  You will also be able to network with the computer and lidars from any computer on the router.  A router can be used without internet connection just to make it easier to connect to the handle in the field.  Make sure any router is set for the 192.168.1.XXX IP range.

In order to participate with the ROS core running on the handle's ROS computer from your laptop over networking, you need to set a couple environment variables to tell your laptop the ROS core is elsewhere on the network.  A script called remote-ig.sh in ig_handle/config/ sets this for you.  You can put the following line in your ~/.bashrc file to automate it, or execute it manually:

```
source ~/catkin_ws/src/ig_handle/config/remote-ig2.sh
```

### Microcontroller

The microcontroller is a Teensy 3.6, which is very similar to an Arduino with more GPIO and power.  The Teensy GPIO is used for sending trigger signals to cameras, receiving camera exposure indication signals, sending an imu start signal, receiving imu measurement indications, sending Lidar PPS (1hz synchronization pulse), and Lidar $GPRMC serial signals (simulating GPS time/location messages).  The Teensy is communicating with the handle's ROS computer using a rosserial node.  The rosserial node gives the arduino code a nodehandle to subscribe/publish messages and access synchronized ROS time.  When the Teensy receives camera/imu measurement indications, it publishes a TimeReference message.  The Lidar messages can be verified by visiting the lidars' online configuration pages, and making sure the PPS is marked as "locked" and the GPS time is being shown and looks accurate (location is irrelevant).

### Udev

Udev rules are used in Ubuntu to create custom USB configurations when USB devices are plugged in.  For example, in order to ensure the Teensy and IMU ports are always known, Udev rules are used to create aliases when these devices are plugged in.  Copy 99-ig2_udev.rules from ig_handle/config/ to /etc/udev/rules.d/ in order to implement the custom rules.  The rules scan for devices plugged in with the correct idVendor and idProduct, and execute actions when there is a match.  Make sure your user is in the "dialout" group (should be by default) for the rules to work.  The teensy is given a symlink found at /dev/teensy, the imu is given /dev/imu, and the Husky robot connection is given /dev/prolific.

### Advanced launching

The package has a launch file for each sensor, which are included or not depending on the robot.  collect_raw_data launches handle_includes (no robot arg), heron_includes (robot:=heron), or husky_includes (robot:=husky).  handle_includes has the core sensor kit launch files, then heron_includes or husky_includes have handle_includes plus robot specific stuff.  If you want to customize your launch for a given application, you can comment things out.

Terminator layouts can be useful for quickly pulling up a complicated nested terminal layout with preset commands, such as viewing all the sensor rostopic publishing frequencies.  The terminator documenation provides more info, but be warned its finicky.  One useful terminator layout is provided that launches collect_raw_data and displays many rostopic publishing frequencies.  In order to use it, copy config from /ig_handle/config/ to ~/.config/terminator/ and run:

```
terminator -l ig_handle
```

### Additonal notes

Synchronization confusion can result from the difference between serialization time and header timestamps.  

Serialization time:

ROS bags record the time that messages are serialized (the moment the message was saved).  This time is used by rqt_bag to show the timeline of messages.  This time is also the third argument returned from iterating bag.read_messages() in python.  Finally, this time indicates when the message is published when playing back the bag.

Timestamp:

Every ROS sensor message has a header (sensor topic + /header) with a stamp field.  This timestamp is part of the message and has nothing to do with data transport times.  For example, the lidar driver sets this field using the GPS time data forwarded by the lidar.

For synchronization, the header timestamps are the important ones, since the time the message save is affected by buffering and bandwidth limitations.  So, running process_raw_bag.py resets all serialization times to match the header timestamps.  Confusion can arise viewing rqt_bag prior to processing the data, when the serialization times have not been reset yet.

1. ./debayer_downsample.sh [path_to_output_bag] [raw_image_topic] [list_of_other_topics]