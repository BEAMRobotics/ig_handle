# ig-handle: a handheld inspector gadget

**ig-handle** is an open-source, hardware-level time-synchronized handheld LiDAR-visual-inertial sensor kit consisting of:
 - One [Velodyne Puck](https://velodyneLiDAR.com/products/puck/) LiDAR
 - Two [FLIR Blackfly S USB3](https://www.flir.com/products/blackfly-s-usb3/?model=BFS-U3-13Y3M-C&vertical=machine+vision&segment=iis) monochrome cameras
 - One [Xsens MTi-30 AHRS](https://www.xsens.com/hubfs/Downloads/usermanual/MTi_usermanual.pdf) IMU

All sensors are synchronized with the Real Time Clock (RTC) of a Teensy 4.1 microcontroller, which provides:
  - PPS/GPRMC time-synchronization for the LiDAR. See Section 7.4 of the following [manual](https://drive.google.com/file/d/1aXXFh7Xt5NxyyPRi7TeC-lx5p7oeyu10/view?usp=sharing) for details. The LiDAR collects data at 10 Hz
  - analog signals to start and simultaneously trigger cameras at 20 Hz, enabling stereo vision
  - digital signals to start and sample IMU data at 200 Hz

**ig-handle** is extensible to additional LiDARs and cameras, with the option to soft-synchronize sonar data collected by a [DT100 multibeam profiling sonar](https://imagenex.com/products/dt100) as demonstrated in our paper:

```
@article{thoms2023tightly,
  title={Tightly Coupled, Graph-Based DVL/IMU Fusion and Decoupled Mapping for SLAM-Centric Maritime Infrastructure Inspection},
  author={Thoms, Alexander and Earle, Gabriel and Charron, Nicholas and Narasimhan, Sriram},
  journal={IEEE Journal of Oceanic Engineering},
  year={2023},
  publisher={IEEE}
}
```

If you are interested in building your own **ig-handle**, the latest parts list, CAD models, electrical schematics, and build instructions can be found [here](https://drive.google.com/drive/folders/1DrAMQ9eQS1JjoDI4LWuoENaN7cZ9nRTC?usp=sharing). Note that our build uses the following [Intel NUC computer kit](https://drive.google.com/file/d/1mJj0qhpS1F2KvkGdUfzvh3qHi5qF908q/view?usp=sharing) with an 11th Gen Intel® CoreTM i7-1165G7 processor and 8GB of DDR4 RAM.


## Installation

For installation, please refer to the [Beam Installation Guide](https://github.com/BEAMRobotics/beam_robotics/wiki/Beam-Robotics-Installation-Guide). This guide covers the installation of dependencies (as required by **ig-handle** and supported robots **pierre** and **ig2**) on a clean Ubuntu 20.04 machine. We recommend setting your catkin workspace to the default directory `~/catkin_ws` as the commands documented in this README follow this convention.
## Launch Files

Before using the launch files, create a bag directory via:
```
mkdir -p bags
```

### Collect Raw Data

**ig-handle** data is collected and saved to a rosbag via:

```
roslaunch ig_handle collect_raw_data.launch
```

By default, rosbags are recorded in a timestamped folder as `raw.bag`. Use the `output` arg to specify an alternative parent directory for the timestamped folder. For example:

```
roslaunch ig_handle collect_raw_data.launch output:=~/my_folder
```

This command will record data to `~/my_folder/YYYY_MM_DD_HH_MM_SS/raw.bag`. Note that, when powered on, the LiDAR takes approximately 25-30 seconds to connect over LAN. Given this, wait 30 seconds after **ig-handle** is powered to collect raw data.

### Using robots

For each robot that integrates **ig-handle**, there is a launch file that starts data collection for **ig-handle** plus additional sensors (see Section **Data** for details). These launch files are invoked with a launch argument called `robot`:

**pierre** adds cameras `F3` and `F4`, LiDAR `lidar_v`, and the DT100 sonar. Data is collected via:

```
roslaunch ig_handle collect_raw_data.launch robot:=heron
```

**ig2** adds cameras `F3` and `F4`, LiDAR `lidar_v`, and the Husky base and control packages. A [FLIR Boson Plus 640](https://www.flir.com/products/boson-plus/?model=22640A012&vertical=lwir&segment=oem) thermal camera is included, though is not tested. Data is collected via:

```
roslaunch ig_handle collect_raw_data.launch robot:=husky
```

## Data

In order to collect data, `collect_raw_data.launch` calls `record_bag.sh` found in `ig_handle/scripts/` and records topics specific to each robot.

For **ig-handle**, the following topics are recorded:
| Topic                     | message types               |
| ------------------------- | --------------------------- |
| /F1/image_raw/compressed  | sensor_msgs/CompressedImage |
| /F2/image_raw/compressed  | sensor_msgs/CompressedImage |
| /cam/time                 | sensor_msgs/TimeReference   |
| /imu/data                 | sensor_msgs/Imu             |
| /imu/time                 | sensor_msgs/TimeReference   |
| /lidar_h/velodyne_packets | velodyne_msgs/VelodyneScan  |
| /lidar_h/velodyne_points  | sensor_msgs/PointCloud2     |

For **pierre** and **ig2**, the following additional topics are recorded:
| Topic                     | message types               |
| ------------------------- | --------------------------- |
| /F3/image_raw/compressed  | sensor_msgs/CompressedImage |
| /F4/image_raw/compressed  | sensor_msgs/CompressedImage |
| /lidar_v/velodyne_packets | velodyne_msgs/VelodyneScan  |
| /lidar_v/velodyne_points  | sensor_msgs/PointCloud2     |

Further, **pierre** records `/DT100/sonar_scans` topics of message type `sensor_msgs/PointCloud2`, while **ig2** records `/thermal/image_raw/compressed` topics of message type `sensor_msgs/CompressedImage`. If additional topics are desired, `record_bag.sh` can be modified accordingly.

### Processing

Raw data is processed using `process_raw_bag.py` found in `ig_handle/scripts/`. Its description and interface follows.

**Description**: This script restamps camera and IMU sensor messages with their appropriate time reference messages as per the RTC clock of the microcontroller. Acknowledging camera and IMU sensor messages (i.e. `sensor_msgs/CompressedImage` and `sensor_msgs/Imu`) take longer to serialize than time reference messages (i.e. `/cam/time` and `/imu/time`), the script discards camera and IMU sensor messages before the first time reference (based on serialized time) and then proceeds to restamp sensor messages with time references using a first-in-first-out queue. In testing, we observe no camera and IMU signal dropout, permitting such a simple offline time-synchronization strategy. This script also re-serializes every message to its header timestamp.

**Interface**: The script's interface is accessed via:
```
cd ~/catkin_ws/src/ig_handle/scripts
python3 process_raw_bag.py --help
```
The bagfile argument `-b` needs to be set every time to find the input bag. The values for data and time topics `-d, -t` are set correctly by default, so only specify those arguments if you've changed the data collection process. Below is an example of how to process collected raw data:
```
cd ~/catkin_ws/src/ig_handle/scripts
python3 process_raw_bag.py -b ~/bags/YYYY_MM_DD_HH_MM_SS/raw.bag
```
The script will output a rosbag called `output.bag` to the same folder specified via the `-b` argument, which can then be passed to a SLAM algorithm.

## Documentation

In addition to the build instructions found [here](https://drive.google.com/drive/folders/1DrAMQ9eQS1JjoDI4LWuoENaN7cZ9nRTC?usp=sharing), instructions on usage are provided.

### Networking

The network (as per `ig_handle/config/01-ig_handle_netplan.yaml`) is configured to expect a `192.168.1.XXX` subnet (255.255.255.0 aka /24 mask). The LiDARs are configured as `192.168.1.201` for LiDAR `lidar_h` and `192.168.1.202` for LiDAR `lidar_v`. The network switch inside of the handle box connects the LiDARs, the handle computer, and the outboard ethernet ports mounted on the handle box. If you are using this switch alone, it is important to set all IPs and netmasks **statically**.

The handle's ROS computer is statically assigned `192.168.1.150` via the netplan. In order to have this set automatically at startup, copy `ig_handle/config/01-ig_handle_netplan.yaml` to `/etc/netplan/`. Next, replace `enp2s0` with the name of the ethernet adapter that you are using. You can find the name (eth0, enp0s1, etc.) with the command:
 ```
 ifconfig
 ```
To gain access to the internet, we recommend to manually connect to a Wifi network. In order to participate with the ROS core running on the handle's ROS computer from your laptop over networking, you need to tell your laptop the ROS core is elsewhere on the network. This can be done be executing:
```
source ~/catkin_ws/src/ig_handle/config/remote-ig_handle.sh
```
Alternatively, you can copy the contents of `remote-ig_handle.sh` into your `~/.bashrc` file to automate the process.

### Hotspot

The handle's computer should be capable of starting its own hotspot. This is useful for ssh'ing into the computer and this can be enabled on startup. First, you need to manually enable hotspot by going to Wifi Settings, click the menu on the top right and click "Turn On Wi-Fi Hotpot...". To change the hotspot network settings (such as name and password), use the editor by running:
```
nm-connection-editor
```
To enable hotspot at computer startup, copy `ig_handle/config/hotspot.service` to `/etc/systemd/`. You also need to check that the shell script in `ig_handle/scripts/start_hotspot.sh` is an excutable. If not, run:
```
sudo chmod +x ~/catkin_ws/src/ig_handle/scripts/start_hotspot.sh
```

### Microcontroller

The microcontroller is a Teensy 4.1, which is very similar to an Arduino with more GPIO and power. The Teensy GPIO is used for sending start and trigger signals to cameras, receiving camera exposure indication signals, sending an imu start signal, receiving imu measurement indications, sending Lidar PPS (1hz synchronization pulse), and Lidar GPRMC serial signals (simulating GPS time/location messages with microsecond accuracy). The Teensy communicates with the handle's ROS computer using a rosserial node. The rosserial node gives the arduino code a nodehandle to subscribe/publish messages and access synchronized ROS time. When the Teensy receives camera/imu measurement indications, it publishes a TimeReference message. The PPS/GPRMC time-synchronization for the LiDAR can be verified by visiting the LiDARs' online configuration pages (enter `192.168.1.201` and/or `192.168.1.202` in a web browser), and making sure the PPS field is marked as `locked` and the GPS Position field shows `43 65.107N 793 47.702E` (this position is spoofed and indicates GPRMC serial signals are being received by the LiDAR).

### Udev

Udev rules are used in Ubuntu to create custom USB configurations when USB devices are plugged in. For example, in order to ensure the Teensy and IMU ports are always known, Udev rules are used to create aliases when these devices are plugged in. The installation process provided in Section **Installation** automates Udev rule creation. This process can also be accomplished manually by copying `/ig-handle/catkin_ws/src/ig_handle/config/99-ig_handle_udev.rules` to `/etc/udev/rules.d/` in order to implement the custom rules via:
```
sudo cp ~/catkin_ws/src/ig_handle/config/99-ig2_udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
Note Udev rules are automatically set in Section **Installation**.The rules scan for devices plugged in with the correct idVendor and idProduct, and execute actions when there is a match. Make sure your user is in the `dialout` group (should be by default) for the rules to work. The teensy is given a symlink found at `/dev/teensy`, the imu is given `/dev/imu`, and the Husky robot connection is given `/dev/prolific`.

The USB configurations assumes you are a member of the `dialout` user group. Use the commands `groups` to see your current groups. To add yourself to `dialout`:

```
sudo adduser $USER dialout
```

### Using Screen for Data Collection

**Screen** is a program for managing terminal sessions. It allows you to create terminal processes then attach and detach them using commands. When you detach, the process keeps running cleanly. You can pick up a detached process from any terminal. This can be used with **ig-handle** to start and stop data collection seamlessly from different computers. It also fixes bugs that occur with orphaned processes from ROS and data loss from the Sonar driver. Install **Screen** via:
```
sudo apt install screen
```
In order to start a session, use the command `screen`, then hit enter. You will now have a fresh terminal, but it is a screen process, so you can detach from it. Press ctrl+a followed by ctrl+d. Now the process is running in the background. You can return to this process from any terminal or ssh connection using the command `screen -r`. If there are multiple detached processes, `screen -r` will display a list of process ids. You can connect to any of them using `screen -r $id`.

A sequence for launching data collection on **pierre** would look like:
```
screen
# press enter to start
roslaunch ig_handle collect_raw_data.launch robot:=heron
# press ctrl+a, then ctrl+d to detach
# dataset collection occurs
screen -r
# end data collection
```
This prevents the sonar virtual machine from freezing bugs that occur when a standard ssh process is awaiting reconnection.
