# ig_handle

This package contains all necessary code to operate ig_handle.

## Launch Files

Make sure you have a bag directory located at ~/bags before using the launch files.

### Raw data

Use collect_raw_data.launch to collect data (sensor messages) + time stamps (TimeReference messages, if applicable) for the core ig_handle sensors.  This includes cameras F1, F2, and F3, lidar_h, and imu.

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