# ig_handle

This package contains all necessary code to operate ig_handle.


## Launch Files

Make sure you have a directory located at /home/"user"/bags before using the launch files.

### Raw data

Use collect_raw_data.launch to collect data + time stamps 


```

roslaunch ig_handle collect_raw_data.launch

```

This launch file will start the handle and record a bag file.  Bags will be recorded in a timestamped folder as "raw.bag".  Use "output" arg to specify the parent directory for the timestamped folder (default is ~/bags).  For example:

```
roslaunch ig_handle collect_raw_data.launch output:=~/my_folder
```

This command will record data to "~/my_folder/2021-Aug-09-07-01pm/raw.bag".