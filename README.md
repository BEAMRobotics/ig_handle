# ig_handle

This package contains all necessary code to operate ig_handle.


## Launch Files

Make sure you have a directory located at /home/"user"/bags before using the launch files.

### Raw data

Use collect_raw_data.launch to collect data + time stamps 


```

roslaunch ig_handle collect_raw_data.launch

```

This launch file will start the handle and record a bag file.

### Restamp datas

Use restamp_data.launch to restamp data topics with time stamp topic headers. 


```

roslaunch ig_handle restamp_data.launch

```

Once the restamp nodelets have started, play back a raw data bag.  It is safe to use -r to speed up the playback.