# ig_handle

This package contains all necessary code to operate ig_handle.


## Starting the handle
Use ig_handle_collect_bag.launch to start ig_handle:

```

roslaunch ig_handle ig_handle_collect_bag.launch

```

This launch file will start the handle and record a bag file.

TODO:
-add a ros service from which teensy will request a startup clock time.

-add imu launching

-add Velodyne launching

-build upon Jake's buffer alignment code to publish timestamped images

-add install script
    >install dependencies (rosserial-python, camera stuff, etc)
    >make scripts/*.sh executable with chmod
    >etc.

