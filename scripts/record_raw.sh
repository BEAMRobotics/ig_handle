#!/bin/bash

set -e # exit on first error

now=$(date +"%Y-%b-%d-%I-%M%P")
bagDir="${1/#~/$HOME}/$now"
mkdir -p $bagDir
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/raw.bag"
rosbag record -O $bagDir/raw.bag \
/F1/image_raw \
/F1/cam_time \
/F2/image_raw \
/F2/cam_time \
/F3/image_raw \
/F3/cam_time \
/imu/data \
/imu/imu_time \
/lidar_h/velodyne_points \
/lidar_v/velodyne_points
