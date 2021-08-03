#!/bin/bash

set -e # exit on first error

now=$(date +"%Y-%m-%d-%H-%M-%S")
bagDir=$1
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/ig_handle_raw_$now.bag"
rosbag record -O $bagDir/ig_handle_raw_$now.bag \
/F1/image_raw \
/F1/cam_time \
/F2/image_raw \
/F2/cam_time \
/F3/image_raw \
/F3/cam_time \
/imu/data \
/imu/imu_time \
/lidar_h/velodyne_packets \
/lidar_v/velodyne_packets
