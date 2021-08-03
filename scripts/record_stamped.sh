#!/bin/bash

set -e # exit on first error

now=$(date +"%Y-%m-%d-%H-%M-%S")
bagDir=$1
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/ig_handle_stamped_$now.bag"
rosbag record -O $bagDir/ig_handle_stamped_$now.bag \
/F1/image_raw/stamped \
/F2/image_raw/stamped \
/F3/image_raw/stamped \
/imu/data/stamped \
/lidar_h/velodyne_packets \
/lidar_v/velodyne_packets
