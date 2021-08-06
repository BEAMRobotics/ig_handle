#!/bin/bash

set -e # exit on first error

bagDir="${1/#~/$HOME}/stamped"
mkdir -p $bagDir
now=$(date +"%Y-%b-%d-%I-%M%P")
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/stamped.bag"
rosbag record -O $bagDir/stamped.bag \
/F1/image_raw/stamped \
/F2/image_raw/stamped \
/F3/image_raw/stamped \
/imu/data/stamped \
/lidar_h/velodyne_packets \
/lidar_v/velodyne_packets
