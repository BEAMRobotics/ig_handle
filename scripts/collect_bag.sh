#!/bin/bash

set -e # exit on first error

now=$(date +"%Y-%m-%d-%H-%M-%S")
bagDir=$1
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/ig_handle_scan_$now.bag"
rosbag record -O $bagDir/ig_handle_scan_$now.bag \
/F1/image_raw/stamped \
/F1/cam_time \
/F2/image_raw/stamped \
/F2/cam_time \
/F3/image_raw/stamped \
/F3/cam_time \
/imu/data/stamped \
/imu/imu_time \