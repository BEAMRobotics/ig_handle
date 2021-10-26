#!/bin/bash

set -e # exit on first error

now=$(date +"%Y_%m_%d_%H_%M_%S")
bagDir="${1/#~/$HOME}/$now"
mkdir -p $bagDir
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/raw.bag"
rosbag record -O $bagDir/raw.bag \
/F1/image_raw \
/F1/cam_time \
/imu/data \
/imu/imu_time \
/lidar_h/velodyne_packets \
/lidar_h/velodyne_points \
/lidar_v/velodyne_packets \
/lidar_v/velodyne_points \
/vicon/SDICCheckerBoardTarget/SDICCheckerBoardTarget \
/vicon/SDICCylinderTarget1/SDICCylinderTarget1 \
/vicon/SDICCylinderTarget2/SDICCylinderTarget2 \
/vicon/SDICIG2/SDICIG2 \
/vicon/markers \
/tf