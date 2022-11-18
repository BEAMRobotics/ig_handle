#!/bin/bash

set -e # exit on first error

now=$(date +"%Y_%m_%d_%H_%M_%S")
bagDir="$HOME/$now"
mkdir -p $bagDir
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/raw.bag"
rosbag record -O $bagDir/raw.bag \
/F1/image_raw \
/F1/cam_time \
/F2/image_raw  \
/F2/cam_time \
