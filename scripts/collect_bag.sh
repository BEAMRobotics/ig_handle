#!/bin/bash

set -e # exit on first error

now=$(date +"%Y-%m-%d-%H-%M-%S")
bagDir=$1
echo "Collecting ig_handle bag file..."
echo "Saving to: "
echo "$bagDir/ig_handle_scan_$now.bag"
rosbag record -O $bagDir/ig_handle_scan_$now.bag \
/F1/cam_time