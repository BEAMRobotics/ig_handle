# beam_camera_sync

This repository contains the code to accurately timestamp images, IMU measurements, and LiDAR scans. This also contains code to perform a spoofed PPS and transmit a NMEA packet to use for synchronizing Velodyne LiDARS.

## Dependencies:
  * rosserial
  * rosserial-arduino
  
## Running:

For this code to perform its job, a serial node must be launched before launching anything else to ensure the connection.

## Wiring:

See https://github.com/BEAMRobotics/Handle_ECAD for the schematic.
  
## Overview

This arduino script performs 3 main tasks:
1. Sending a variable rate signal (20Hz) to trigger cameras. The camera sends a signal back to the Teensy corresponding to the shutter being open. The Teensy publishes the timestamp associated with the camera shutter closing to rosserial. (publishing on F1,F2,F3, + /cam_time)
2. Creates timestamps for IMU data based on the GPIO output from the IMU into the Teensy
3. Spoofs a PPS and sends an NMEA string to the LiDAR for synchronization

The Teensy waits in the setup function until a connection with rosserial is gained, at which point, it will continue into the main loop where it publishes the timestamps.

## Notes

For instructions on editing the Teensy software, refer to the rosserial wiki: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
