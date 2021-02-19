# beam_camera_sync

This repository contains the code to accurately timestamp images which have been hardware triggered. This also contains code to perform a spoofed PPS and transmit a NMEA packet to use for synchronizing Velodyne LiDARS.

## Dependencies:
  * rosserial
  * rosserial-arduino
  
## Running:

For this code to perform its job, a serial node must be launched before launching anything else to ensure the connection.

## Wiring:

See https://github.com/BEAMRobotics/Handle_ECAD for the schematic.
  
## Overview

This arduino script performs 3 main tasks:
1. Sends a variable rate signal (20Hz) to trigger cameras, and publishes the timestamp associated with the camera shutter closing to rosserial. (publishing on F1,F2,F3, + /cam_time)
2. Creates timestamps for IMU data based on the GPIO output from the IMU into the Teensy
3. Spoofs a PPS using the onboard clock
4. Constructs and sends an NMEA packet to send over UART serial in concert with the PPS for the lidars to use for synchronisation.

The Teensy is time synchronised via rosserial and therefore rosserial needs to be running on the host computer for this method to properly synchronise with the PC. It does this by waiting in the setup function until a connection with rosserial is gained, at which point, it will continue into the main loop.

## Notes

For instructions on editing the Teensy software, refer to the rosserial wiki: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
