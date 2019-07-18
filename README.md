# beam_camera_sync

This repository contains the code to accurately timestamp images which have been hardware triggered. This also contains code to perform a spoofed PPS and transmit a NMEA packet to use for synchronizing Velodyne LiDARS.

## Dependencies:
  * rosserial
  * rosserial-arduino
  
## Running:

For this code to perform its job, a serial node must be launched before launching anything else to ensure the connection.
