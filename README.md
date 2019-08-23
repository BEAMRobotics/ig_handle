# beam_camera_sync

This repository contains the code to accurately timestamp images which have been hardware triggered. This also contains code to perform a spoofed PPS and transmit a NMEA packet to use for synchronizing Velodyne LiDARS.

## Dependencies:
  * rosserial
  * rosserial-arduino
  
## Running:

For this code to perform its job, a serial node must be launched before launching anything else to ensure the connection.

## Wiring:

  * VARF output: pin 2
  * PPS output: pin 3
  * NMEA serial output: pin 1
  
## Overview

This arduino script performs 3 main tasks:
1. Sends a variable rate signal (20Hz) through pin 2, and simultaneously publishes the timestamp associated with the triggering to rosserial. (publishing on F1,F2,F3,F4 + /cam_time)
2. Spoofs a PPS using the onboard clock over pin 3
3. Constructs and sends an NMEA packet to send over UART serial in concert with the PPS for the lidars to use for synchronisation.

The Teensy is time synchronised via rosserial and therefore rosserial needs to be running on the host computer for this method to properly synchronise with the PC. It does this by waiting in the setup function until a connection with rosserial is gained, at which point, it will continue into the main loop.
