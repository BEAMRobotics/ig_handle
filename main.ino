#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>
#include "Trigger.h"

#define GPSERIAL Serial1
using std_srvs::SetBool;

/* ROS node handler */
ros::NodeHandle nh;
/* Trigger variables for camera */
Trigger camera(2, 0.35, 50000);
std_msgs::Header cam_msg;
ros::Publisher F1_time("/F1/cam_time", &cam_msg);
ros::Publisher F3_time("/F3/cam_time", &cam_msg);
ros::Publisher F4_time("/F4/cam_time", &cam_msg);
bool F1publishing = false, F4publishing = false, F3publishing = false;
uint32_t F1sequence = 0, F3sequence = 0, F4sequence = 0;
/* Trigger variables for lidar */
Trigger pps(3, 0.01, 1000000);
bool arduino_pps = true;
/* Forward function declarations */
void F1Callback(const std_msgs::Bool &msg);
void F4Callback(const std_msgs::Bool &msg);
void F3Callback(const std_msgs::Bool &msg);
bool ppsCallback(const SetBool::Request &req, SetBool::Response &res);
String checksum(String msg);
String getTimeNow();
ros::Subscriber<std_msgs::Bool> toggleF1("/F1/toggle", F1Callback);
ros::Subscriber<std_msgs::Bool> toggleF4("/F4/toggle", F4Callback);
ros::Subscriber<std_msgs::Bool> toggleF3("/F3/toggle", F3Callback);
ros::ServiceServer<SetBool::Request, SetBool::Response> server("arduino_pps", &ppsCallback);

/*
 * Initial setup for the arduino sketch
 * This function performs:
 *  - Advertisement and subscribing to ROS topics
 *  - UART Serial setup for NMEA strings
 *  - IMU setup and calibration
 *  - Holds until rosserial is connected
 */
void setup()
{
  nh.initNode();
  nh.advertise(F1_time);
  nh.advertise(F3_time);
  nh.advertise(F4_time);
  nh.advertiseService(server);
  nh.subscribe(toggleF1);
  nh.subscribe(toggleF3);
  nh.subscribe(toggleF4);
  GPSERIAL.begin(9600);
  UART0_C3 = 16;
  while (!nh.connected())
  {
    nh.spinOnce();
  }
}

/*
 * Continously looping function performs the following:
 *  - Triggering camera line at certain frequency and publishes the timestamp to /cam_time
 *  - Trigger lidar line (PPS) and transmits NMEA string over Serial1
 *  - Reads data from IMU (over i2c) and publishes to /imu0 at certain frequency
 */
void loop()
{
  // If rosserial disconnects, stop publishing cam_time and imu0 and reset to 0
  if (!nh.connected())
  {
    F1publishing = false, F3publishing = false, F4publishing = false;
    F1sequence = 0, F3sequence = 0, F4sequence = 0;
  }

  nh.spinOnce();
  /* Handle camera triggering and stamping here */
  if (camera.TriggerPin() == true)
  {
    ros::Time t = nh.now();
    if (F1publishing)
    {
      cam_msg.seq = F1sequence;
      cam_msg.stamp = t;
      cam_msg.frame_id = "F1";
      F1_time.publish(&cam_msg);
      F1sequence++;
    }
    if (F3publishing)
    {
      cam_msg.seq = F3sequence;
      cam_msg.stamp = t;
      cam_msg.frame_id = "F3";
      F3_time.publish(&cam_msg);
      F3sequence++;
    }
    if (F4publishing)
    {
      cam_msg.seq = F4sequence;
      cam_msg.stamp = t;
      cam_msg.frame_id = "F4";
      F4_time.publish(&cam_msg);
      F4sequence++;
    }
  }
  //Handle lidar PPS and NMEA sending
  if (arduino_pps)
  {
    if (pps.TriggerPinImmediate())
    {
      delay(30);
      String time_now = getTimeNow();
      String nmea_string = "GPRMC," + time_now + ",A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A";
      GPSERIAL.print(nmea_string);
    }
  }
}

/***********************************************
 *            Helper functions                 *
 ***********************************************/

/* Callback for /toggle to being publishing */
void F1Callback(const std_msgs::Bool &msg)
{
  F1publishing = msg.data;
}
void F4Callback(const std_msgs::Bool &msg)
{
  F4publishing = msg.data;
}
void F3Callback(const std_msgs::Bool &msg)
{
  F3publishing = msg.data;
}
/* PPS callback for toggling whether to trigger PPS and NMEA */
bool ppsCallback(const SetBool::Request &req, SetBool::Response &res)
{
  arduino_pps = req.data;
  res.success = true;
  res.message = "success";
}
/* Gets current time from RTC and formats into HHMMSS */
String getTimeNow()
{
  String time_formatted;
  int ts = nh.now().sec;
  int nsec = (nh.now().nsec) / 1000000;
  int h = (ts / 3600) % 24, min = (ts / 60) % 60, sec = ts % 60;
  String hours_s, minutes_s, seconds_s;
  if (h < 10)
  {
    hours_s = "0" + String(h);
  }
  else
  {
    hours_s = String(h);
  }
  if (min < 10)
  {
    minutes_s = "0" + String(min);
  }
  else
  {
    minutes_s = String(min);
  }
  if (sec < 10)
  {
    seconds_s = "0" + String(sec);
  }
  else
  {
    seconds_s = String(sec);
  }
  time_formatted = hours_s + minutes_s + seconds_s + "." + String(nsec);
  return time_formatted;
}
/* COmputes XOR checksum of NMEA sentnece */
String checksum(String msg)
{
  byte chksum = 0;
  int l = msg.length();
  for (int i = 0; i < l; i++)
  {
    chksum ^= msg[i];
  }
  String result = String(chksum, HEX);
  result.toUpperCase();
  if (result.length() < 2)
  {
    result = "0" + result;
  }
  return result;
}
