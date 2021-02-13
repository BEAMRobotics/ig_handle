#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
//#include <sensor_msgs/Imu.h>
#include <FrequencyTimer2.h>
#include <TimeLib.h>
#include "Trigger.h"

#define GPSERIAL Serial1
#define PPS_PIN 6
#define IMU_IN 12
#define CAM0_OUT 2
#define CAM1_OUT 7
#define CAM2_OUT 8
#define CAM0_IN 9
#define CAM1_IN 10
#define CAM2_IN 11

using std_srvs::SetBool;

/* ROS node handler */
//ros::NodeHandle nh;
/* Trigger variables for camera */
//Trigger camera(2, 0.35, 50000);
//std_msgs::Header cam_msg;
//ros::Publisher F1_time("/F1/cam_time", &cam_msg);
//ros::Publisher F2_time("/F2/cam_time", &cam_msg);
//ros::Publisher F3_time("/F3/cam_time", &cam_msg);
//ros::Publisher F4_time("/F4/cam_time", &cam_msg);
//bool F1publishing = false, F2publishing = false, F4publishing = false, F3publishing = false;
//uint32_t F1sequence = 0, F3sequence = 0, F4sequence = 0, F2sequence = 0;

///* Forward function declarations */
//void F1Callback(const std_msgs::Bool &msg);
//void F2Callback(const std_msgs::Bool &msg);
//void F4Callback(const std_msgs::Bool &msg);
//void F3Callback(const std_msgs::Bool &msg);
void setSendNMEA(void);
void enableTriggers(bool onOff);
//bool ppsCallback(const SetBool::Request &req, SetBool::Response &res);
//String checksum(String msg);
//ros::Subscriber<std_msgs::Bool> toggleF1("/F1/toggle", F1Callback);
//ros::Subscriber<std_msgs::Bool> toggleF2("/F2/toggle", F2Callback);
//ros::Subscriber<std_msgs::Bool> toggleF4("/F4/toggle", F4Callback);
//ros::Subscriber<std_msgs::Bool> toggleF3("/F3/toggle", F3Callback);
//ros::ServiceServer<SetBool::Request, SetBool::Response> server("arduino_pps", &ppsCallback);

volatile bool sendNMEA = false;
volatile elapsedMicros microsSincePPS;

/*
 * Initial setup for the arduino sketch
 * This function performs:
 *  - Configure timers for LiDAR and camera triggering
 *  - Advertisement and subscribing to ROS topics
 *  - UART Serial setup for NMEA strings
 *  - Holds until rosserial is connected
 */
void setup()
{
  /* In future maybe there'll be a subscriber to get settings from the main PC */
  
  /* Setup outputs to LiDAR
   *    - PPS but don't start it yet
        - Attach */
  pinMode(PPS_PIN, OUTPUT); // pin 5 is still driven by default but has a 50% duty cycle
  FrequencyTimer2::setPeriod(1000000);     // 10^6 microseconds, 1 second
  FrequencyTimer2::setOnOverflow(setSendNMEA);  // sets the function that runs when the timer overflows

  /* set up the camera triggers but don't start them yet either */
  analogWriteFrequency(CAM0_OUT, 20.0); // 20.0 Hz base frequency for the PWM signal
  analogWriteFrequency(CAM0_OUT, 20.0); // We're using a PWM signal because it's a way of offloading
  analogWriteFrequency(CAM0_OUT, 20.0); // the task to free up the main loop

  /* node initialization
  nh.initNode();
  nh.advertise(F1_time);
  nh.advertise(F2_time);
  nh.advertise(F3_time);
  nh.advertise(F4_time);
  nh.advertiseService(server);
  nh.subscribe(toggleF1);
  nh.subscribe(toggleF2);
  nh.subscribe(toggleF3);
  nh.subscribe(toggleF4);
  */

//  while (!nh.connected())
//  {
//    nh.spinOnce();
//  }
//  setSyncProvider((time_t) Teensy3Clock.get);  // Arduino time library will use the onboard RTC
  
//  Teensy3Clock.set(nh.now());           // Initialize RTC with time
//  setTime(nh.now());

  /* start all the timers */
  enableTriggers(true);

  /* enable interrupts */
  pinMode(CAM0_IN, INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(CAM0_IN), cam0_ISR, RISING); // Falling or rising TBD
  attachInterrupt(digitalPinToInterrupt(CAM0_IN), cam0_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM0_IN), cam0_ISR, RISING);
}

/*
 * Continously looping function performs the following:
 *  - Triggering camera line at certain frequency and publishes the timestamp to /cam_time
 *  - Trigger lidar line (PPS) and transmits NMEA string over Serial1
 */
void loop()
{
  if (sendNMEA == true) {
    char time_now[7], date_now[7];
    sprintf(time_now, "%02u%02i%02i", hour(), minute(), second());
    sprintf(date_now, "%02u%02i%02i", day(), month(), year()-2000);
    String nmea_string = F("GPRMC,") + String(time_now) + F(",A,4807.038,N,01131.000,E,022.4,084.4,") + String(date_now) + ",003.1,W";
    String chk = checksum(nmea_string);
    nmea_string = "$" + nmea_string + "*" + chk + "\n";
    GPSERIAL.print(nmea_string);
    sendNMEA = false;
    digitalWriteFast(PPS_PIN, LOW); // minimum pulse duration required by LiDAR is 10 us

    Serial.print(nmea_string);  // for debugging
  }
  
//  // If rosserial disconnects, stop publishing cam_time and imu0 and reset to 0
//  if (!nh.connected())
//  {
//    F1publishing = false, F2publishing = false, F3publishing = false, F4publishing = false;
//    F1sequence = 0, F2sequence = 0, F3sequence = 0, F4sequence = 0;
//  }
//
//  nh.spinOnce();
//  /* Handle camera triggering and stamping here */
//  if (camera.TriggerPin() == true)
//  {
//    ros::Time t = nh.now();
//    if (F1publishing)
//    {
//      cam_msg.seq = F1sequence;
//      cam_msg.stamp = t;
//      cam_msg.frame_id = "F1";
//      F1_time.publish(&cam_msg);
//      F1sequence++;
//    }
//    if (F2publishing)
//    {
//      cam_msg.seq = F2sequence;
//      cam_msg.stamp = t;
//      cam_msg.frame_id = "F2";
//      F2_time.publish(&cam_msg);
//      F2sequence++;
//    }
//    if (F3publishing)
//    {
//      cam_msg.seq = F3sequence;
//      cam_msg.stamp = t;
//      cam_msg.frame_id = "F3";
//      F3_time.publish(&cam_msg);
//      F3sequence++;
//    }
//    if (F4publishing)
//    {
//      cam_msg.seq = F4sequence;
//      cam_msg.stamp = t;
//      cam_msg.frame_id = "F4";
//      F4_time.publish(&cam_msg);
//      F4sequence++;
//    }
//  }
}

/***********************************************
 *            Helper functions                 *
 ***********************************************/

/* Callbacks for /toggle to begin publishing */
//void F1Callback(const std_msgs::Bool &msg)
//{
//  F1publishing = msg.data;
//}
//void F2Callback(const std_msgs::Bool &msg)
//{
//  F2publishing = msg.data;
//}
//void F4Callback(const std_msgs::Bool &msg)
//{
//  F4publishing = msg.data;
//}
//void F3Callback(const std_msgs::Bool &msg)
//{
//  F3publishing = msg.data;
//}
///* PPS callback for toggling whether to trigger PPS and NMEA */
//bool ppsCallback(const SetBool::Request &req, SetBool::Response &res)
//{
//  arduino_pps = req.data;
//  res.success = true;
//  res.message = "success";
//}

void setSendNMEA(void) {
  /*  It's not really recommended to write to pins from an interrupt as it can take a relatively
   *  long time to execute. However, one of the main purposes of this code is to output accurate
   *  PPS signal and this is the most accurate way that abides by the format constraints on the
   *  signal.
   */
  digitalWriteFast(PPS_PIN, HIGH);
  sendNMEA = true;
  microsSincePPS = 0;
}

/* COmputes XOR checksum of NMEA sentence */
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

enableTriggers(bool onOff)
{
  if (onOff)
  {
    FrequencyTimer2::enable();
  } 
  else
  {
    FrequencyTimer2::disable();
  }
  analogWrite(CAM0_OUT, 5 * onOff);   // 5% duty cycle @ 20 Hz = 2.5 ms pulse
  analogWrite(CAM1_OUT, 5 * onOff);
  analogWrite(CAM2_OUT, 5 * onOff);
}
