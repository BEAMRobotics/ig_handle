#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
//#include <sensor_msgs/Imu.h>
#include <FrequencyTimer2.h>
#include <TimeLib.h>
//#include "Trigger.h"

#define GPSERIAL Serial1
#define PPS_PIN 6
#define IMU_IN 12
#define CAM1_OUT 2
#define CAM2_OUT 7
#define CAM3_OUT 8
#define CAM1_IN 9
#define CAM2_IN 10
#define CAM3_IN 11

using std_srvs::SetBool;

/* ROS node handler */
ros::NodeHandle nh;
/* Trigger variables for camera */
std_msgs::Header cam_msg;
ros::Publisher F1_time("/F1/cam_time", &cam_msg);
ros::Publisher F2_time("/F2/cam_time", &cam_msg);
ros::Publisher F3_time("/F3/cam_time", &cam_msg);
ros::Publisher IMU_time("/imu/imu_time", &cam_msg);
bool arduino_pps, F1publishing = true, F2publishing = true, F3publishing = true, IMUpublishing = true;
uint32_t F1sequence = 0, F2sequence = 0, F3sequence = 0, IMUsequence = 0;
volatile bool sendNMEA = false, F1_closed = false, F2_closed = false, F3_closed = false, IMU_sampled = false;
 elapsedMicros microsSincePPS;
volatile uint32_t F1_close_s, F1_close_us, F2_close_s, F2_close_us, F3_close_s, F3_close_us, IMU_sample_s, IMU_sample_us;

///* Forward function declarations */
void F1Callback(const std_msgs::Bool &msg);
void F2Callback(const std_msgs::Bool &msg);
void F3Callback(const std_msgs::Bool &msg);
void IMUCallback(const std_msgs::Bool &msg);
void cam1_ISR(void);
void cam2_ISR(void);
void cam3_ISR(void);
void IMU_ISR(void);
void setSendNMEA(void);
void enableTriggers(bool onOff);
void ppsCallback(const SetBool::Request &req, SetBool::Response &res);
String checksum(String msg);
ros::Subscriber<std_msgs::Bool> toggleF1("/F1/toggle", F1Callback);
ros::Subscriber<std_msgs::Bool> toggleF2("/F2/toggle", F2Callback);
ros::Subscriber<std_msgs::Bool> toggleF3("/F3/toggle", F3Callback);
ros::Subscriber<std_msgs::Bool> toggleIMU("/imu/toggle", IMUCallback);
//ros::ServiceServer<SetBool::Request, SetBool::Response> server("arduino_pps", &ppsCallback);

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
  /* Setup outputs to LiDAR
   *    - PPS but don't start it yet
        - Attach */
  pinMode(PPS_PIN, OUTPUT); // pin 5 is still driven by default but has a 50% duty cycle
  FrequencyTimer2::setPeriod(1000000);     // 10^6 microseconds, 1 second
  FrequencyTimer2::setOnOverflow(setSendNMEA_ISR);  // sets the function that runs when the timer overflows

  /* set up the camera triggers but don't start them yet either */
  analogWriteFrequency(CAM1_OUT, 20.0); // 20.0 Hz base frequency for the PWM signal
  analogWriteFrequency(CAM2_OUT, 20.0); // We're using a PWM signal because it's a way of offloading
  analogWriteFrequency(CAM3_OUT, 20.0); // the task to free up the main loop

  /* configure input pins */
  pinMode(CAM1_IN, INPUT_PULLUP);
  pinMode(CAM2_IN, INPUT_PULLUP);
  pinMode(CAM3_IN, INPUT_PULLUP);
  pinMode(IMU_IN, INPUT);

// node initialization
  nh.initNode();
  nh.advertise(F1_time);
  nh.advertise(F2_time);
  nh.advertise(F3_time);
  nh.advertise(IMU_time);
////  nh.advertiseService(server);
  nh.subscribe(toggleF1);
  nh.subscribe(toggleF2);
  nh.subscribe(toggleF3);
  nh.subscribe(toggleIMU);

  while (!nh.connected())
  {
    nh.spinOnce();
  }

// TODO: initialize RTC clock using ROS service
  setSyncProvider((time_t) Teensy3Clock.get); // set RTC clock as Time library source.
  setTime(Teensy3Clock.get());

  /* start all the timers */
  enableTriggers(true);

  Serial.begin(57600);
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
    time_t t = now();
    sprintf(time_now, "%02i%02i%02i", hour(t), minute(t), second(t));
    sprintf(date_now, "%02i%02i%02i", day(t), month(t), year(t) % 100);
    String nmea_string = F("GPRMC,") + String(time_now) + F(",A,4807.038,N,01131.000,E,022.4,084.4,") + String(date_now) + ",003.1,W";
    String chk = checksum(nmea_string);
    nmea_string = "$" + nmea_string + "*" + chk + "\n";
    GPSERIAL.print(nmea_string);
    sendNMEA = false;
    digitalWriteFast(PPS_PIN, LOW); // minimum pulse duration required by LiDAR is 10 us

    Serial.print(nmea_string);  // for debugging
  }
  if ((F1_closed && F1publishing) == true) {
    cam_msg.seq = F1sequence;
    cam_msg.stamp = ros::Time(F1_close_s, 1000*F1_close_us);
    Serial.print(F1_close_s);
    cam_msg.frame_id = "F1";
    F1_time.publish(&cam_msg);
    F1sequence++;
    F1_closed = false;
  }
//  if (F2_closed == true) {
  if ((F2_closed && F2publishing) == true) {
    cam_msg.seq = F2sequence;
    cam_msg.stamp = ros::Time(F2_close_s, 1000*F2_close_us);
    cam_msg.frame_id = "F2";
    F2_time.publish(&cam_msg);
    F2sequence++;
    F2_closed = false;
  }
//  if (F3_closed == true) {
  if ((F3_closed && F3publishing) == true) {
    cam_msg.seq = F3sequence;
    cam_msg.stamp = ros::Time(F3_close_s, 1000*F3_close_us);
    cam_msg.frame_id = "F3";
    F3_time.publish(&cam_msg);
    F3sequence++;
    F3_closed = false;
  }
//  if (IMU_sampled == true) {
  if ((IMU_sampled && IMUpublishing) == true) {
    cam_msg.seq = IMUsequence;
    cam_msg.stamp = ros::Time(IMU_sample_s, 1000*IMU_sample_us);
    cam_msg.frame_id = "imu";
    IMU_time.publish(&cam_msg);
    IMUsequence++;
    IMU_sampled = false;
  }
  
  // If rosserial disconnects, stop publishing cam_time and imu0 and reset to 0
  if (!nh.connected())
  {
    enableTriggers(false);
    F1publishing = false, F2publishing = false, F3publishing = false, IMUpublishing = false;
    F1sequence = 0, F2sequence = 0, F3sequence = 0, IMUsequence = 0;
  }

  nh.spinOnce();
  /* Handle camera triggering and stamping here */

}

/***********************************************
 *            Helper functions                 *
 ***********************************************/

/* Callbacks for /toggle to begin publishing */
void F1Callback(const std_msgs::Bool &msg)
{
  F1publishing = msg.data;
}
void F2Callback(const std_msgs::Bool &msg)
{
  F2publishing = msg.data;
}
void F3Callback(const std_msgs::Bool &msg)
{
  F3publishing = msg.data;
}
void IMUCallback(const std_msgs::Bool &msg)
{
  IMUpublishing = msg.data;
}
/* PPS callback for toggling whether to trigger PPS and NMEA */
void ppsCallback(const SetBool::Request &req, SetBool::Response &res)
{
  arduino_pps = req.data;
  res.success = true;
  res.message = "success";
}

void setSendNMEA_ISR(void) {
  /*  It's not really recommended to write to pins from an interrupt as it can take a relatively
   *  long time to execute. However, one of the main purposes of this code is to output accurate
   *  PPS signal and this is the most accurate way that abides by the format constraints on the
   *  signal.
   */
  digitalWriteFast(PPS_PIN, HIGH);
  sendNMEA = true;
  microsSincePPS = 0;
}

// Timestamp creation interrupts
void cam1_ISR(void){
  F1_close_s = now();
  F1_close_us = microsSincePPS;
  F1_closed = true;
}
void cam2_ISR(void){
  F2_close_s = now();
  F2_close_us = microsSincePPS;
  F2_closed = true;
}
void cam3_ISR(void){
  F3_close_s = now();
  F3_close_us = microsSincePPS;
  F3_closed = true;
}
void IMU_ISR(void){
  IMU_sample_s = now();
  IMU_sample_us = microsSincePPS;
  IMU_sampled = true;
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

void enableTriggers(bool onOff)
{
  if (onOff)
  {
    FrequencyTimer2::enable();
  } 
  else
  {
    FrequencyTimer2::disable();
  }
  analogWrite(CAM1_OUT, 5 * onOff);   // 5% duty cycle @ 20 Hz = 2.5 ms pulse
  analogWrite(CAM2_OUT, 5 * onOff);
  analogWrite(CAM3_OUT, 5 * onOff);
  F1publishing = true;
  F2publishing = true;
  F3publishing = true;
  IMUpublishing = true;

  /* enable interrupts */
  attachInterrupt(digitalPinToInterrupt(CAM1_IN), cam1_ISR, RISING); // Falling or rising TBD
  attachInterrupt(digitalPinToInterrupt(CAM2_IN), cam2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM3_IN), cam3_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), IMU_ISR, RISING);
}
