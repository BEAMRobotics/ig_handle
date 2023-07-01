#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <TeensyTimerTool.h>
#include <TimeLib.h>

#define USE_USBCON

// Electrical component pin numbers
#define GPSERIAL Serial1  // LiDAR: GPS Serial Receive (White)
#define PPS_PIN 2         // LiDAR: GPS Sync Pulse (Yellow)
#define CAM_OUT 3         // Cam: Line 0 (Black)
#define CAM_OPEN_IN 4     // Cam: Line 1 (White)
#define CAM_CLOSE_IN 7    // Cam: Line 1 (White)
#define IMU_OUT 8         // IMU: SynchIn (Blue)
#define IMU_IN 9          // IMU: SynchOut (Pink)

// PPS/GPRMC time-synch
constexpr int BAUD_RATE = 9600;              // baud/s
constexpr int PPS_PULSE_WIDTH = 20;          // ms
constexpr int PPS_NMEA_MIN_SEPARATION = 55;  // ms
constexpr int TIME_ZONE_OFFSET = -7;         // hr from UTC (User Set)

using namespace TeensyTimerTool;

// ROS nodehandle
ros::NodeHandle nh;

// microcontroller clock
PeriodicTimer teensy_clock(TCK_RTC);

// messages and time topics for camera and imu
sensor_msgs::TimeReference cam_time_msg;
sensor_msgs::TimeReference imu_time_msg;
ros::Publisher cam_time_pub("/cam/time", &cam_time_msg);
ros::Publisher imu_time_pub("/imu/time", &imu_time_msg);

// time-sync indicators
elapsedMillis nmea_delay;
elapsedMicros micros_since_pps;
ros::Time pps_stamp, cam_mid_stamp, imu_stamp;
unsigned long cam_open_t_sec, cam_mid_t_sec, cam_close_t_sec;
unsigned long cam_open_t_nsec, cam_mid_t_nsec, cam_close_t_nsec;
volatile bool send_nmea = false, cam_captured = false, imu_sampled = false;

/*
 * Initial setup for the arduino sketch
 * This function:
 *  - Configures timers for LiDAR PPS and camera triggering
 *  - Advertises and subscribes to ROS topics
 *  - UART Serial setup for NMEA messages
 *  - Holds until rosserial is connected
 */
void setup() {
  /* Lidar */

  // set GPSERIAL baud rate and transmission inversion for TTL RS-232
  // transmission
  GPSERIAL.begin(BAUD_RATE, SERIAL_8N1_TXINV);

  // set PPS pin
  pinMode(PPS_PIN, OUTPUT);

  // begin clock and call ppsISR every second
  teensy_clock.begin(ppsISR, 1'000'000);

  /* Camera and IMU */

  // node initialization
  nh.initNode();
  nh.advertise(cam_time_pub);
  nh.advertise(imu_time_pub);

  // configure input and out pins
  pinMode(CAM_OPEN_IN, INPUT_PULLUP);
  pinMode(CAM_CLOSE_IN, INPUT_PULLUP);
  pinMode(CAM_OUT, OUTPUT);
  pinMode(IMU_IN, INPUT);
  pinMode(IMU_OUT, OUTPUT);

  // enable interrupts
  attachInterrupt(digitalPinToInterrupt(CAM_OPEN_IN), camOpenISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM_CLOSE_IN), camCloseISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), imuISR, RISING);

  // Note: We're using a PWM signal because it's a way of offloading
  //       the task to free up the main loop. The base frequency for
  //       the PWM signal is 20.0 Hz
  analogWriteFrequency(CAM_OUT, 20.0);

  // ensure pin is low before we send a rising edge
  digitalWrite(IMU_OUT, LOW);

  // await node handle time sync
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // enable triggers
  nh.loginfo("Setup complete, Enabling triggers.");
  analogWrite(CAM_OUT, 5);  // 5% duty cycle @ 20 Hz = 2.5 ms pulse
  digitalWrite(IMU_OUT, HIGH);
  nh.loginfo("Triggers enabled.");
}

/*
 * Main loop
 * This function:
 *  - Transmits NMEA messages over GPSERIAL
 *  - Publishes the camera capture timestamp to /cam_time
 *  - Publishes the IMU sample timestamp to /imu_time
 */
void loop() {
  // ensure PPS width satisfied
  if (send_nmea && nmea_delay >= PPS_PULSE_WIDTH) {
    // set PPS pin to low
    digitalWriteFast(PPS_PIN, LOW);

    // ensure min 50 ms width between end of PPS and start of NMEA message
    if (nmea_delay >= PPS_NMEA_MIN_SEPARATION) {
      // get PPS time
      const time_t t_sec_gmt = pps_stamp.sec - TIME_ZONE_OFFSET * 3600;

      // create NMEA string
      char time_now[7], date_now[7];
      sprintf(time_now, "%02i%02i%02i", hour(t_sec_gmt), minute(t_sec_gmt),
              second(t_sec_gmt));
      sprintf(date_now, "%02i%02i%02i", day(t_sec_gmt), month(t_sec_gmt),
              year(t_sec_gmt) % 100);
      String nmea_string = F("GPRMC,") + String(time_now) +
                           F(",A,4365.107,N,79347.702,E,022.4,084.4,") +
                           String(date_now) + ",003.1,W";
      String chk = checksum(nmea_string);
      nmea_string = "$" + nmea_string + "*" + chk + "\n";

      // print NMEA string to serial as an NMEA message
      GPSERIAL.print(nmea_string);
      // nh.loginfo(nmea_string.c_str());  // DEBUG

      // reset send
      send_nmea = false;
    }
  }

  if (cam_captured) {
    cam_time_msg.time_ref = cam_mid_stamp;
    cam_time_pub.publish(&cam_time_msg);
    cam_captured = false;
  }

  if (imu_sampled) {
    imu_time_msg.time_ref = imu_stamp;
    imu_time_pub.publish(&imu_time_msg);
    imu_sampled = false;
  }

  nh.spinOnce();
}

// Timestamp creation interrupts
void ppsISR(void) {
  // reset elapsed micro seconds
  micros_since_pps = 0;

  // get time of PPS
  const time_t pps_sec = rtc_get();
  pps_stamp.sec = pps_sec;
  pps_stamp.nsec = 0;
  // printROSTime("PPS Time:", pps_stamp);  // DEBUG

  // toggle to HIGH
  digitalToggleFast(PPS_PIN);

  // enable send and reset delay counter
  send_nmea = true;
  nmea_delay = 0;
}

void camOpenISR(void) {
  cam_open_t_sec = pps_stamp.sec;
  cam_open_t_nsec = micros_since_pps * 1000;
  // ros::Time cam_open_stamp(cam_open_t_sec, cam_open_t_nsec);  // DEBUG
  // printROSTime("CAM OPN Time:", cam_open_stamp);              // DEBUG
}

void camCloseISR(void) {
  cam_close_t_sec = pps_stamp.sec;
  cam_close_t_nsec = micros_since_pps * 1000;

  cam_mid_t_sec = cam_open_t_sec;
  if (cam_close_t_nsec > cam_open_t_nsec) {
    cam_mid_t_nsec = (cam_close_t_nsec + cam_open_t_nsec) * 0.5;
  } else {
    cam_mid_t_nsec = cam_open_t_nsec +
                     0.5 * (cam_close_t_nsec + 1000000000 - cam_open_t_nsec);
  }

  const ros::Time cam_mid_stamp_tmp(cam_mid_t_sec, cam_mid_t_nsec);
  cam_mid_stamp = cam_mid_stamp_tmp;

  // ros::Time cam_close_stamp(cam_close_t_sec, cam_close_t_nsec);  // DEBUG
  // printROSTime("CAM MID Time:", cam_mid_stamp);                  // DEBUG
  // printROSTime("CAM CLD Time:", cam_close_stamp);                // DEBUG
}

void imuISR(void) {
  imu_stamp.sec = pps_stamp.sec;
  imu_stamp.nsec = micros_since_pps * 1000;
  imu_sampled = true;
  // printROSTime("IMU Time:", imu_stamp);  // DEBUG
}

// Computes XOR checksum of NMEA sentence
String checksum(String msg) {
  byte chksum = 0;
  int l = msg.length();
  for (int i = 0; i < l; i++) {
    chksum ^= msg[i];
  }

  String result = String(chksum, HEX);
  result.toUpperCase();
  if (result.length() < 2) {
    result = "0" + result;
  }
  return result;
}

// Print for debugging
void printROSTime(const String& msg, const ros::Time& ros_time) {
  // get seconds and nano seconds
  const time_t& t_sec = ros_time.sec;
  const time_t& t_nsec = ros_time.nsec;

  // convert ros time to string
  char t_sec_string[11], t_nsec_string[10];
  sprintf(t_sec_string, "%lld", (long long)t_sec);
  sprintf(t_nsec_string, "%lld", (long long)t_nsec);
  String ros_time_string =
      "sec: " + String(t_sec_string) + " nsec: " + String(t_nsec_string);

  // print
  nh.loginfo(msg.c_str());
  nh.loginfo(ros_time_string.c_str());
}
