#include <ros.h>
#include <sensor_msgs/TimeReference.h>

#include <RTClib.h>
#include <TimeLib.h>

#define USE_USBCON

// Electrical component pin numbers
#define GPSERIAL Serial1  // LiDAR: GPS Serial Receive (White)
#define PPS_OUT 2         // LiDAR: GPS Sync Pulse (Yellow)
#define PPS_IN 3          // RTC: PPS Signal (SQW)
#define CAM_OUT 4         // Cam: Line 0 (Black)
#define CAM_OPEN_IN 5     // Cam: Line 1 (White)
#define CAM_CLOSE_IN 6    // Cam: Line 1 (White)
#define IMU_OUT 7         // IMU: SynchIn (Blue)
#define IMU_IN 8          // IMU: SynchOut (Pink)

// PPS/GPRMC time-synch
constexpr int BAUD_RATE = 9600;              // baud/s
constexpr int PPS_PULSE_WIDTH = 20;          // ms
constexpr int PPS_NMEA_MIN_SEPARATION = 55;  // ms
constexpr int TIME_ZONE_OFFSET = -7;         // hr from UTC (User Set)

// ROS nodehandle
ros::NodeHandle nh;

// PPS signal source
RTC_DS3231 rtc;

// messages and time topics for camera and imu
sensor_msgs::TimeReference pps_time_msg;
sensor_msgs::TimeReference cam_time_msg;
sensor_msgs::TimeReference imu_time_msg;
ros::Publisher pps_time_pub("/pps/time", &pps_time_msg);
ros::Publisher cam_time_pub("/cam/time", &cam_time_msg);
ros::Publisher imu_time_pub("/imu/time", &imu_time_msg);

// time-sync indicators
time_t rtc_time{0};
elapsedMillis nmea_delay;
elapsedMicros micros_since_pps;
ros::Time pps_stamp, cam_mid_stamp, imu_stamp;
unsigned long cam_open_t_sec, cam_mid_t_sec, cam_close_t_sec;
unsigned long cam_open_t_nsec, cam_mid_t_nsec, cam_close_t_nsec;
volatile bool pub_pps_time = false, send_nmea = false;
volatile bool cam_captured = false, imu_sampled = false;

/*
   Initial setup for the arduino sketch
   This function:
    - Configures timers for LiDAR PPS and camera triggering
    - Advertises and subscribes to ROS topics
    - UART Serial setup for NMEA messages
    - Holds until rosserial is connected
*/
void setup() {
  /* Lidar */

  // set GPSERIAL baud rate and transmission inversion for TTL RS-232
  // transmission
  GPSERIAL.begin(BAUD_RATE, SERIAL_8N1_TXINV);

  // set PPS synch pin
  pinMode(PPS_OUT, OUTPUT);

  /* Camera and IMU */

  // node initialization
  nh.initNode();
  nh.advertise(pps_time_pub);
  nh.advertise(cam_time_pub);
  nh.advertise(imu_time_pub);
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // configure input and output pins
  pinMode(CAM_OPEN_IN, INPUT_PULLUP);
  pinMode(CAM_CLOSE_IN, INPUT_PULLUP);
  pinMode(CAM_OUT, OUTPUT);
  pinMode(IMU_IN, INPUT);
  pinMode(IMU_OUT, OUTPUT);

  // enable interrupts
  attachInterrupt(digitalPinToInterrupt(CAM_OPEN_IN), camOpenISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM_CLOSE_IN), camCloseISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), imuISR, RISING);

  // set write frequency
  analogWriteFrequency(CAM_OUT, 20.0);
  digitalWrite(IMU_OUT, LOW);

  // enable triggers
  analogWrite(CAM_OUT, 5);
  digitalWrite(IMU_OUT, HIGH);

  /* RTC */

  // initialize
  if (!rtc.begin()) {
    nh.loginfo("Couldn't find RTC");
    while (1) delay(10);
  }
  rtc.disable32K();

  // set PPS input pin and write signal
  pinMode(PPS_IN, INPUT_PULLUP);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  // enable interrupt
  attachInterrupt(digitalPinToInterrupt(PPS_IN), ppsISR, RISING);
}

/*
   Main loop
   This function:
    - Transmits NMEA messages over GPSERIAL
    - Publishes the camera capture timestamp to /cam_time
    - Publishes the IMU sample timestamp to /imu_time
*/
void loop() {
  // publish PPS time as a reference for soft-synch
  if (pub_pps_time) {
    pps_time_pub.publish(&pps_time_msg);
    pub_pps_time = false;
  }

  // ensure PPS width satisfied
  if (send_nmea && nmea_delay >= PPS_PULSE_WIDTH) {
    // set PPS pin to low
    digitalWriteFast(PPS_OUT, LOW);

    // ensure min 50 ms width between end of PPS and start of NMEA message
    if (nmea_delay >= PPS_NMEA_MIN_SEPARATION) {
      // get PPS time and adjust to time zone
      const time_t t_sec_gmt = pps_stamp.sec - TIME_ZONE_OFFSET * 3600;

      // create GPRMC sentence
      char time_now[7], date_now[7];
      sprintf(time_now, "%02i%02i%02i", hour(t_sec_gmt), minute(t_sec_gmt),
              second(t_sec_gmt));
      sprintf(date_now, "%02i%02i%02i", day(t_sec_gmt), month(t_sec_gmt),
              year(t_sec_gmt) % 100);
      String gprmc_sentence = F("GPRMC,") + String(time_now) +
                              F(",A,4365.107,N,79347.702,E,022.4,084.4,") +
                              String(date_now) + ",003.1,W";
      String chk = checksum(gprmc_sentence);
      gprmc_sentence = "$" + gprmc_sentence + "*" + chk + "\n";

      // print GPRMC sentence to serial as an NMEA message
      GPSERIAL.print(gprmc_sentence);
      // nh.loginfo(gprmc_sentence.c_str());  // DEBUG

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
  // set RTC time in interrupt for correct ROS time
  if (rtc_time == 0) {
    rtc_time = nh.now().toSec();
    // ros::Time rtc_time_stamp(rtc_time, 0);            // DEBUG
    // printROSTime("RTC Start Time:", rtc_time_stamp);  // DEBUG
  }

  // reset elapsed microseconds
  micros_since_pps = 0;

  // set time of PPS according to RTC clock
  pps_stamp.sec = rtc_time;
  pps_stamp.nsec = 0;
  // printROSTime("PPS Time:", pps_stamp);  // DEBUG

  // set time of PPS time reference with respect to ROS time
  pps_time_msg.header.seq = rtc_time;
  pps_time_msg.header.stamp = nh.now();
  pps_time_msg.time_ref = pps_stamp;

  // toggle to HIGH
  digitalToggleFast(PPS_OUT);

  // counters and resets
  rtc_time++;           // increment time
  pub_pps_time = true;  // enable pps time ref publication
  send_nmea = true;     // enable nmea send
  nmea_delay = 0;       // reset delay counter
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

  // set camera capture time to midway between shutter open and close
  cam_mid_t_sec = cam_open_t_sec;
  cam_mid_t_nsec = (cam_close_t_nsec + cam_open_t_nsec) * 0.5;
  if (cam_close_t_nsec < cam_open_t_nsec) {
    cam_mid_t_nsec =
        cam_open_t_nsec + 0.5 * (cam_close_t_nsec + 1E9 - cam_open_t_nsec);
  }

  cam_mid_stamp.sec = cam_mid_t_sec;
  cam_mid_stamp.nsec = cam_mid_t_nsec;
  cam_captured = true;

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

// Computes XOR checksum of GPRMC sentence
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
