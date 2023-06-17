#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <TimeLib.h>

#define USE_USBCON

// Electrical component pin numbers
#define GPSERIAL Serial1  // GPRMC
#define PPS_PIN 2         // PPS
#define CAM_OUT 3         // Cam_Trig
#define CAM_IN 4          // Cam_Exp
#define IMU_IN 8          // IMU_SyncIn
#define IMU_OUT 9         // IMU_SyncOut

// ROS node handler
ros::NodeHandle nh;

// time-sync clock
IntervalTimer teensy_clock;

// time topics for camera and imu
sensor_msgs::TimeReference cam_time_msg;
sensor_msgs::TimeReference imu_time_msg;
ros::Publisher CAM_time("/cam/cam_time", &cam_time_msg);
ros::Publisher IMU_time("/imu/imu_time", &imu_time_msg);

// time-sync indicators
elapsedMillis nmea_delay;
ros::Time PPS_stamp, CAM_stamp, IMU_stamp;
volatile bool sendNMEA = false, CAM_closed = false, IMU_sampled = false;

// Forward function declarations
void setSendNMEA_ISR(void);
void CAM_ISR(void);
void IMU_ISR(void);
String checksum(String msg);
void debugNMEA();

/*
 * Initial setup for the arduino sketch
 * This function:
 *  - Configures timers for LiDAR and camera triggering
 *  - Advertises and subscribes to ROS topics
 *  - UART Serial setup for NMEA strings
 *  - Holds until rosserial is connected
 */
void setup() {
  /* Lidar */

  // set GPSERIAL baud rate and transmission inversion for TTL RS-232
  // transmission
  GPSERIAL.begin(9600, SERIAL_8N1_TXINV);

  // set PPS pin
  pinMode(PPS_PIN, OUTPUT);

  // begin clock and call setSendNMEA_ISR every second
  teensy_clock.begin(setSendNMEA_ISR, 1000000);

  /* Camera and IMU */

  // node initialization
  nh.initNode();
  nh.advertise(CAM_time);
  nh.advertise(IMU_time);

  // configure input and out pins
  pinMode(CAM_IN, INPUT_PULLUP);
  pinMode(CAM_OUT, OUTPUT);
  pinMode(IMU_IN, INPUT);
  pinMode(IMU_OUT, OUTPUT);

  // enable interrupts
  attachInterrupt(digitalPinToInterrupt(CAM_IN), CAM_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), IMU_ISR, RISING);

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
 *  - Triggers lidar line (PPS) and transmits NMEA string over GPSERIAL
 *  - Triggers camera line and publishes the timestamp to /cam_time
 *  - Publishes the timestamp of IMU capture to /imu_time
 */
void loop() {
  // ensure min 50 ms width between end of PPS and start of NMEA message
  if (sendNMEA && nmea_delay >= 55) {
    // get PPS time
    time_t t_sec = PPS_stamp.sec;
    time_t t_nsec = PPS_stamp.nsec;
    int t_msec = round(t_nsec / 1000000);  // TODO: fix rounding

    // create NMEA string
    char millisec_now[9], time_now[7], date_now[7];
    sprintf(millisec_now, "%d", t_msec);
    sprintf(time_now, "%02i%02i%02i", hour(t_sec), minute(t_sec),
            second(t_sec));
    sprintf(date_now, "%02i%02i%02i", day(t_sec), month(t_sec),
            year(t_sec) % 100);
    String nmea_string = F("GPRMC,") + String(time_now) + "." +
                         String(millisec_now) +
                         F(",A,4365.107,N,79347.702,E,022.4,084.4,") +
                         String(date_now) + ",003.1,W";
    String chk = checksum(nmea_string);
    nmea_string = "$" + nmea_string + "*" + chk + "\n";

    // print NMEA string to serial as an NMEA message
    GPSERIAL.print(nmea_string);

    // reset send
    sendNMEA = false;

    // set PPS pin to low
    digitalWriteFast(PPS_PIN, LOW);

    // DEBUG NMEA
    debugNMEA(nmea_string, t_sec, t_nsec);
  }

  if (CAM_closed == true) {
    CAM_closed = false;
    cam_time_msg.time_ref = CAM_stamp;
    CAM_time.publish(&cam_time_msg);
  }

  if (IMU_sampled == true) {
    IMU_sampled = false;
    imu_time_msg.time_ref = IMU_stamp;
    IMU_time.publish(&imu_time_msg);
  }

  nh.spinOnce();
}

// Send NMEA interrupt
void setSendNMEA_ISR(void) {
  // get time of PPS
  PPS_stamp = nh.now();

  /* It's not really recommended to write to pins from an interrupt as it can
   * take a relatively long time to execute. However, one of the main purposes
   * of this code is to output accurate PPS signal and this is the most accurate
   * way that abides by the format constraints on the signal. In testing, the
   * default digitalWriteFast pulse width satisfies the 10 us - 200ms width
   * required by the VLP16
   */
  digitalWriteFast(PPS_PIN, HIGH);

  // enable send and reset delay counter
  sendNMEA = true;
  nmea_delay = 0;
}

// Timestamp creation interrupts
void CAM_ISR(void) {
  CAM_stamp = nh.now();
  CAM_closed = true;
}

void IMU_ISR(void) {
  IMU_stamp = nh.now();
  IMU_sampled = true;
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

void debugNMEA(String& nmea_string, time_t& t_sec, time_t& t_nsec) {
  // convert ros time to string
  char t_sec_string[10], t_nsec_string[9];
  sprintf(t_sec_string, "%lld", (long long)t_sec);
  sprintf(t_nsec_string, "%lld", (long long)t_nsec);
  String ros_time_string = String(t_sec_string) + "." + String(t_nsec_string);

  // print NMEA summary
  nh.loginfo("ROS time:");
  nh.loginfo(ros_time_string.c_str());
  nh.loginfo("NMEA string:");
  nh.loginfo(nmea_string.c_str());
}
