#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <TimeLib.h>

#define USE_USBCON

// Electrical component pin numbers
#define GPSERIAL Serial1  // $GPRMC
#define PPS_PIN 2         // PPS
#define CAM_OUT 3         // Cam_Trig
#define CAM_IN 4          // Cam_Exp
#define IMU_IN 8          // IMU_SyncIn
#define IMU_START 9       // IMU_SyncOut

// ROS node handler
ros::NodeHandle nh;

// Trigger variables for camera and imu
sensor_msgs::TimeReference cam_time_msg;
sensor_msgs::TimeReference imu_time_msg;
ros::Publisher CAM_time("/cam/cam_time", &cam_time_msg);
ros::Publisher IMU_time("/imu/imu_time", &imu_time_msg);
IntervalTimer teensy_clock;
ros::Time CAM_close_stamp, IMU_stamp;
volatile bool sendNMEA = false, CAM_closed = false, IMU_sampled = false;

// Forward function declarations
void CAM_ISR(void);
void IMU_ISR(void);
void setSendNMEA(void);
String checksum(String msg);

/*
 * Initial setup for the arduino sketch
 * This function:
 *  - Configures timers for LiDAR and camera triggering
 *  - Advertises and subscribes to ROS topics
 *  - UART Serial setup for NMEA strings
 *  - Holds until rosserial is connected
 */
void setup() {
  /* Setup input/outputs to LiDAR
   *  - Set baud rate for GPSERIAL
   *  - PPS but don't start it yet
   *  - Attach
   */
  Serial1.begin(57600);
  pinMode(PPS_PIN, OUTPUT);  // 50% duty cycle

  // begin clock and call setSendNMEA_ISR every 10^6 microseconds
  teensy_clock.begin(setSendNMEA_ISR, 1000000);

  // node initialization
  nh.initNode();
  nh.advertise(CAM_time);
  nh.advertise(IMU_time);

  // configure input pins
  pinMode(CAM_IN, INPUT_PULLUP);
  pinMode(IMU_IN, INPUT);

  // enable interrupts
  attachInterrupt(digitalPinToInterrupt(CAM_IN), CAM_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), IMU_ISR, RISING);

  // set up the camera triggers but don't start them yet either
  // Note: We're using a PWM signal because it's a way of offloading
  //       the task to free up the main loop. The base frequency for
  //       the PWM signal is 20.0 Hz
  pinMode(CAM_OUT, OUTPUT);
  analogWriteFrequency(CAM_OUT, 20.0);

  // setup IMU_START ping as output
  pinMode(IMU_START, OUTPUT);

  // ensure pin is low before we send a rising edge
  digitalWrite(IMU_START, LOW);

  // await node handle time sync
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // enable triggers
  nh.loginfo("Setup complete, Enabling triggers.");
  analogWrite(CAM_OUT, 5);  // 5% duty cycle @ 20 Hz = 2.5 ms pulse
  digitalWrite(IMU_START, HIGH);
  nh.loginfo("Triggers enabled.");
}

/*
 * Main loop
 * This function:
 *  - Triggers lidar line (PPS) and transmits NMEA string over Serial1
 *  - Triggers camera line at certain frequency and publishes the timestamp to
 * /cam_time
 *  - Publishes the timestamp of IMU capture to /imu_time
 */
void loop() {
  if (sendNMEA == true) {
    char time_now[7], date_now[7];
    time_t t = nh.now().sec;
    sprintf(time_now, "%02i%02i%02i", hour(t), minute(t), second(t));
    sprintf(date_now, "%02i%02i%02i", day(t), month(t), year(t) % 100);
    String nmea_string = F("GPRMC,") + String(time_now) +
                         F(",A,4365.107,N,79347.702,E,022.4,084.4,") +
                         String(date_now) + ",003.1,W";
    String chk = checksum(nmea_string);
    nmea_string = "$" + nmea_string + "*" + chk + "\n";
    GPSERIAL.print(nmea_string);
    sendNMEA = false;

    // minimum pulse duration required by LiDAR is 10 us
    digitalWriteFast(PPS_PIN, LOW);

    // DEBUG NMEA
    // nh.loginfo(nmea_string.c_str());
  }

  if (CAM_closed == true) {
    CAM_closed = false;
    cam_time_msg.time_ref = CAM_close_stamp;
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
  /* It's not really recommended to write to pins from an interrupt as it can
   * take a relatively long time to execute. However, one of the main purposes
   * of this code is to output accurate PPS signal and this is the most accurate
   * way that abides by the format constraints on the signal.
   */
  digitalWriteFast(PPS_PIN, HIGH);
  sendNMEA = true;
}

// Timestamp creation interrupts
void CAM_ISR(void) {
  CAM_close_stamp = nh.now();
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
