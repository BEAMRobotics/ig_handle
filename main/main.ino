#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <FrequencyTimer2.h>
#include <TimeLib.h>

#define GPSERIAL Serial1
#define PPS_PIN 6
#define IMU_IN 12
#define CAM1_OUT 2
#define CAM2_OUT 7
#define CAM3_OUT 8
#define CAM4_OUT 3
#define CAM1_IN 9
#define CAM2_IN 10
#define CAM3_IN 11
#define CAM4_IN 24
#define IMU_START 23

/* ROS node handler */
ros::NodeHandle nh;
/* Trigger variables for camera */
sensor_msgs::TimeReference time_msg;
ros::Publisher F1_time("/F1/cam_time", &time_msg);
ros::Publisher F2_time("/F2/cam_time", &time_msg);
ros::Publisher F3_time("/F3/cam_time", &time_msg);
ros::Publisher F4_time("/F4/cam_time", &time_msg);
ros::Publisher IMU_time("/imu/imu_time", &time_msg);
volatile bool sendNMEA = false, F1_closed = false, F2_closed = false,
              F3_closed = false, F4_closed = false, IMU_sampled = false;
ros::Time F1_close_stamp, F2_close_stamp, F3_close_stamp, F4_close_stamp, IMU_stamp;

///* Forward function declarations */
void cam1_ISR(void);
void cam2_ISR(void);
void cam3_ISR(void);
void cam4_ISR(void);
void IMU_ISR(void);
void setSendNMEA(void);
void enableTriggers();
String checksum(String msg);

/*
 * Initial setup for the arduino sketch
 * This function performs:
 *  - Configure timers for LiDAR and camera triggering
 *  - Advertisement and subscribing to ROS topics
 *  - UART Serial setup for NMEA strings
 *  - Holds until rosserial is connected
 */
void setup() {  
  /* Setup outputs to LiDAR
   *    - PPS but don't start it yet
        - Attach */
  pinMode(PPS_PIN, OUTPUT);  // pin 5 is still driven by default but has a 50% duty cycle
  FrequencyTimer2::setPeriod(1000000);  // 10^6 microseconds, 1 second
  FrequencyTimer2::setOnOverflow(setSendNMEA_ISR);  // sets the function that runs when the timer overflows
  
  // node initialization
  nh.initNode();
  nh.advertise(F1_time);
  nh.advertise(F2_time);
  nh.advertise(F3_time);
  nh.advertise(F4_time);
  nh.advertise(IMU_time);


 /* configure input pins */
  pinMode(CAM1_IN, INPUT_PULLUP);
  pinMode(CAM2_IN, INPUT_PULLUP);
  pinMode(CAM3_IN, INPUT_PULLUP);
  pinMode(CAM4_IN, INPUT_PULLUP);
  pinMode(IMU_IN, INPUT);

  /* enable interrupts */
  attachInterrupt(digitalPinToInterrupt(CAM1_IN), cam1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM2_IN), cam2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM3_IN), cam3_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM4_IN), cam4_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU_IN), IMU_ISR, RISING);

  /* set up the camera triggers but don't start them yet either */
  analogWriteFrequency(CAM1_OUT, 20.0);  // 20.0 Hz base frequency for the PWM signal
  analogWriteFrequency(CAM2_OUT, 20.0);  // We're using a PWM signal because it's a way of offloading
  analogWriteFrequency(CAM3_OUT, 20.0);  // the task to free up the main loop
  analogWriteFrequency(CAM4_OUT, 20.0);
  /* setup IMU_START ping as output */
  pinMode(IMU_START, OUTPUT);
  digitalWrite(IMU_START, LOW); // make sure that the pin is low before we send a rising edge

//  await nodehandle time sync
  while (nh.now().sec < 100000) {
    nh.spinOnce(); 
  }
  
  /* start sampling */
  nh.loginfo("Setup complete, enabling triggers");
  enableTriggers();
  // Serial.begin(57600); // This takes some time so commenting it out should speed up the startup
}


/*
 * Continously looping function performs the following:
 *  - Triggering camera line at certain frequency and publishes the timestamp to
 * /cam_time
 *  - Trigger lidar line (PPS) and transmits NMEA string over Serial1
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
    digitalWriteFast(PPS_PIN,
                     LOW);  // minimum pulse duration required by LiDAR is 10 us
                     

//    nh.loginfo(nmea_string.c_str()); //debug nmea string in rosconsole
  }
  if (F1_closed == true) {
    F1_closed = false;
    time_msg.time_ref = F1_close_stamp;
    F1_time.publish(&time_msg);
  }
  if (F2_closed == true) {
    F2_closed = false;
    time_msg.time_ref = F2_close_stamp;
    F2_time.publish(&time_msg);
  }
  if (F3_closed == true) {
    F3_closed = false;
    time_msg.time_ref = F3_close_stamp;
    F3_time.publish(&time_msg);
  }
  if (F4_closed == true) {
    F4_closed = false;
    time_msg.time_ref = F4_close_stamp;
    F4_time.publish(&time_msg);
  }
  if (IMU_sampled == true) {
    IMU_sampled = false;
    time_msg.time_ref = IMU_stamp;
    IMU_time.publish(&time_msg);
  }

  nh.spinOnce();
}

/***********************************************
 *            Helper functions                 *
 ***********************************************/
void setSendNMEA_ISR(void) {
  /*  It's not really recommended to write to pins from an interrupt as it can
   * take a relatively long time to execute. However, one of the main purposes
   * of this code is to output accurate PPS signal and this is the most accurate
   * way that abides by the format constraints on the signal.
   */
  digitalWriteFast(PPS_PIN, HIGH);
  sendNMEA = true;
//  microsSincePPS = 0;
}

// Timestamp creation interrupts
void cam1_ISR(void) {
  F1_close_stamp = nh.now();
  F1_closed = true;
}
void cam2_ISR(void) {
  F2_close_stamp = nh.now();
  F2_closed = true;
}
void cam3_ISR(void) {
  F3_close_stamp = nh.now();
  F3_closed = true;
}
void cam4_ISR(void) {
  F4_close_stamp = nh.now();
  F4_closed = true;
}
void IMU_ISR(void) {
  IMU_stamp = nh.now();
  IMU_sampled = true;
}

/* Computes XOR checksum of NMEA sentence */
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

void enableTriggers() {
  /* await even second */
  // By switching to nh.now() we shouldn't need this wait
  // while (!(micros() % 1000000) {
  // }
  FrequencyTimer2::enable();

  /* start sampling */
  analogWrite(CAM1_OUT, 5);  // 5% duty cycle @ 20 Hz = 2.5 ms pulse
  analogWrite(CAM2_OUT, 5);
  analogWrite(CAM3_OUT, 5);
  analogWrite(CAM4_OUT, 5);
  digitalWrite(IMU_START, HIGH);
}
