#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include "Trigger.h"
#include "MPU6050.h"

#define GPSERIAL Serial1
using std_srvs::SetBool;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
/* ROS node handler */
ros::NodeHandle nh;
/* Trigger variables for camera */
Trigger camera(2, 0.35, 50000);
std_msgs::Header cam_msg;
ros::Publisher cam_time("cam_time", &cam_msg);
bool publishing = false;
/* Trigger variables for lidar */
Trigger lidar(3, 0.01, 1000000);
bool arduino_pps = true;
/* Trigger variables for imu */
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu0", &imu_msg);
uint32_t imu_sequence = 0;
/* Time for setting RTC */
bool time_set = false;
/* Forward function declarations */
void readIMUData();
void buildAndPublishIMU();
void publishCallback(const std_msgs::Bool &msg);
bool ppsCallback(const SetBool::Request &req, SetBool::Response &res);
String checksum(String msg);
String getTimeNow();
ros::Subscriber<std_msgs::Bool> toggle("/camera/toggle", publishCallback);
ros::ServiceServer<SetBool::Request, SetBool::Response> server("arduino_pps", &ppsCallback);
/* IMU variable definitions */
int intPin = 14;                 // Teensy has multiple interrupt pins
int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;
float aRes, gRes; // scale resolutions per LSB for the sensors
MPU6050lib mpu;

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
  nh.advertise(cam_time);
  nh.advertise(imu_pub);
  nh.advertise(chatter);
  nh.advertiseService(server);
  nh.subscribe(toggle);

  GPSERIAL.begin(9600);
  UART0_C3 = 16;
  Wire.begin();
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050); // Read WHO_AM_I register for MPU-6050
  if (c == 0x68)                                               // WHO_AM_I should always be 0x68
  {
    mpu.MPU6050SelfTest(SelfTest);
    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f)
    {
      mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      mpu.initMPU6050();
    }
    else
    {
      while (1)
        ; // Loop forever if communication doesn't happen
    }
  }
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
    publishing = false;
    camera.sequence = 0;
    imu_sequence = 0;
  }

  nh.spinOnce();
  /* Handle camera triggering and stamping here */
  if (camera.TriggerPin() == true && publishing)
  {
    cam_msg.seq = camera.sequence;
    cam_msg.stamp = nh.now();
    cam_msg.frame_id = "F1";
    cam_time.publish(&cam_msg);
    camera.sequence++;
  }
  //Handle lidar PPS and NMEA sending
  if (arduino_pps)
  {
    if (lidar.TriggerPinImmediate())
    {
      delay(30);
      String time_now = getTimeNow();
      String nmea_string = "GPRMC," + time_now + ",A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A";
      nmea_string = "$" + nmea_string + "*" + checksum(nmea_string) + "\n";
      char string[100];
      nmea_string.toCharArray(string, 100);
      str_msg.data = string;
      chatter.publish(&str_msg);
      GPSERIAL.print(nmea_string);
    }
  }
  /* 
  // Reads data from IMU and stores into variables
  readIMUData();
  uint32_t delt_t = millis() - count;
  // Publish at a rate of 100Hz
  if (delt_t >= 10)
  {
    // Builds IMU ros message from variables and publishes
    buildAndPublishIMU();
    count = millis();
  }*/
}

/***********************************************
 *            Helper functions                 *
 ***********************************************/

/* Read IMU and fill variables */
void readIMUData()
{
  // If data ready bit set, all data registers have new data
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
  {                                // check if data ready interrupt
    mpu.readAccelData(accelCount); // Read the x/y/z adc values
    aRes = mpu.getAres();
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes;
    az = (float)accelCount[2] * aRes;
    mpu.readGyroData(gyroCount); // Read the x/y/z adc values
    gRes = mpu.getGres();
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
    tempCount = mpu.readTempData();                  // Read the x/y/z adc values
    temperature = ((float)tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }
}
/* Build IMU message from variables and publish */
void buildAndPublishIMU()
{
  imu_msg.angular_velocity.x = ((gx / 180) * PI);
  imu_msg.angular_velocity.y = ((gy / 180) * PI);
  imu_msg.angular_velocity.z = ((gz / 180) * PI);
  imu_msg.linear_acceleration.x = (ax * 9.80665);
  imu_msg.linear_acceleration.y = (ay * 9.80665);
  imu_msg.linear_acceleration.z = (az * 9.80665);
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "mpu";
  imu_msg.header.seq = imu_sequence;
  imu_pub.publish(&imu_msg);
  imu_sequence++;
}
/* Callback for /toggle to being publishing */
void publishCallback(const std_msgs::Bool &msg)
{
  publishing = msg.data;
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
