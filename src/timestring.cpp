#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>

std::string time_topic_;

void timeCb(const std_msgs::Header::ConstPtr &msg) {
  boost::posix_time::ptime my_posix_time = ros::Time(msg->stamp).toBoost();
  std::string iso_time_str =
      boost::posix_time::to_iso_extended_string(my_posix_time);
  ROS_ERROR_STREAM(time_topic_ << ": " << iso_time_str << std::endl);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "timestring");
  ros::NodeHandle nh;

  if (!ros::param::get("time_topic", time_topic_)) {
    ROS_ERROR_STREAM(std::endl << time_topic_ << std::endl);
    ROS_ERROR("Please provide a time_topic parameter.");
    return 0;
  }

  ros::Subscriber time_sub_ = nh.subscribe(time_topic_, 10, timeCb);

  ROS_INFO_STREAM("Logging " << time_topic_ << " as a string." << std::endl);
  ros::spin();

  return 0;
};
