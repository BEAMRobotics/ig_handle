#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <boost/thread.hpp>

namespace ig_handle {

template <typename T>
class restamp_nodelet : public nodelet::Nodelet {
 public:
  restamp_nodelet() = default;

 private:
  virtual void onInit();

  void dataCb(const typename T::ConstPtr& msg);
  void timeCb(const std_msgs::Header::ConstPtr& msg);

  bool synchronizeAndPublish();

  int max_buffer_size;

  std::string data_topic_;
  std::string time_topic_;
  std::vector<typename T::ConstPtr> data_buffer_;
  std::vector<std_msgs::Header::ConstPtr> time_buffer_;
  boost::mutex data_mutex_;
  boost::mutex time_mutex_;

  int seq_offset;
  bool seq_offset_set = false;

  ros::Subscriber data_sub_;
  ros::Subscriber time_sub_;
  ros::Publisher pub_;
};
}  // namespace ig_handle

#include "ig_handle/impl/restamp_nodelet.hpp"