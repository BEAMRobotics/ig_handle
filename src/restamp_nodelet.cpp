#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include <boost/thread.hpp>

#include "nodelet/nodelet.h"
#include "ros/ros.h"

#define MAX_BUFFER_SIZE 20

namespace ig_handle {

class restamp_nodelet : public nodelet::Nodelet {
 public:
  restamp_nodelet() = default;

 private:
  virtual void onInit() {
    std::cout << "in nodelet" << std::endl;
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    data_buffer_.reserve(MAX_BUFFER_SIZE);
    time_buffer_.reserve(MAX_BUFFER_SIZE);

    data_sub_ = nh.subscribe("/imu/data", 10,
                             &ig_handle::restamp_nodelet::dataCb, this);
    time_sub_ = nh.subscribe("/imu/imu_time", 10,
                             &ig_handle::restamp_nodelet::timeCb, this);

    pub_ = pnh.advertise<sensor_msgs::Imu>("/imu/data/sync", 10);
  };

  void dataCb(const sensor_msgs::Imu::ConstPtr& msg) {
    // Lock buffer and push
    data_mutex_.lock();
    data_buffer_.push_back(msg);
    data_mutex_.unlock();
    ROS_DEBUG_STREAM("Added data with sequence " << msg->header.seq << ":"
                                                 << msg->header.stamp);
    synchronizeAndPublish();
  }

  void timeCb(const std_msgs::Header::ConstPtr& msg) {
    // Lock buffer and push
    time_mutex_.lock();
    time_buffer_.push_back(msg);
    time_mutex_.unlock();
    ROS_DEBUG_STREAM("Added timestamp with sequence " << msg->seq << ":"
                                                      << msg->stamp);
    synchronizeAndPublish();
  }

  bool synchronizeAndPublish() {
    boost::mutex::scoped_lock scopedDataLock(data_mutex_);
    boost::mutex::scoped_lock scopedTimeLock(time_mutex_);

    // return if one buffer is empty
    if (time_buffer_.size() < 1 || data_buffer_.size() < 1) {
      return false;
    }

    // set seq_offset the first time we can
    if (!seq_offset_set) {
      time_buffer_.erase(time_buffer_.begin(), std::prev(time_buffer_.end()));
      data_buffer_.erase(data_buffer_.begin(), std::prev(data_buffer_.end()));

      std::cout << time_buffer_.size() << std::endl;
      std::cout << data_buffer_.size() << std::endl;

      seq_offset = data_buffer_[0]->header.seq - time_buffer_[0]->seq;
      seq_offset_set = true;
    }

    auto i = time_buffer_.begin();

    while (i != time_buffer_.end()) {
      // if data is ahead, trim timestamps
      if ((*i)->seq + seq_offset < data_buffer_[0]->header.seq) {
        time_buffer_.erase(i);

        // if sequence aligns, publish
      } else if ((*i)->seq + seq_offset == data_buffer_[0]->header.seq) {
        sensor_msgs ::Imu new_message = *(data_buffer_[0]);
        new_message.header.stamp = (*i)->stamp;

        time_buffer_.erase(i);
        data_buffer_.erase(data_buffer_.begin());

        pub_.publish(new_message);
        return true;

        // if timestamps are ahead, wait for data buffer to fill.
      } else if ((*i)->seq + seq_offset > data_buffer_[0]->header.seq) {
        return false;
      }
    }
  }

  std::vector<sensor_msgs::Imu::ConstPtr> data_buffer_;
  std::vector<std_msgs::Header::ConstPtr> time_buffer_;
  boost::mutex data_mutex_;
  boost::mutex time_mutex_;

  int seq_offset;
  bool seq_offset_set = false;

  ros::Subscriber data_sub_;
  ros::Subscriber time_sub_;
  ros::Publisher pub_;
};

PLUGINLIB_EXPORT_CLASS(ig_handle::restamp_nodelet,
                       nodelet::Nodelet)  // Needed for Nodelet declaration

}  // namespace ig_handle