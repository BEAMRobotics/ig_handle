#include <iostream>
#include <string>

namespace ig_handle {

template <typename T>
void restamp_nodelet<T>::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  pnh.getParam("max_buffer_size", max_buffer_size);

  if (!pnh.getParam("data_topic", data_topic_) ||
      !pnh.getParam("time_topic", time_topic_)) {
    ROS_ERROR("Please provide a data_topic and time_topic parameter.");
    return;
  }

  data_buffer_.reserve(max_buffer_size);
  time_buffer_.reserve(max_buffer_size);

  data_sub_ = nh.subscribe(data_topic_, 10,
                           &ig_handle::restamp_nodelet<T>::dataCb, this);
  time_sub_ = nh.subscribe(time_topic_, 10,
                           &ig_handle::restamp_nodelet<T>::timeCb, this);

  pub_ = pnh.advertise<T>(data_topic_ + "/stamped", 100);
  ROS_INFO_STREAM("Restamping " << data_topic_ << " with " << time_topic_
                                << " and publishing on " << data_topic_
                                << "/stamped.");
};

template <typename T>
void restamp_nodelet<T>::dataCb(const typename T::ConstPtr& msg) {
  if (data_buffer_.size() >= max_buffer_size) {
    ROS_ERROR_STREAM("Data buffer at max size.  Data with sequence "
                     << msg->header.seq << " will be dropped.");
    return;
  }

  // Lock buffer and push
  data_mutex_.lock();
  data_buffer_.push_back(msg);
  data_mutex_.unlock();
  ROS_DEBUG_STREAM("Added data with sequence " << msg->header.seq << ":"
                                               << msg->header.stamp.sec << "."
                                               << msg->header.stamp.nsec);
  synchronizeAndPublish();
}

template <typename T>
void restamp_nodelet<T>::timeCb(const std_msgs::Header::ConstPtr& msg) {
  if (data_buffer_.size() >= max_buffer_size) {
    ROS_ERROR_STREAM("Time buffer at max size.  Timestamp with sequence "
                     << msg->seq << " will be dropped.");
    return;
  }
  // Lock buffer and push
  time_mutex_.lock();
  time_buffer_.push_back(msg);
  time_mutex_.unlock();
  ROS_DEBUG_STREAM("Added timestamp with sequence " << msg->seq << ":"
                                                    << msg->stamp.sec << "."
                                                    << msg->stamp.nsec);
  synchronizeAndPublish();
}

template <typename T>
bool restamp_nodelet<T>::synchronizeAndPublish() {
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

    seq_offset = data_buffer_[0]->header.seq - time_buffer_[0]->seq;
    seq_offset_set = true;
  }

  auto i = time_buffer_.begin();

  while (i != time_buffer_.end()) {
    // if data is ahead, a data message was dropped, trim timestamps
    if ((*i)->seq + seq_offset < data_buffer_[0]->header.seq) {
      ROS_INFO_STREAM("Data with sequence " << (*i)->seq << " was not found.");
      time_buffer_.erase(i);

      // if sequence aligns, publish
    } else if ((*i)->seq + seq_offset == data_buffer_[0]->header.seq) {
      T new_message = *(data_buffer_[0]);
      new_message.header.stamp = (*i)->stamp;
      ROS_DEBUG_STREAM("publishing data seq #" << data_buffer_[0]->header.seq);

      time_buffer_.erase(i);
      data_buffer_.erase(data_buffer_.begin());

      pub_.publish(new_message);
      return true;

      // if timestamps are ahead, a timestamp message was droppde, trim data
    } else if ((*i)->seq + seq_offset > data_buffer_[0]->header.seq) {
      ROS_INFO_STREAM("Timestamp with sequence " << (*i)->seq
                                                 << " was not found.");
      data_buffer_.erase(data_buffer_.begin());

      return false;
    }
  }
}

}  // namespace ig_handle