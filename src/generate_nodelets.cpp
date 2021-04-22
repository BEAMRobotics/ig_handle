#include <ig_handle/restamp_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <wfov_camera_msgs/WFOVImage.h>

namespace ig_handle {

using imu_restamp_nodelet = ig_handle::restamp_nodelet<sensor_msgs::Imu>;
using camera_restamp_nodelet =
    ig_handle::restamp_nodelet<wfov_camera_msgs::WFOVImage>;

PLUGINLIB_EXPORT_CLASS(ig_handle::imu_restamp_nodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ig_handle::camera_restamp_nodelet, nodelet::Nodelet)

}  // namespace ig_handle