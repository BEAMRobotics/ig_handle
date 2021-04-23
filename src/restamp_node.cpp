#include <ros/ros.h>
#include <nodelet/loader.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "restamp_node");

  ros::NodeHandle nh;

  // This is code based nodelet loading, the preferred nodelet launching is done
  // through roslaunch
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string node_name = ros::this_node::getName();
  std::string nodelet_name;
  if (!nh.getParam("nodelet_name", nodelet_name)) {
    ROS_ERROR("Please provide a nodelet name to load");
  } else {
    nodelet.load(node_name, "ig_handle/" + nodelet_name, remap, nargv);
  }

  ros::spin();

  return 0;
}
