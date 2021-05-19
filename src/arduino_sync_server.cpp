#include <ig_handle/arduino_sync_server.h>

namespace ig_handle {

arduino_sync_action::arduino_sync_action(std::string name) :
    as_(nh_, name, boost::bind(&arduino_sync_action::executeCB, this, _1), false),
    action_name_(name) {
        as_.start();
    }

void arduino_sync_action::executeCB(const ig_handle::arduino_syncGoal::ConstPtr &goal) {
    ROS_DEBUG_STREAM("Received an arudino sync action request with initial time: " << goal->t0);
    
    result_.t0 = goal->t0;
    result_.t1 = ros::Time::now();
    
    as_.setSucceeded(result_);    
    ROS_DEBUG_STREAM("Returned an arduino sync action response with ROS time: " << result_.t1);
}

} // namespace ig_handle


int main(int argc, char** argv) {
    ros::init(argc, argv, "arduino_sync");

    ig_handle::arduino_sync_action arduino_sync("arduino_sync");
    ros::spin();

    return 0;
}