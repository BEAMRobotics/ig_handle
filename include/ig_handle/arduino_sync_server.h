#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ig_handle/arduino_syncAction.h>

namespace ig_handle
{
class arduino_sync_action {
protected: 
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<arduino_syncAction> as_;
    std::string action_name_;
    ig_handle::arduino_syncResult result_;

public: 
    arduino_sync_action(std::string name);

    void executeCB(const ig_handle::arduino_syncGoal::ConstPtr &goal);


};

} // namespace ig_handle