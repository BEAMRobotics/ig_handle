#include <ros/ros.h>
#include <stdlib.h>
#include <signal.h>

// virtual box, CLI extension, and XP snapshot must all installed/available.

void shutdown(int sig)
{
    system("VBoxManage controlvm \"XP\" poweroff");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_virtual_machine");
    ros::NodeHandle nh;

    ROS_INFO("sonar_virtual_machine initialize");

    system("VBoxManage startvm \"XP\"");
    signal(SIGINT, shutdown);

    ros::spin();
    return 0;
}