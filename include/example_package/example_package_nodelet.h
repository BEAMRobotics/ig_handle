/* Copyright (c) 2019, BEAM Robotics,
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |     _ _ __ _ _      _ _ _ _ _ _ _    _ _ _ _ _ _       _ _         _ _     |
 |   |     _ _   \    |     _ _ _ _|   /            \    /    \      /    \   |
 |   |    |   \   \   |    |           |     _ _     |   |     \_ _ /     |   |
 |   |    |   |   |   |    |           |    |   |    |   |                |   |
 |   |    |   /   /   |    |_ _ _ _    |    |_ _|    |   |                |   |
 |   |    |  /   /    |            |   |             |   |    /\ _ /\     |   |
 |   |    |  \   \    |     _ _ _ _|   |     _ _     |   |    |      |    |   |
 |   |    |   \   \   |    |           |    |   |    |   |    |      |    |   |
 |   |    |   |   |   |    |           |    |   |    |   |    |      |    |   |
 |   |    |_ _/   /   |    |_ _ _ _    |    |   |    |   |    |      |    |   |
 |   |_ _ _ _ _ _/    |_ _ _ _ _ _ |   | _ _|   |_ _ |   | _ _|      | _ _|   |
 |  _________________________________________________________________________ |
 | |____________________________________________________________________/___/ |
 |   |    _ _     _ _     _ _     _ _   _ _ _   _ _      _ _     _ _     ||   |
 |   |   |_ _|   /   \   |_ _|   /   \    |      |     /    `  /_ _ `    ||   |
 |  _|__ |  \ __ \_ _/ _ |_ _| _ \_ _/  _ |   _ _|_  _ \ _ _.  ._ _/ ____||_  |
 | |___________________________________________________________________/___/  |                                                                           |
 ******************************************************************************
 * ############################################################################
 *
 * File: example_package_node.cpp
 * Desc: Code that allows the example_package_nodelet to be run as a node for
 *       convenience.
 * Auth: Nick Charron
 *
 * ############################################################################
*/

#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include <example_package/example_class/example_class.h>

namespace example_package {

class ExamplePackageNodelet : public nodelet::Nodelet {
 public:
    // Nodelet Constructor
    // MUST NEVER FAIL
    ExamplePackageNodelet();

 private:
    // Initialization Function
    // Can fail?
    virtual void onInit();

    void LoadParams();

    // Set logging sink to be used by boost::log.
    //      Only used for code internal to the nodelet (non-ROS code)
    void SetInternalLogger();

    // Timer callback to enforce publishing rate
    void timeCb();

    // Nodehandles, both public and private
    ros::NodeHandle nh_, private_nh_;

    // Publisher
    ros::Publisher example_publisher_;

    // Timer for establishing publishing rate
    ros::Timer timer_;

    // Publishing Period
    double publishing_period_;

    // Log directory relative to the users $HOME directory
    std::string log_directory_;

    // INTERNAL OBJECT OF Cpp Class (Bulk of work should be done in here)
    ExampleClass foobar;
};

}  // namespace example_package

