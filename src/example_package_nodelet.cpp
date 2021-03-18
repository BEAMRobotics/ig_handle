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

#include <example_package/example_package_nodelet.h>

#include <stdlib.h>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>


namespace example_package {

ExamplePackageNodelet::ExamplePackageNodelet() {}

void ExamplePackageNodelet::onInit() {
    this->nh_ = getNodeHandle();
    this->private_nh_ = getPrivateNodeHandle();
    this->LoadParams();
    this->example_publisher_ = nh_.advertise<std_msgs::String>("chatter", 1000);

    // Set nodelet's internal logger
    this->SetInternalLogger();

    this->timer_ =
      this->nh_.createTimer(ros::Duration(this->publishing_period_),
                            boost::bind(&ExamplePackageNodelet::timeCb, this));
}

void ExamplePackageNodelet::LoadParams() {
    if (nh_.getParam("example_package/publish_period",
                     this->publishing_period_)) {
        ROS_INFO("Loaded publish_period: %f", this->publishing_period_);
    } else {
        ROS_INFO("Could not load publish_period");
    }

    if (nh_.getParam("example_package/log_directory", this->log_directory_)) {
        ROS_INFO("Loaded log_directory: %s", this->log_directory_.c_str());
    } else {
        ROS_INFO("Could not load log_directory");
    }
}

// Set a logger to catch the errors published by the internal C++ library so
// that you don't spam the console!
void ExamplePackageNodelet::SetInternalLogger() {
    boost::log::add_common_attributes();

    // Set the logging directory
    // TODO(msmart) clean this up so that it doesn't use '/' convention for
    // files going to HOME. Maybe check for first character in parameter being
    // '/' and then treat as absolute, or relative if '/' is not first
    // character.
    std::string log_path = getenv("HOME") + this->log_directory_;

    // TODO(msmart) add more complicated example demonstrating tiers of severity
    // logging. For now - just drop anything less than info.

    // Filter based on logging severity
    boost::log::core::get()->set_filter(
      // trace and debug level log events are filtered out
      boost::log::trivial::severity >= boost::log::trivial::info);

    // Define the desired log file.
    // TODO(msmart) - fix this formatting so lint stops yelling.
    boost::log::add_file_log(
      // Set log file name
      boost::log::keywords::file_name =
        log_path + "/example_package_nodelet_%N.log",
      // Set log file rotation size in bytes.
      // These values are low here for example.
      boost::log::keywords::rotation_size = 512,
      // Set log entry format
      boost::log::keywords::format = "[%TimeStamp%]: %Message%"
      // Call make_collector off of the sink created by add_file_log
      )
      ->locked_backend()
      ->set_file_collector(
        // Collectors are only applied after a log file is closed.
        // Each time a log file is closed, the collector checks to
        // see if an action needs to be taken relative to the next
        // log file.
        boost::log::sinks::file::make_collector(
          // 'target' sets the folder that will be "managed"
          // by overwriting logs to maintain following
          // objective.
          boost::log::keywords::target = log_path,
          // If the logs being created in total exceed
          // max_size, then the next log file created will
          // overwrite the first log file.
          boost::log::keywords::max_size = 5 * 512));
}

void ExamplePackageNodelet::timeCb() {
    // Functions from the ExampleClass do all of the work.
    // The nodelet just handles message passing

    // Call function from the C++ library ExampleClass
    //      Does some work outside of ROS, returns a value
    int value_from_foo = this->foobar.Foo();

    // Publish result from Foo
    //      Bring the result into ROS
    std_msgs::String msg;
    msg.data = "Foo() gave " + std::to_string(value_from_foo);
    this->example_publisher_.publish(msg);

    // Call processing function bar to do some more work outside of ROS
    // note: Bar() returns an error code. 0 = success.
    //
    // If the call to Bar() triggers an error, diagnostic information
    // is logged to the file set by
    // ExamplePackageNodelet::SetInternalLogger()
    // instead of spamming the console
    int bar_result = this->foobar.Bar(value_from_foo);

    // Was Bar successfull?
    if (bar_result == 1) {
        // We had an error!
        ROS_ERROR("Function ExampleClass::Bar() failed!...");

        // Do some error handling work within ROS if applicable.
        return;
    }

    // No errors. Continue working...
}

}  // namespace example_package

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_package::ExamplePackageNodelet, nodelet::Nodelet);
