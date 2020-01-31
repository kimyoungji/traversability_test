/*
 * traversability_test_node.cpp
 */

#include <ros/ros.h>
#include "traversability_test/TMapping.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traversability_test");
  ros::NodeHandle nodeHandle("~");
  traversability_test::TMapping mapping(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(0);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
