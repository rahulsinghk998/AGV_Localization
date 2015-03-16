#include "robot_localization/ros_filter_types.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_navigation_node");

  RobotLocalization::RosEkf ekf;

  ekf.run();

  return 0;
}

