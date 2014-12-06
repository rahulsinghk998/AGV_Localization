#include "robot_localization/navsat_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navsat_transform_node");

  RobotLocalization::NavSatTransform trans;

  trans.run();

  return 0;
}


