/******************************************************************************
 * spot_control_delib_node.cpp
 * Author: Charles Sedgwick
 * Licence: GPL
 *****************************************************************************/

#include <spot_control_delib/SpotControlDelib.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "spot_control_delib");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //call constructor of SpotControlReact which sets up subscribers/publishers
  SpotControlDelib spot_control_delib(nh, nh_private);
  ros::spin();
  return 0;
}
