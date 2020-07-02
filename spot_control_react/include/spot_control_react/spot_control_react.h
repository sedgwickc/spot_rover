/*  
 * SpotControlReact.h
 * Author: Charles Sedgwick
 * Licence: GPL
 */

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class SpotControlReact
{
  public:

    SpotControlReact(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SpotControlReact();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber behaviour_subscriber_;

    geometry_msgs::Twist latest_vel_msg_;
    //TODO: current behaviour
    //TODO: list of past behaviours?
    //TODO: list of outstanding/upcoming behaviours

    void initParams();
    void processScan(LDP& curr_ldp_scan, const ros::Time& time);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void poseCallback (const sensor_msgs::PoseStamped::ConstPtr& pose_msg);

};

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
