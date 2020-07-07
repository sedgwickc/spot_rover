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

    ros::Subscriber teleop_subscriber_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber behaviour_subscriber_;

    ros::Publisher cmd_vel_publisher_;

    // set by scanCallback, accessed by safeToMove()
    sensor_msgs::LaserScan latest_scan_;
    //TODO: current behaviour
    //TODO: list of past behaviours?
    //TODO: list of outstanding/upcoming behaviours
    geometry_msgs::PoseStamped curr_pose_;
    geometry_msgs::PoseStamped goal_pose_;


    //void initParams();
    //void processScan(LDP& curr_ldp_scan, const ros::Time& time);

    // scanCallback: stores lastest scan msg in latest_scan_
    void teleopCallback (const geometry_msgs::Twist::ConstPtr& twist_msg);

    // scanCallback: stores lastest scan msg in latest_scan_
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // poseCallback: stores current pose in curr_pose_
    void poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    float euclidian_distance(geometry_msgs::PoseStamped goal_pose_, geometry_msgs::PoseStamped curr_pose);
    float linear_vel(geometry_msgs::PoseStamped goal_pose_, float constant);
    float steering_angle(geometry_msgs::PoseStamped goal_pose_);
    float angular_vel(geometry_msgs::PoseStamped goal_pose_, float constant);
    void moveToGoal();

    /* safeToMove(): uses current pose and laserScan data to determine if there 
     * is an obstacle in front of the rover */
    bool safeToMove();
};

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
