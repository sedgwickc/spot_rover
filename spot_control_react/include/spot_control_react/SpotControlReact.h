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
#include <spot_control_react/Behaviour.h>
#include <tf/transform_datatypes.h>

// the distance from a goal_pose that the rover must be before it stops trying to reach goal_pose
#define DISTANCE_TOLERANCE  0.02 // in metres

// BEHAVE_TYPE: enum that stores indexes for supported behaviour types
enum BEHAVE_TYPE{ 
    MOVE,
    MOVETO,
    WANDER,
    EXPLORE,
    WAIT,
    SCAN
    };

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
    //current behaviour
    spot_control_react::Behaviour curr_behaviour_;
    //TODO: list of past behaviours?
    //TODO: list of outstanding/upcoming behaviours
    geometry_msgs::PoseStamped curr_pose_;
    geometry_msgs::PoseStamped goal_pose_;

    // scanCallback: stores lastest scan msg in latest_scan_
    void teleopCallback (const geometry_msgs::Twist::ConstPtr& twist_msg);

    // scanCallback: stores lastest scan msg in latest_scan_
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // poseCallback: stores current pose in curr_pose_
    void poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    
    // behaviourCallback: stores current behaviour in curr_behaviour_
    void behaviourCallback (const spot_control_react::Behaviour::ConstPtr& behaviour_msg);

    float euclidian_distance(geometry_msgs::PoseStamped goal_pose);
    float linear_vel(geometry_msgs::PoseStamped goal_pose_, float constant);
    float steering_angle(geometry_msgs::PoseStamped goal_pose_);
    float angular_vel(geometry_msgs::PoseStamped goal_pose_, float constant);
    void moveToGoal(geometry_msgs::PoseStamped goal_pose);

    /* safeToMoveForeward(): uses latest laserScan data to determine if there 
     * is an obstacle in front of the rover */
    bool safeToMoveForward();

    /* safeToMoveBackward(): uses latest laserScan data to determine if there 
     * is an obstacle behind the rover */
    bool safeToMoveBackward();

};

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
