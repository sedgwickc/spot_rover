/*  
 * SpotControlDelib.h
 * Author: Charles Sedgwick
 * Licence: GPL
 */

#ifndef SPOT_CONTROL_DELIB_H
#define SPOT_CONTROL_DELIB_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <spot_control_react/Behaviour.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/PixyData.h>

class SpotControlDelib
{
  public:

    SpotControlDelib(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SpotControlDelib();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber pose_subscriber_;
    ros::Subscriber blocks_subscriber_;

    ros::Publisher behaviour_publisher_;

    //currently detected blocks
    pixy_msgs::PixyData curr_blocks_;


    //current behaviour
    spot_control_react::Behaviour curr_behaviour_;
    //TODO: list of past behaviours?
    //TODO: list of outstanding/upcoming behaviours
    geometry_msgs::PoseStamped curr_pose_;
    geometry_msgs::PoseStamped goal_pose_;


    // poseCallback: stores current pose in curr_pose_
    void poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    
    // blockCallback: stores current blocks detected by pixycam in 
    void blocksCallback (const pixy_msgs::PixyData::ConstPtr& blocks_msg);
};

#endif //SPOT_CONTROL_DELIB_H
