/* SpotControlDelib.cpp
 * Author: Charles Sedgwick
 * Licence: GPL
 */

#include <vector>
#include <iterator>
#include <spot_control_delib/SpotControlDelib.h>

SpotControlDelib::SpotControlDelib(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting SpotControlDelib");

  // setup publishers
  behaviour_publisher_  = nh_.advertise<spot_control_react::Behaviour>("behaviour", 1);
  
  // setup subscribers
  pose_subscriber_ = nh_.subscribe("pose_stamped", 1, &SpotControlDelib::poseCallback, this);
  
  blocks_subscriber_ = nh_.subscribe("/my_pixy/block_data", 1, &SpotControlDelib::blocksCallback, this);
    
  
}

SpotControlDelib::~SpotControlDelib()
{
  ROS_INFO("Destroying SpotControlDelib");
}

void SpotControlDelib::poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg){

    // store pose message in curr_pose_
    this->curr_pose_.pose = pose_msg->pose;
    this->curr_pose_.header = pose_msg->header;
}
    
void SpotControlDelib::blocksCallback (const pixy_msgs::PixyData::ConstPtr& blocks_msg){
    this->curr_blocks_.blocks = blocks_msg->blocks;
    if(boost::size(this->curr_blocks_.blocks) > 0){
        ROS_INFO("Blocks detected!");
    }
}
