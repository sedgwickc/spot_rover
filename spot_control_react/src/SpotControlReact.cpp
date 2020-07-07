/* SpotControlReact.cpp
 * Author: Charles Sedgwick
 * Licence: GPL
 */

#include <spot_control_react/SpotControlReact.h>

SpotControlReact::SpotControlReact(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting SpotControlReact");

  // state variables

  // setup publishers
  cmd_vel_publisher_  = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  
  // setup subscribers
  teleop_subscriber_ = nh_.subscribe("teleop/cmd_vel", 1, &SpotControlReact::teleopCallback, this);
  
  scan_subscriber_ = nh_.subscribe("scan", 1, &SpotControlReact::scanCallback, this);
  
  pose_subscriber_ = nh_.subscribe("pose_stamped", 1, &SpotControlReact::poseCallback, this);
    
  //TODO: subscribe to behaviour topic
  
}

SpotControlReact::~SpotControlReact()
{
  ROS_INFO("Destroying SpotControlReact");
}

void SpotControlReact::poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg){

    // store pose message in curr_pose_
    this->curr_pose_.pose = pose_msg->pose;
    this->curr_pose_.header = pose_msg->header;
}

void SpotControlReact::teleopCallback (const geometry_msgs::Twist::ConstPtr& twist_msg){

    //if safe than publish twist_msg to cmd_vel
    if(safeToMove()){
        cmd_vel_publisher_.publish(twist_msg);
    }
}

void SpotControlReact::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg){

    // store latest scan message into latest_scan_
    this->latest_scan_.header = scan_msg->header;
    this->latest_scan_.angle_increment = scan_msg->angle_increment;
    this->latest_scan_.range_min = scan_msg->range_min;
    this->latest_scan_.range_max = scan_msg->range_max;
    this->latest_scan_.ranges = scan_msg->ranges;
}

//void SpotControlReact::processScan(sensor_msgs::LaserScan::ConstPtr, const ros::Time& time){

 // ros::WallTime start = ros::WallTime::now();

 // // CSM is used in the following way:
 // // The scans are always in the laser frame
 // // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
 // // The new scan (currLDPScan) has a pose equal to the movement
 // // of the laser in the laser frame since the last scan
 // // The computed correction is then propagated using the tf machinery

 // prev_ldp_scan_->odometry[0] = 0.0;
 // prev_ldp_scan_->odometry[1] = 0.0;
 // prev_ldp_scan_->odometry[2] = 0.0;

 // prev_ldp_scan_->estimate[0] = 0.0;
 // prev_ldp_scan_->estimate[1] = 0.0;
 // prev_ldp_scan_->estimate[2] = 0.0;

 // prev_ldp_scan_->true_pose[0] = 0.0;
 // prev_ldp_scan_->true_pose[1] = 0.0;
 // prev_ldp_scan_->true_pose[2] = 0.0;

 // input_.laser_ref  = prev_ldp_scan_;
 // input_.laser_sens = curr_ldp_scan;

 // // **** estimated change since last scan

 // double dt = (time - last_icp_time_).toSec();
 // double pr_ch_x, pr_ch_y, pr_ch_a;
 // getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

 // // the predicted change of the laser's position, in the fixed frame

 // tf::Transform pr_ch;
 // createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

 // // account for the change since the last kf, in the fixed frame

 // pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

 // // the predicted change of the laser's position, in the laser frame

 // tf::Transform pr_ch_l;
 // pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

 // input_.first_guess[0] = pr_ch_l.getOrigin().getX();
 // input_.first_guess[1] = pr_ch_l.getOrigin().getY();
 // input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

 // // If they are non-Null, free covariance gsl matrices to avoid leaking memory
 // if (output_.cov_x_m)
 // {
 //   gsl_matrix_free(output_.cov_x_m);
 //   output_.cov_x_m = 0;
 // }
 // if (output_.dx_dy1_m)
 // {
 //   gsl_matrix_free(output_.dx_dy1_m);
 //   output_.dx_dy1_m = 0;
 // }
 // if (output_.dx_dy2_m)
 // {
 //   gsl_matrix_free(output_.dx_dy2_m);
 //   output_.dx_dy2_m = 0;
 // }

 // // *** scan match - using point to line icp from CSM

 // sm_icp(&input_, &output_);
 // tf::Transform corr_ch;

 // // **** statistics

 // double dur = (ros::WallTime::now() - start).toSec() * 1e3;
 // ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
//}

/* safeToMove()
 * This method used the latest scan stored in latest_scan to determine if the area in front of the rover is blocked within
 * a certain range. The method returns true if area not blocked, false otherwise 
 */ 
bool SpotControlReact::safeToMove(){

    // the front of the rover (green axis in rviz config) is aligned with the y-axis of the rover
    return true;

}

float SpotControlReact::euclidian_distance(geometry_msgs::PoseStamped goal_pose, geometry_msgs::PoseStamped curr_pose){
    return 0.0;

}


float SpotControlReact::linear_vel(geometry_msgs::PoseStamped goal_pose_, float constant){
    return 0.0;

}

float SpotControlReact::steering_angle(geometry_msgs::PoseStamped goal_pose_){
    return 0.0;

}

float SpotControlReact::angular_vel(geometry_msgs::PoseStamped goal_pose_, float constant){
    return 0.0;

}

void SpotControlReact::moveToGoal(){

}
