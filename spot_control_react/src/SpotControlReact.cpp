/* SpotControlReact.cpp
 * Author: Charles Sedgwick
 * Licence: GPL
 */

#include <cmath>
#include <spot_control_react/SpotControlReact.h>

SpotControlReact::SpotControlReact(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  curr_pose_(),
  goal_pose_(),
  curr_behaviour_()
{
  ROS_INFO("Starting SpotControlReact");

  // setup publishers
  this->cmd_vel_publisher_  = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  // setup subscribers
  this->teleop_subscriber_ = nh_.subscribe("teleop/cmd_vel", 10, &SpotControlReact::teleopCallback, this);
  
  this->scan_subscriber_ = nh_.subscribe("scan", 1, &SpotControlReact::scanCallback, this);
  
  this->pose_subscriber_ = nh_.subscribe("pose_stamped", 1, &SpotControlReact::poseCallback, this);
  
  this->behaviour_subscriber_ = nh_.subscribe("behaviour", 1, &SpotControlReact::behaviourCallback, this);
}

SpotControlReact::~SpotControlReact()
{
  ROS_INFO("Destroying SpotControlReact");
}

void SpotControlReact::poseCallback (const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
    
    // only update pose if change in pose seems reasonable (ie difference in position is less than 10 cm)
    if( abs(curr_pose_.pose.position.x - pose_msg->pose.position.x) < .1 && 
        abs(curr_pose_.pose.position.y - pose_msg->pose.position.y) < .1){
        // store pose message in curr_pose_
        this->curr_pose_.pose = pose_msg->pose;
        this->curr_pose_.header = pose_msg->header;
        this->curr_pose_.pose.position.x = round(this->curr_pose_.pose.position.x * 1000.0)/1000.0;
        this->curr_pose_.pose.position.y = round(this->curr_pose_.pose.position.y * 1000.0)/1000.0;
    }
}

void SpotControlReact::teleopCallback (const geometry_msgs::Twist::ConstPtr& twist_msg){

    //if safe than publish twist_msg to cmd_vel
    // issue(?): multiple instances of teleopCallback executing at a time due to speed of laser
    if(twist_msg->linear.x > 0.0){
        if(safeToMoveForward())
            cmd_vel_publisher_.publish(twist_msg);
    } else if(twist_msg->linear.x < 0.0){
        if(safeToMoveBackward())
            cmd_vel_publisher_.publish(twist_msg);
    } else {
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

void SpotControlReact::behaviourCallback (const spot_control_react::Behaviour::ConstPtr& behaviour_msg){
    this->curr_behaviour_.type = behaviour_msg->type;
    
    switch(this->curr_behaviour_.type){
        case MOVE:
            break;
        case MOVETO:
            this->moveToGoal(behaviour_msg->goal_pose);
            break;
        default: 
            ROS_INFO("Behaviour type no supported!");
            break;
        }
}

/* safeToMoveForward()
 * This method used the latest scan stored in latest_scan to determine if the area in front of the rover is blocked
 * within a certain range. The method returns true if area not blocked, false otherwise 
 */ 
bool SpotControlReact::safeToMoveForward(){

    // the front of the rover (green axis in rviz config) is aligned with the y-axis of the rover
    // index into ranges for front of rover is 539
    // store latest scan in a local variable to avoid issues when latest scan updated by callback?

    // if moving forward, check a 90 deg field of view from front therefore the range 539-(91), 539+91 must be 
    //checked to ensure the range values are > 0.15m
    for(int i = 448; i <= 630; i++){
        if(latest_scan_.ranges[i] < 0.16){
            ROS_INFO("Forward: not safe! ");
            return false;
        }
    }

    return true;
}

/* safeToMoveBackward()
 * This method used the latest scan stored in latest_scan to determine if the area behind the rover is blocked
 * within a certain range. The method returns true if area not blocked, false otherwise 
 */ 
bool SpotControlReact::safeToMoveBackward(){

    // store latest scan in a local variable to avoid issues when latest scan updated by callback?

    for(int i = 89; i <= 269; i++){
        if(latest_scan_.ranges[i] < 0.16){
            ROS_INFO("Backward: not safe! ");
            return false;
        }
    }

    return true;
}

float SpotControlReact::euclidian_distance(geometry_msgs::PoseStamped goal_pose){
    return sqrt(pow(goal_pose.pose.position.x - this->curr_pose_.pose.position.x, 2) +
                pow(goal_pose.pose.position.y - this->curr_pose_.pose.position.y, 2));
}

/* linear_vel()
 * Uses the distance of the rover from a goal_pose to calculate the linear velocity used to achieve goal_pose
 * The tanh function is used to normalize the distance to the range [0,1]
 * The constant is used to scale the output of tanh to limit the maximum velocity
 */
float SpotControlReact::linear_vel(geometry_msgs::PoseStamped goal_pose, float constant){
    // normalize distance to a value between 0 and 1 for use as velocity
    // tanh used as normalization function as range of tanh(x) is [0, 1) for x >= 1
    // the input to tanh is scaled up to increase the rate at which the function increases to its max value
    return constant * tanh(20*this->euclidian_distance(goal_pose));
}

float SpotControlReact::steering_angle(geometry_msgs::PoseStamped goal_pose){
   double roll, pitch, yaw; 
   double angle_to_goal;
   tf::Quaternion q(this->curr_pose_.pose.orientation.x, 
      this->curr_pose_.pose.orientation.y, 
      this->curr_pose_.pose.orientation.z, 
      this->curr_pose_.pose.orientation.w); 
   tf::Matrix3x3 m(q);

   // the yaw value is used when determining angular velocity
   m.getRPY(roll, pitch, yaw);
   angle_to_goal = atan2(goal_pose.pose.position.y - this->curr_pose_.pose.position.y,
        goal_pose.pose.position.x - this->curr_pose_.pose.position.x);
   //ROS_INFO("angle to goal: %f, yaw: %f", angle_to_goal, yaw);
   return angle_to_goal - yaw;

}

/* angular_vel()
 * Uses the steering angle calculated using the goal_pose is used to determine the angular velocity to use to achieve 
 * the goal_pose.
 * The tanh function is used to normalize the steering angle to the range [0,1]
 * The constant is used to scale the output of tanh to limit the maximum velocity
 */
float SpotControlReact::angular_vel(geometry_msgs::PoseStamped goal_pose, float constant){
    // normalize distance to a value between 0 and 1 for use as angular velocity
    // tanh used as normalization function as range of tanh(x) is [0, 1) for x >= 1
    // the input to tanh is scaled up to increase the rate at which the function increases to its max value
    return constant * tanh(20*this->steering_angle(goal_pose));
}

void SpotControlReact::moveToGoal(geometry_msgs::PoseStamped goal_pose){

   float distance_tolerance = DISTANCE_TOLERANCE; 
   ros::Rate rate(10);
   //TODO: parameterize these values for easy tuning
   // constant values represent max value that can be achieved for velocity (since constant is used to a scale a value
   // in range [0,1]
   float angular_constant = 0.1;
   float linear_constant = 0.13;
   float lin_vel = 0.0;
   float ang_vel = 0.0;
   float euclidian_dist = 0.0;
   float steering_ang = 0.0;
   bool at_goal = false;
    

   geometry_msgs::Twist vel_msg;

   ROS_INFO("Moving toward goal");
   euclidian_dist = this->euclidian_distance(goal_pose);
   while (at_goal == false){
        // angle to goal
        steering_ang = steering_angle(goal_pose);
        // angle tolerance: 3 deg/ 0.05 rad, 10 deg/0.17rad
        if ( abs(steering_ang) >= 0.10){
            lin_vel = 0.0;
            ang_vel = 0.11;
            //ang_vel = this->angular_vel(goal_pose, angular_constant);
            if(steering_ang < 0)
                ang_vel = -ang_vel;
            vel_msg.linear.x = lin_vel;
            vel_msg.angular.x = 0.0;
            vel_msg.angular.y = 0.0;
            vel_msg.angular.z = ang_vel;
        } else if( abs(euclidian_dist) > distance_tolerance){
            // linear velocity in x-axis
            ang_vel = 0.0;
            vel_msg.angular.z = ang_vel;
            lin_vel = this->linear_vel(goal_pose, linear_constant);
            vel_msg.linear.x = -lin_vel;
            vel_msg.linear.y = 0.0;
            vel_msg.linear.z = 0.0;
        }

        // TODO: add object avoidance logic
        this->cmd_vel_publisher_.publish(vel_msg);

        rate.sleep();
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        this->cmd_vel_publisher_.publish(vel_msg);
        ros::spinOnce(); // process callbacks once for latest pose data
        euclidian_dist = this->euclidian_distance(goal_pose);
        if(abs(euclidian_dist) <= distance_tolerance){
            at_goal = true;
        }
   }

   ROS_INFO("Goal reached. Stopping");
   // bring rover to stop 
   vel_msg.linear.x = 0.0;
   vel_msg.angular.z = 0.0;
   this->cmd_vel_publisher_.publish(vel_msg);
}

