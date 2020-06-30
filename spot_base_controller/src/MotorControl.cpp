/***************************************************************************
 * MotorControl.cpp
 * Charles Sedgwick
 * The MotorControl class is reponsible for controlling the direction and speed
 * of the 4 motors. 
 *
 ***************************************************************************/

// main roboticscape API header
#include "MotorControl/MotorControl.hpp"
extern "C"
{  
#include "roboticscape.h"
}
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib_msgs/GoalStatus.h>

using namespace std;

namespace rover {

/***************************************************************************
 * CONSTRUCTOR
 * @brief  Instantiates a new motor control class
 *************************************************************************/
MotorControl::MotorControl(){
	// initialize motors
	if(rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ)){
		fprintf(stderr,"ERROR: failed to initialize hardware.\n");
        return;
	}
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

void MotorControl::set_speed(geometry_msgs::Vector3 pLinear){
    linear = pLinear;
}

geometry_msgs::Vector3 MotorControl::get_speed(){
	return linear;
}

void MotorControl::set_angular(geometry_msgs::Vector3 pAngular){
    angular = pAngular;
}

geometry_msgs::Vector3 MotorControl::get_angular(){
	return angular;
}

int MotorControl::clean_up(){
    rc_motor_cleanup();
	return 1;
}

MotorControl::~MotorControl(){
    rc_motor_cleanup();
}

void MotorControl::status_callback(const actionlib_msgs::GoalStatus::ConstPtr& msg){
    if( msg->status == msg->ABORTED ){
        rc_motor_cleanup();
    }
}

/**
 * This call back will convert the linear components of the message read from
 * the topic in to movement of the rover. 
 */
void MotorControl::callback(const geometry_msgs::Twist::ConstPtr& msg){
    /* forward: +linear.x; backward: -linear.x; left: angular.z; right:
     * -anguler.z */
    linear = msg->linear;
    angular = msg->angular;

    float left_speed_out = linear.x - (angular.z);
    float right_speed_out = linear.x + (angular.z);

    rc_motor_set(2, right_speed_out);
    rc_motor_set(1, left_speed_out);
}

}; // rover namespace

