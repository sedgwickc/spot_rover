/***************************************************************************
 * MotorControl.hpp
 * Charles Segdwick
 * Defines the class members got MotorControl which is used to control a rover
 * with up to 4 motors (two on each side)
 ***************************************************************************/

#include <stdint.h>
extern "C"
{  
#include "roboticscape.h"
}
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib_msgs/GoalStatus.h>

namespace rover {

/* Defines */
#define ROBOT_WIDTH .15 // robot width in meters

class MotorControl
{
public:
	MotorControl();

	int turn_left();
	int turn_right();
	int stop();
	int forward();
	int backward();
	void set_speed(geometry_msgs::Vector3 pLinear);
	geometry_msgs::Vector3 get_speed();
	void set_angular(geometry_msgs::Vector3 pAngular);
	geometry_msgs::Vector3 get_angular();

	int clean_up();
    void callback(const geometry_msgs::Twist::ConstPtr& msg);
    void status_callback(const actionlib_msgs::GoalStatus::ConstPtr& msg);
    virtual ~MotorControl();
private:

    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
};

} // rover namespace
