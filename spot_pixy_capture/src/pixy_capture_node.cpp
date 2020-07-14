/******************************************************************************
 * talker.cpp
 * This file defines the talker node which prints output from the attached
 * PixyCam to the chatter topic.
 *****************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "pixy.h"
#include <sstream>

#define BLOCK_BUFFER_SIZE 25

//pixy Block buffer 
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    int i = 0;
    int index; 
    int blocks_copied;
    int pixy_init_status;
    char buf[128]; 

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "spot_talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
    // only continue if we can connect to pixycam
    pixy_init_status = pixy_init();
    if(!pixy_init_status == 0){
        // error initializing pixy
        printf("pixy_init(): ");
        pixy_error(pixy_init_status);
        return pixy_init_status;
    }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
        // wait for new blocks to be available
        while (!pixy_blocks_are_new() && run_flag);
        
        // get blocks from pixy
        blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "frame  " << count <<":";
    for(index=0; index != blocks_copied; ++index){
        blocks[index].print(buf);
        ss<<"  "<<buf<<std::endl;
    }

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  pixy_close();


  return 0;
}
