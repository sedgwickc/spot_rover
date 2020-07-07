/* spot_control_react_node.cpp
 * Charles Sedgwick
 */

#include <spot_control_react/SpotControlReact.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "SpotControlReact");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    //call constructor of SpotControlReact which sets up subscribers/publishers
    SpotControlReact spot_control_react(nh, nh_private);
    ros::spin();
    return 0;
}


