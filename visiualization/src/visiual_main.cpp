#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Path.h"
#include "visiualization/visiualization.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visiual_node");
    ros::NodeHandle nh;

    Tube_planning::Visiualization visiual;

    visiual.init(nh);

    ros::spin();

    return 0;
}