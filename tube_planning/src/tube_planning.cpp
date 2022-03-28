#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include "tube_planning/tube_planning.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tube_plan_node");
    ros::NodeHandle nh("~");

    Tube_planning::GeneratorFist generator_first;

    generator_first.init(nh);

    ros::spin();
    
    return 0;
}
