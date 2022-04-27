#include "ros/ros.h"
// #include "nav_msgs/Path.h"
#include <iostream>
#include "swarm_msgs/Pipeline.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_sender_zs");
    ros::NodeHandle n;
    // ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/paths", 1, true);
    ros::Publisher pipeline_pub = n.advertise<swarm_msgs::Pipeline>("/pipeline/paths", 1, true);

    ros::Rate loop_rate(1);
    std::cout << "[path_sender]: begin publishing paths" << std::endl;
    while(ros::ok())
    {
        // nav_msgs::Path path;
        // geometry_msgs::PoseStamped this_pose_stamped;

        swarm_msgs::Pipeline tube;
        swarm_msgs::Pipeunit unit;
        for (int i = 0; i < 20; i++)
        { /* 装填路径点 */
            // this_pose_stamped.pose.position.x = 1 * i;
            // this_pose_stamped.pose.position.y = 2 * i;
            // this_pose_stamped.pose.position.z = 3 * i;
            // path.poses.push_back(this_pose_stamped);

            unit.middle.x = 1 * i;
            unit.middle.y = 2 * i;
            unit.middle.z = 1;

            unit.left.x = 3 * i;
            unit.left.y = 4 * i;
            unit.left.z = 1;

            unit.right.x = 5 * i;
            unit.right.y = 6 * i;
            unit.right.z = 1;


            tube.units.push_back(unit);


            
        }
        // path_pub.publish(path);
        pipeline_pub.publish(tube);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}