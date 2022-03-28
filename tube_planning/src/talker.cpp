#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sstream>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    /* 该代码用于模拟南开发送的路径点消息 */
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    /* 发送路径点 */
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/paths", 1, true);

    ros::Time current_time;
    current_time = ros::Time::now();

    /* 设置模拟发送的路径点 */
    std::ifstream inFile;
    inFile.open("/home/nvidia/buaa_ws/src/tube_planning/doc/route.txt");
    if (!inFile.is_open())
    {
        std::cout << "read route failed." << std::endl;
    }
    else
    {
        std::cout << "read route sucess." << std::endl;
    }
    int len = 10;
    double path_array[len][3];
    for (int i = 0; i < len; i++)
    {
        double wt;
        inFile >> wt;
        path_array[i][0] = wt * 0.1;
        inFile >> wt;
        path_array[i][1] = wt * 0.1;
        path_array[i][2] = 1;
    }
    inFile.close();
    // std::cout << path_array << std::endl;

    ros::Rate loop_rate(1);
    // ROS_INFO("talker begin publishing paths");
    std::cout << "[talker]: begin publishing paths" << std::endl;
    while (ros::ok())
    {
        nav_msgs::Path path;
        path.header.stamp = current_time;
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped this_pose_stamped;
        for (int i = 0; i < len; i++)
        { /* 装填路径点 */
            current_time = ros::Time::now();
            this_pose_stamped.header.stamp = current_time;
            this_pose_stamped.header.frame_id = "map";

            this_pose_stamped.pose.position.x = path_array[i][0];
            this_pose_stamped.pose.position.y = path_array[i][1];
            this_pose_stamped.pose.position.z = path_array[i][2];

            path.poses.push_back(this_pose_stamped);
        }
        path_pub.publish(path);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}