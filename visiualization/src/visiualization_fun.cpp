#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Path.h"
#include "visiualization/visiualization.h"
#include "visualization_msgs/Marker.h"
#include "swarm_msgs/Pipeline.h"

namespace Tube_planning
{
    void Visiualization::init(ros::NodeHandle &nh)
    {
        /* 路径点 */
        path_sub = nh.subscribe("/generator_curve/paths", 10, &Visiualization::pathCallback, this);
        /* 生成线 */
        generator_sub = nh.subscribe("/tube/generator_curve", 10, &Visiualization::genCallback, this);
        /* 最大曲率 */
        // cuv_sub =  nh.subscribe("/tube/max_cuv_r", 10, &Visiualization::cuvCallback, this);
        cuv_sub =  nh.subscribe("/tube/right_dis", 10, &Visiualization::cuvCallback, this);
        /* 右侧管道 */
        right_curve_sub = nh.subscribe("/tube/tube_right", 10, &Visiualization::rightCallback, this);
        left_curve_sub = nh.subscribe("/tube/tube_left", 10, &Visiualization::leftCallback, this);

        /* sub drone point */
        drone1_sub = nh.subscribe("/drone_1/mavros/local_position/pose_cor", 10, &Visiualization::drone1Callback, this);
        drone2_sub = nh.subscribe("/drone_2/mavros/local_position/pose_cor", 10, &Visiualization::drone2Callback, this);
        drone3_sub = nh.subscribe("/drone_3/mavros/local_position/pose_cor", 10, &Visiualization::drone3Callback, this);
  
        // pipe_sub = nh.subscribe("/tube/right_dis", 10, &Visiualization::pipeCallback, this);

        path_pub = nh.advertise<visualization_msgs::Marker>("/visualization/path", 10);
        generator_pub = nh.advertise<visualization_msgs::Marker>("/visualization/generator", 10);
        cuv_pub = nh.advertise<visualization_msgs::Marker>("/visualization/cuv", 10);
        right_curve_pub = nh.advertise<visualization_msgs::Marker>("/visualization/right_curve", 10);
        left_curve_pub = nh.advertise<visualization_msgs::Marker>("/visualization/left_curve", 10);
        drone1_pub = nh.advertise<visualization_msgs::Marker>("/visualization/drone1", 10);
        drone2_pub = nh.advertise<visualization_msgs::Marker>("/visualization/drone2", 10);
        drone3_pub = nh.advertise<visualization_msgs::Marker>("/visualization/drone3", 10);


    }

    void Visiualization::drone3Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "points_and_lines";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 5;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0f;
        // marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.pose = msg->pose;//Pose
        std::cout << "msg->pose :" << msg->pose << std::endl;

        drone3_pub.publish(marker);
    }


    void Visiualization::drone1Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "points_and_lines";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 5;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0f;
        // marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.pose = msg->pose;//Pose
        // std::cout << "msg->pose :" << msg->pose << std::endl;

        drone1_pub.publish(marker);
    }



    void Visiualization::drone2Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "points_and_lines";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 5;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0f;
        // marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.pose = msg->pose;//Pose
        std::cout << "msg->pose :" << msg->pose << std::endl;

        drone2_pub.publish(marker);


    }



    void Visiualization::leftCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        int path_length = end(msg->poses) - begin(msg->poses);

        visualization_msgs::Marker path_point;
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "map";
        path_point.ns = "points_and_lines";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.id = 1;
        path_point.type = visualization_msgs::Marker::LINE_STRIP;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.color.g = 1.0f;
        path_point.color.b = 1.0f;
        path_point.color.a = 1.0;

        for (int i = 0; i < path_length; i++)
        {
            geometry_msgs::Point p;
            p.x = msg->poses[i].pose.position.x;
            p.y = msg->poses[i].pose.position.y;
            p.z = msg->poses[i].pose.position.z;

            path_point.points.push_back(p);

            geometry_msgs::Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = 0;
            q.w = 1;
            path_point.pose.orientation = q;

        }

        left_curve_pub.publish(path_point);
    }
    
    void Visiualization::rightCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        int path_length = end(msg->poses) - begin(msg->poses);

        visualization_msgs::Marker path_point;
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "map";
        path_point.ns = "points_and_lines";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.id = 1;
        path_point.type = visualization_msgs::Marker::LINE_STRIP;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.color.g = 1.0f;
        path_point.color.b = 1.0f;
        path_point.color.a = 1.0;

        for (int i = 0; i < path_length; i++)
        {
            geometry_msgs::Point p;
            p.x = msg->poses[i].pose.position.x;
            p.y = msg->poses[i].pose.position.y;
            p.z = msg->poses[i].pose.position.z;

            path_point.points.push_back(p);

            geometry_msgs::Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = 0;
            q.w = 1;
            path_point.pose.orientation = q;

        }

        right_curve_pub.publish(path_point);
    }
    // void Visiualization::pipeCallback(const swarm_msgs::Pipeline &msg)
    // {
    //     // int path_length = end(msg->units) - begin(msg->units);
    //     // std::cout << path_length << std::endl;
    // }

    void Visiualization::cuvCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        int path_length = end(msg->poses) - begin(msg->poses);
        visualization_msgs::Marker path_point;
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "map";
        path_point.ns = "points_and_lines";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.id = 2;
        path_point.type = visualization_msgs::Marker::LINE_LIST;
        path_point.scale.x = 0.08;
        path_point.scale.y = 0.08;
        path_point.color.r = 1.0f;
        path_point.color.a = 1.0;

        for (int i = 0; i < path_length; i++)
        {
            geometry_msgs::Point p;
            p.x = msg->poses[i].pose.position.x;
            p.y = msg->poses[i].pose.position.y;
            p.z = msg->poses[i].pose.position.z;

            path_point.points.push_back(p);

            geometry_msgs::Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = 0;
            q.w = 1;
            path_point.pose.orientation = q;

        }

        cuv_pub.publish(path_point);


    }
    void Visiualization::genCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        int path_length = end(msg->poses) - begin(msg->poses);

        visualization_msgs::Marker path_point;
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "map";
        path_point.ns = "points_and_lines";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.id = 1;
        path_point.type = visualization_msgs::Marker::LINE_STRIP;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.color.g = 1.0f;
        path_point.color.a = 1.0;

        for (int i = 0; i < path_length; i++)
        {
            geometry_msgs::Point p;
            p.x = msg->poses[i].pose.position.x;
            p.y = msg->poses[i].pose.position.y;
            p.z = msg->poses[i].pose.position.z;

            path_point.points.push_back(p);

            geometry_msgs::Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = 0;
            q.w = 1;
            path_point.pose.orientation = q;

        }

        generator_pub.publish(path_point);
        // std::cout << "visual gen pub sucess" << std::endl;


    }

    void Visiualization::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        int path_length = end(msg->poses) - begin(msg->poses);

        visualization_msgs::Marker path_point;
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "map";
        path_point.ns = "points_and_lines";
        path_point.action = visualization_msgs::Marker::ADD;
        path_point.id = 0;
        path_point.type = visualization_msgs::Marker::POINTS;
        path_point.scale.x = 0.2;
        path_point.scale.y = 0.2;
        path_point.color.r = 1.0f;
        path_point.color.a = 1.0;

        for (int i = 0; i < path_length; i++)
        {
            geometry_msgs::Point p;
            p.x = msg->poses[i].pose.position.x;
            p.y = msg->poses[i].pose.position.y;
            p.z = msg->poses[i].pose.position.z;

            path_point.points.push_back(p);
        }

        path_pub.publish(path_point);
        // std::cout << "visual path pub sucess" << std::endl;

    }
}