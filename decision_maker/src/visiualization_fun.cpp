#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/Path.h"
#include "visiualization.h"
#include "visualization_msgs/Marker.h"
#include "swarm_msgs/Pipeline.h"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Eigen>

namespace Tube_planning
{
    void Visiualization::init(ros::NodeHandle &nh)
    {

        /* sub nankai problem map */
        nk_map_sub = nh.subscribe("/map2d1", 10, &Visiualization::nankaiCallback, this);
        // pipe_sub = nh.subscribe("/tube/right_dis", 10, &Visiualization::pipeCallback, this);
        nk_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/visualization/map", 10);


    }

    void Visiualization::nankaiCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        gridmap.resolution = msg->info.resolution;
        gridmap.width = msg->info.width;
        gridmap.height = msg->info.height;
        gridmap.data = Eigen::MatrixXi::Zero(gridmap.height, gridmap.width);

        for (int i = 0; i < gridmap.height; i++)
        {
            for (int j = 0; j < gridmap.width; j++)
            {
                gridmap.data(i, gridmap.width - 1 - j) = msg->data[i * gridmap.width + j];
            }
        }

        nav_msgs::OccupancyGrid map_rect;
        int p[gridmap.width * gridmap.height];
        map_rect.info.width = gridmap.width;
        map_rect.info.height = gridmap.height;
        // map_rect.data[gridmap.width * gridmap.height]= {0}; 
        for (int i = 0; i < gridmap.height; i++)
        {
            for (int j = 0; j < gridmap.width; j++)
            {
                p[i * gridmap.width + j] = gridmap.data(i, j);
                // map_rect.data[i * gridmap.width + j] = gridmap.data(i, j);

            }
        }
        std::vector<signed char> a(p, p + gridmap.width * gridmap.height);
        map_rect.data = a;
        map_rect.info.resolution = msg->info.resolution;
        map_rect.info.origin.position.x = -4.2;
        map_rect.info.origin.position.y = -1.6;
        map_rect.info.origin.position.z = 1.8;
        map_rect.header.frame_id = "map";
        map_rect.header.stamp = ros::Time::now();
        nk_map_pub.publish(map_rect);
        // std::cout << map_rect.data[0] << std::endl;

    };

   
}