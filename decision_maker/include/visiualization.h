#ifndef VISIUALIZATION_H_
#define VISIUALIZATION_H_
#include "ros/ros.h"
#include <iostream>
#include "visualization_msgs/Marker.h"
#include "swarm_msgs/Pipeline.h"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Eigen>

namespace Tube_planning
{
    class Visiualization
    {
    private:
        struct GridMap
        {
            float resolution;
            int width;
            int height;
            Eigen::MatrixXi data;
        };
        GridMap gridmap;
        
        

        ros::Subscriber nk_map_sub;

        ros::Publisher nk_map_pub;

        
        void nankaiCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    public:
        Visiualization()
        {
        }
        ~Visiualization()
        {
        }
        void init(ros::NodeHandle& nh);
    };
}

#endif