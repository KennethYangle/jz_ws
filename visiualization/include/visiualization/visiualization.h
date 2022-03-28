#ifndef VISIUALIZATION_H_
#define VISIUALIZATION_H_
#include "ros/ros.h"
#include <iostream>
#include "visualization_msgs/Marker.h"
#include "swarm_msgs/Pipeline.h"

namespace Tube_planning
{
    class Visiualization
    {
    private:
        
        
        


        ros::Subscriber path_sub, generator_sub, cuv_sub, pipe_sub;
        ros::Subscriber right_curve_sub, left_curve_sub;
        ros::Subscriber drone1_sub, drone2_sub, drone3_sub;

        ros::Publisher path_pub, generator_pub, cuv_pub, right_curve_pub, left_curve_pub;
        ros::Publisher drone1_pub, drone2_pub, drone3_pub;
        void pathCallback(const nav_msgs::Path::ConstPtr &msg);
        void genCallback(const nav_msgs::Path::ConstPtr &msg);
        void cuvCallback(const nav_msgs::Path::ConstPtr &msg);
        // void pipeCallback(const swarm_msgs::Pipeline &msg);
        void rightCallback(const nav_msgs::Path::ConstPtr &msg);
        void leftCallback(const nav_msgs::Path::ConstPtr &msg);
        void drone1Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void drone2Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void drone3Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

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