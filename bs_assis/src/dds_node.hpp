#pragma once
#include "MavDataNode.h" 
#include "BBoxesDataNode.h" 


// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/State.h>

#include "swarm_msgs/BoundingBox.h"
#include "swarm_msgs/BoundingBoxes.h"


#include <vector>
#include <thread>
#include <sstream>




// 接收从dds来的数据
std::vector<mav_data_struct> mavDataVec;
std::vector<swarm_msgs::BoundingBoxes> bboxesDataVec;




ros::V_Publisher posPubs;
ros::V_Publisher velPubs;
ros::V_Publisher bboxesPubs;


// ros callback本地的状态
void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, int mav_id) { mavDataVec[mav_id].t = *msg; }
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int mav_id) { mavDataVec[mav_id].p = *msg; }

void bboxesCallback(const swarm_msgs::BoundingBoxes::ConstPtr& msg, int mav_id) { bboxesDataVec[mav_id] = *msg; }

void constructMavData(MavData& data,int mav_id)
{
    mav_data_struct tmp = mavDataVec[mav_id];
    data.system_ID(mav_id);

    data.pose_x(tmp.p.pose.position.x);
    data.pose_y(tmp.p.pose.position.y);
    data.pose_z(tmp.p.pose.position.z);

    data.quat_w(tmp.p.pose.orientation.w);
    data.quat_x(tmp.p.pose.orientation.x);
    data.quat_y(tmp.p.pose.orientation.y);
    data.quat_z(tmp.p.pose.orientation.z);


    data.vel_x(tmp.t.twist.linear.x);
    data.vel_y(tmp.t.twist.linear.y);
    data.vel_z(tmp.t.twist.linear.z);

    data.ang_x(tmp.t.twist.angular.x);
    data.ang_y(tmp.t.twist.angular.y);
    data.ang_z(tmp.t.twist.angular.z);

}

BBoxData constructBBox(const swarm_msgs::BoundingBox msg)
{
    BBoxData data;

    data.id(msg.id);
    data.obj_class(msg.Class);

    data.xmin(msg.xmin);
    data.ymin(msg.ymin);
    data.xmax(msg.xmax);
    data.ymax(msg.ymax);
    data.probability(msg.probability);

    // geometry_msgs/Point
    data.pose_x(msg.point.x);
    data.pose_y(msg.point.y);
    data.pose_z(msg.point.z);

    return data;
}

void constructBBoxes(BBoxesData& data,int mav_id)
{
    swarm_msgs::BoundingBoxes tmp = bboxesDataVec[mav_id];
    data.system_ID(mav_id);
    std::cout<< "constructBBoxes() mav_id ="<< mav_id <<std::endl;
    std::cout<< "constructBBoxes() tmp.bounding_boxes.size() ="<< tmp.bounding_boxes.size() <<std::endl;

    std::vector<BBoxData> bboxVec;
    for (int i=0; i < tmp.bounding_boxes.size(); i++)
    {
        bboxVec.push_back(constructBBox(tmp.bounding_boxes[i]));
    }

    data.bounding_boxes(bboxVec);

}


// dds推送当前的状态
void MavDataUpdate(MavDataNode* dds_node,int mav_id)
{
    while (true)
    {

        MavData data;
        constructMavData(data,mav_id);
        if (dds_node->publish(&data))
        {
            static auto last_ptime = ros::Time::now().toSec();
            if(ros::Time::now().toSec() - last_ptime > 1)
            {
                std::cout << " send system_ID: " << data.system_ID()+1 << " pose z: " << mavDataVec[mav_id].p.pose.position.z << std::endl;
                last_ptime = ros::Time::now().toSec();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/40));
    }
}



void BBoxesDataUpdate(BBoxesDataNode* dds_node,int mav_id)
{
    while (true)
    {

        BBoxesData data;
        constructBBoxes(data,mav_id);
        if (dds_node->publish(&data))
        {

            std::cout << " send system_ID: " << data.system_ID()+1 << " bboxes len: " << bboxesDataVec[mav_id].bounding_boxes.size() << std::endl;

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/10));
    }
}