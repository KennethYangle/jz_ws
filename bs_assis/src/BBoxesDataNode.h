#pragma once
/*
    本文件是BBoxes的头文件定义
*/

// 生成的消息类型头文件
#include "BBoxDataPubSubTypes.h"
#include "BBoxesDataPubSubTypes.h"

#include "DDS_NodeBase.h"

// ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "swarm_msgs/BoundingBox.h"
#include "swarm_msgs/BoundingBoxes.h"

#include <string>


extern ros::V_Publisher bboxesPubs;



class BBoxesDataSubscriber : public DDS_Subscriber
{
    // 接收，数据可用的时候
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&recvData, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                swarm_msgs::BoundingBoxes bboxesMsg;
                

                bboxesMsg.header.stamp = ros::Time::now();
                for (size_t i = 0; i < recvData.bounding_boxes().size(); i++)
                {
                    swarm_msgs::BoundingBox bboxMsg;
                    auto bbox_data = recvData.bounding_boxes()[i];

                    bboxMsg.id = bbox_data.id();
                    bboxMsg.Class = bbox_data.obj_class();

                    bboxMsg.xmin = bbox_data.xmin();
                    bboxMsg.ymin = bbox_data.ymin();
                    bboxMsg.xmax = bbox_data.xmax();
                    bboxMsg.ymax = bbox_data.ymax();
                    bboxMsg.probability = bbox_data.probability();

                    bboxMsg.point.x = bbox_data.pose_x();
                    bboxMsg.point.y = bbox_data.pose_y();
                    bboxMsg.point.z = bbox_data.pose_z();

                    bboxesMsg.bounding_boxes.push_back(bboxMsg);
                }
                bboxesPubs[recvData.system_ID()].publish(bboxesMsg);
            }
        }
    }

    BBoxesData  recvData;
};




class BBoxesDataNode
{
public:

    BBoxesDataNode();
    ~BBoxesDataNode();

    bool init();
    bool publish(BBoxesData* sendData);

private:
    DomainParticipant* participant_;

    Topic* topic_;
    TypeSupport type_;

    Subscriber* subscriber_;
    DataReader* reader_;

    Publisher* publisher_;
    DataWriter* writer_;
    
    DDS_Publisher dwListener_;
    BBoxesDataSubscriber drListener_;

};
