#include "dds_node.hpp" 



/*
    本文件是dds接收和传输节点主程序
    MavData*文件是框架生成的文件
*/




// 先把从mavros拿到的东西，存到向量中，并且从DDS发出去
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dds_node");
    ros::NodeHandle nh("~");



    int mav_num = 1;
    int mav_id = 1;
    nh.param<int>("mav_num",mav_num , 6);
    nh.param<int>("mav_id",mav_id , 1);

    std::cout<<"mav_num:"<<mav_num<<std::endl;
    std::cout<<"mav_id:"<<mav_id<<std::endl;
    
    // index start 0 in C++
    mav_id = mav_id-1; 
    mavDataVec.reserve(mav_num);
    bboxesDataVec.reserve(mav_num);


    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose_cor", 10,
        boost::bind(&poseCallback, _1, mav_id)
    );
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped> (
        "/mavros/local_position/velocity_local", 10,
        boost::bind(&velCallback, _1, mav_id)
    );
    ros::Subscriber bboxes_sub = nh.subscribe<swarm_msgs::BoundingBoxes> (
        "/tracker/pos_image", 10,
        boost::bind(&bboxesCallback, _1, mav_id)
    );


    




    // 接收从DDS来的数据并推到ros的话题中
    for (size_t i = 0; i < mav_num; i++)
    {
        // drone id 从1开始
        std::stringstream fmt;
        fmt << "/drone_"<<(i+1);
        
        mav_data_struct temp_mavdata;
        swarm_msgs::BoundingBoxes temp_bboxMsg;
        mavDataVec.push_back(temp_mavdata);
        bboxesDataVec.push_back(temp_bboxMsg);

        posPubs.push_back(nh.advertise<geometry_msgs::PoseStamped>(fmt.str()+"/mavros/local_position/pose_cor", 10));
        velPubs.push_back(nh.advertise<geometry_msgs::TwistStamped>(fmt.str()+"/mavros/local_position/velocity_local", 10));
        bboxesPubs.push_back(nh.advertise<swarm_msgs::BoundingBoxes>(fmt.str()+"/tracker/pos_image", 10));
    }
    
    std::cout << "Starting DDS pub and sub" << std::endl;

    MavDataNode* mav_data_node = new MavDataNode();
    if(!mav_data_node->init())
    {
        std::cout<< "DDS init fail!" <<std::endl;
        return 1;
    }

    BBoxesDataNode* bboxes_data_node = new BBoxesDataNode();
    if(!bboxes_data_node->init())
    {
        std::cout<< "DDS init fail!" <<std::endl;
        return 1;
    }

    std::cout<< "DDS init successful!" <<std::endl;

    std::thread ddsPub_thread(MavDataUpdate, mav_data_node, mav_id);
    std::thread dds_bbox_thread(BBoxesDataUpdate, bboxes_data_node, mav_id);


    ros::spin();

    return 0;
}
