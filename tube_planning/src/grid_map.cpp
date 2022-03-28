#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/Map/OccupancyGrid", 1);
    nav_msgs::OccupancyGrid map;

    map.header.frame_id = "map";
    map.header.stamp = ros::Time::now();

    float resolution;
    int width, height;
    // nh.param("map/info/resolution", resolution, 0.1);
    // nh.param("map/info/width", width, 600);
    // nh.param("map/info/height", height, 600);
    map.info.resolution = 0.1; // float32
    map.info.width = 1250;       // uint32
    map.info.height = 991;      // uint32
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 1;

    // map.info.resolution = 0.1; // float32
    // map.info.width = 37;       // uint32
    // map.info.height = 30;      // uint32
    // map.info.origin.position.x = 0;
    // map.info.origin.position.y = 0;
    // map.info.origin.position.z = 1;

    
    std::ifstream inFile;
    inFile.open("/home/nvidia/buaa_ws/src/tube_planning/doc/gridmap.txt");
    if (!inFile.is_open())
    {
        std::cout << "read file failed." << std::endl;
    }
    else
    {
        std::cout << "read file sucess." << std::endl;
    }
    
    
    int p[map.info.width * map.info.height] = {-1}; // [0,100]
    for (int i = 0; i < map.info.width * map.info.height; i++)
    {
        int wt;
        inFile >> wt;
        // std::cout << wt << std::endl;
        p[i] = wt * 100;
        // if (wt == 100)
        // {
        //     std::cout << wt << std::endl;
        // }
        
    }
    inFile.close();
    std::vector<signed char> a(p, p + map.info.width * map.info.height);
    map.data = a;

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        pub.publish(map);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
