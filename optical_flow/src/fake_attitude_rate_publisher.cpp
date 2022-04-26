#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    geometry_msgs::Vector3 msg;
    msg.x = msg.y = msg.z = 0;

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("attitude_rate", 1);
    ros::Rate loop_rate(50);
    while (ros::ok())         //循环发布msg
    {
        pub.publish(msg);     //以1Hz的频率发布msg
        loop_rate.sleep();    //根据前面的定义的loop_rate,设置1s的暂停
    }
    return 0;
}