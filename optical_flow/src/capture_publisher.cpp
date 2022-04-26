#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char **argv)
{
    cv::VideoCapture cap(0);

    ros::init(argc, argv, "captyre_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

    ros::Rate loop_rate(20);
    while (nh.ok())
    {
        cv::Mat image;
        cap >> image;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
