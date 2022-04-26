#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <math.h>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

const double PI = 3.1415926;
const int BLOCK_NUM_X = 16;
const int BLOCK_NUM_Y = 16;
const int FRAME_STEP = 1;           // Simulink迭代一次传出四张图
const string OPENCV_WINDOW = "Image window";
const string TTC_WINDOW = "TTC window";
const int ROI[4] = {5,10,5,10};       // x1,x2,y1,y2; 闭区间

static geometry_msgs::Vector3 att_rate;
static cv::Mat color, last_color;
int num = 0;
list< cv::Point2f > keypoints;      // 要删除跟踪失败的点，使用list

class CalcTTC
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber get_attitude_rate_;
    ros::Publisher balance_force_pub_;
    ros::Publisher ttc_mean_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    std_msgs::Float64 last_force_;

public:
    CalcTTC();
    ~CalcTTC();
    void attitudeRateCb(const geometry_msgs::Vector3& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void ttcMean(const double ttc_sum[BLOCK_NUM_X][BLOCK_NUM_Y], const int kp_cnt[BLOCK_NUM_X][BLOCK_NUM_Y]);
    void balanceStrategy(const vector<cv::Point2f>& optical_flow);
};

CalcTTC::CalcTTC() : it_(nh_)
{
    // get_attitude_rate_ = nh_.subscribe("attitude_rate", 1, &CalcTTC::attitudeRateCb, this);
    balance_force_pub_ = nh_.advertise<std_msgs::Float64>("balance_force", 1);
    ttc_mean_pub_ = nh_.advertise<std_msgs::Float64>("ttc_mean", 1);
    // image_sub_ = it_.subscribe("camera/image_raw", 1, &CalcTTC::imageCb, this);
    // image_sub_ = it_.subscribe("airsim/image_raw", 1, &CalcTTC::imageCb, this);
    image_sub_ = it_.subscribe("/camera/left", 1, &CalcTTC::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(TTC_WINDOW);
}

CalcTTC::~CalcTTC()
{
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(TTC_WINDOW);
}

void CalcTTC::attitudeRateCb(const geometry_msgs::Vector3& msg)
{
    att_rate.x = msg.x;
    att_rate.y = msg.y;
    att_rate.z = msg.z;
}

void CalcTTC::ttcMean(const double ttc_sum[BLOCK_NUM_X][BLOCK_NUM_Y], const int kp_cnt[BLOCK_NUM_X][BLOCK_NUM_Y])  // kp_cnt：统计图像块中关键点个数
{
    int kp_num = 0;
    double ttc_global_sum = 0;
    std_msgs::Float64 ttc_mean;
    for(int i = ROI[0]; i < ROI[1]; i++)
    {
        for(int j = ROI[2]; j < ROI[3]; j++)
        {
            ttc_global_sum += ttc_sum[i][j];
            kp_num += kp_cnt[i][j];
        }
    }
    ttc_mean.data = ttc_global_sum / kp_num;
    ttc_mean_pub_.publish(ttc_mean);
}

void CalcTTC::balanceStrategy(const vector<cv::Point2f>& optical_flow)
{
    int img_width = color.cols;
    int img_height = color.rows;
    int half_width = img_width / 2;
    double left_sum, right_sum, left_mean, right_mean;
    left_sum = right_sum = left_mean = right_mean = 0;
    int left_cnt, right_cnt;
    left_cnt = right_cnt = 0;

    auto iter_key = keypoints.begin();
    for (int i=0; i<optical_flow.size(); i++)
    {
        if (iter_key->x < half_width)
        {
            left_sum += optical_flow[i].x;
            left_cnt++;
        }
        else
        {
            right_sum += optical_flow[i].x;
            right_cnt++;
        }
        
        iter_key++;
    }

    left_mean = left_sum / left_cnt;
    right_mean = right_sum / right_cnt;
    std_msgs::Float64 force;
    force.data = left_mean - right_mean;
    if (isnan(force.data))
    {
        force = last_force_;
    }
    else
    {
        last_force_ = force;
    }
    
    balance_force_pub_.publish(force);
}

void CalcTTC::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if (num % FRAME_STEP != 0) //每隔两个周期执行一次
    {
        num++;
        return;
    }
    try
    {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);  //ROS图像和OpenCV图像的转换
        color = cv_ptr->image.clone();
        cv::imshow(OPENCV_WINDOW, color);
        cv::waitKey(1);
        cout << num << endl;
        cout << msg->header;

        int img_width = color.cols;
        int img_height = color.rows;
        if(num % (15*FRAME_STEP) == 0) //每隔16个周期执行一次
        {
            keypoints.clear();
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps );
            for ( auto kp:kps )
                keypoints.push_back( kp.pt );
            last_color = color.clone();
            num++;
            return;
        }
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints;
        vector<cv::Point2f> prev_keypoints;
        for ( auto kp:keypoints )
            prev_keypoints.push_back(kp);   // 前一帧的特征点变成这一帧的prev
        vector<unsigned char> status;
        vector<float> error;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;

        // 把跟丢的点删掉
        int i = 0;
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            if ( status[i] == 0 )
            {
                iter = keypoints.erase(iter);
                continue;
            }
            *iter = next_keypoints[i];
            iter++;
        }
        i = 0;
        for ( auto iter=prev_keypoints.begin(); iter!=prev_keypoints.end(); i++)
        {
            if ( status[i] == 0 )
            {
                iter = prev_keypoints.erase(iter);
                continue;
            }
            iter++;
        }

        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            return;
        }

        // 计算光流
        vector<cv::Point2f> optical_flow;
        i = 0;
        cv::Point2f temp;
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            temp.x = iter->x - prev_keypoints[i].x;  //这一时刻关键点的坐标 - 上一时刻关键点的坐标
            temp.y = iter->y - prev_keypoints[i].y;
            optical_flow.push_back(temp);
            iter++;
        }

        // 计算延伸焦点 Focus of Extend
        cv::Mat A(optical_flow.size(), 2, CV_64F);
        cv::Mat b(optical_flow.size(), 1, CV_64F);
        cv::Mat foe(2, 1, CV_64F);
        cv::Mat optical_translation(optical_flow.size(), 2, CV_64F);
        auto iter_key = keypoints.begin();
        for ( int i=0; i<optical_flow.size(); i++)  // 计算光流平移分量
        {
            auto pxvec = optical_translation.ptr<double>(i);
            pxvec[0] = optical_flow[i].x - (iter_key->x * iter_key->y * att_rate.x  
                    -  (1 + iter_key->x * iter_key->x) * att_rate.y  +  iter_key->y * att_rate.z);           // 减去旋转分量
            pxvec[1] = optical_flow[i].y - ((1 + iter_key->y * iter_key->y) * att_rate.x  
                    -  iter_key->x * iter_key->y * att_rate.y  -  iter_key->x * att_rate.z);
            iter_key++;
            
        }
        iter_key = keypoints.begin();
        for ( int i=0; i<A.rows; i++)               // 计算A，b
        {
            auto px_A = A.ptr<double>(i);
            auto px_b = b.ptr<double>(i);
            auto px_trans = optical_translation.ptr<double>(i);
            //px_trans为n个像素点光流的分量，iter_key为该点图像平面的x，y坐标
            px_A[0] = px_trans[1];
            px_A[1] = -px_trans[0];
            px_b[0] = iter_key->x * px_trans[1] - iter_key->y * px_trans[0];
            iter_key++;
        }
        cv::solve(A, b, foe, cv::DECOMP_QR);                // 求解FOE，即延伸焦点
        cout << "FOE: \n" << foe << endl << endl;
        if (isnan(foe.at<double>(0,0))) return;

        // 计算碰撞时间 Time to Collision
        double ttc[keypoints.size()] = {0};                 // 计算每个keypoint的ttc
        i = 0;
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); iter++)
        {
            auto pxvec = optical_translation.ptr<double>(i);
            ttc[i] = sqrt( ( (iter->x-foe.at<double>(0,0))*(iter->x-foe.at<double>(0,0)) + (iter->y-foe.at<double>(1,0))*(iter->y-foe.at<double>(1,0)) ) / (pxvec[0]*pxvec[0] + pxvec[1]*pxvec[1]));
            i++;
        }
        int kp_cnt[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};         // 统计图像块中关键点个数
        double ttc_sum[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
        i = 0;
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); iter++)
        {
            int index_x = (int)(iter->x * BLOCK_NUM_X / img_width);
            int index_y = (int)(iter->y * BLOCK_NUM_Y /img_height);
            kp_cnt[index_x][index_y]++;
            ttc_sum[index_x][index_y] += ttc[i];
            i++;
        }
        double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};   // 计算块中平均碰撞时间
        for( int i=0; i<BLOCK_NUM_X; i++)
        {
            for (int j=0; j<BLOCK_NUM_Y; j++)
            {
                if (kp_cnt[i][j])
                {
                    ttc_block[i][j] = ttc_sum[i][j] / kp_cnt[i][j];
                }
            }
        }
        // 画出 keypoints和光流
        cv::Mat img_show = color.clone();
        cv::Scalar line_color;
        // cv::circle(img_show, cv::Point(int(foe.at<double>(0,0)), int(foe.at<double>(1,0))), 5, cv::Scalar(0, 0, 240), 3);     // 画FOE
        i = 0;
        for ( auto iter=optical_flow.begin(); iter!=optical_flow.end(); iter++)
        {
            int theta_to_scale = int( atan( abs(iter->y/iter->x) ) /PI*2*255 );
            if ( iter->x > 0 && iter->y >0)
                line_color = cv::Scalar(10, theta_to_scale, 10);
            else if ( iter->x > 0 && iter->y < 0)
                line_color = cv::Scalar(60, 60, theta_to_scale);
            else if ( iter->x < 0 && iter->y > 0)
                line_color = cv::Scalar(theta_to_scale, 10, 110);
            else
                line_color = cv::Scalar(theta_to_scale, 160, 160);
            cv::line(img_show, prev_keypoints[i], prev_keypoints[i]+*iter, line_color, 1, 8);
            i++;
        }

        // 画出碰撞时间
        for (int i=0; i<BLOCK_NUM_X; i++)
        {
            for (int j=0; j<BLOCK_NUM_Y; j++)
            {
                if (kp_cnt[i][j] && isfinite(ttc_block[i][j]))
                {
                    char str[25];
                    sprintf(str, "%.1lf", ttc_block[i][j]);
                    string text = str;
                    cv::Point origin = cv::Point(img_width/BLOCK_NUM_X * i, img_height/BLOCK_NUM_Y * j);
                    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.2;
                    cv::Scalar color = cv::Scalar(255,255,255);
                    int thickness = 1;
                    int lineType = 8;
                    cv::putText(img_show, text, origin, fontFace, fontScale, color, thickness, lineType);
                }
            }
        }

        // 发布ttc_mean
        ttcMean(ttc_sum, kp_cnt);

        // 光流平衡
        balanceStrategy(optical_flow);

        last_color = color.clone();
        cv::imshow(TTC_WINDOW, img_show);
        cv::waitKey(1);
        num++;
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "calc_ttc");
    CalcTTC ct;
    ros::spin();
    return 0;
}