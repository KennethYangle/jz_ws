#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <math.h>
#include <string>
#include <random>
#include <algorithm>
#include <cmath>
#include <numeric>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/Imu.h>

#include "swarm_msgs/Pipeline.h"
#include <swarm_msgs/Index.h>
#include <geometry_msgs/Point.h>
cv::VideoWriter vw;
// cv::VideoWriter ttcw;
// cv::VideoWriter ofw;
// cv::VideoWriter opticalw;

const double PI = 3.1415926;
const int BLOCK_NUM_X = 16;//16
const int BLOCK_NUM_Y = 16;//16
const int FRAME_STEP = 1;           // Simulink迭代一次传出四张图
const string OPENCV_WINDOW = "Image window";
const string TTC_WINDOW = "TTC window";
const string ttc_WINDOW = "ttc window";
const string of_WINDOW = "oc window";
const string optical_WINDOW = "Image window";
// const int ROI[4] = {5,10,5,10};       // x1,x2,y1,y2; 闭区间
const int ROI[4] = {5, 10, 5, 10}; // x1,x2,y1,y2; 闭区间
cv::Size winSize(10, 10);
cv::TermCriteria termcrit = cv::TermCriteria(
    cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
    20,
    0.03);

static geometry_msgs::Vector3 att_rate;
static cv::Mat color, last_color, prevgray;
float color_x = 0;
int num = 0;
float foe_x = 0;
float foe_y = 0;
float tc[1];
float foc[1];
float ttc_safe = 10.0;
float flow_safe = 0.1;
int u_ttc_safe[2] = {BLOCK_NUM_X / 2, BLOCK_NUM_Y / 2}; // = (BLOCK_NUM_X / 2, BLOCK_NUM_Y / 2);
vector<vector<bool>> is_TTC_u(BLOCK_NUM_X, vector<bool>(BLOCK_NUM_Y,false));
vector<vector<bool>> is_OF_u(BLOCK_NUM_X, vector<bool>(BLOCK_NUM_Y, false));
int ttc_min_pos[2];
int u_flow_safe[2];
float ttc_ratio = 0.1;
// vector<double> filter_median(6, 0.0);
//vector<vector<vector<double>>> filter_col_row(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)
vector<vector<vector<double>>> ttc_block_median(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)
list<cv::Point2f> keypoints; // 要删除跟踪失败的点，使用list

swarm_msgs::Index ttc_pub;
swarm_msgs::Index of_pub;

class CalcTTC
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber get_attitude_rate_;
    ros::Publisher balance_force_pub_;
    ros::Publisher ttc_mean_pub_;
    ros::Publisher TTC_message_pub_;
    ros::Publisher OF_message_pub_;
    //ros::Publisher ttc_mean_plus_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    // image_transport::Subscriber image_sub_;

    std_msgs::Float64 last_force_;

public:
    CalcTTC();
    ~CalcTTC();
    void attitudeRateCb(const sensor_msgs::Imu& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    // void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
    void ttcMean(const double ttc_sum[BLOCK_NUM_X][BLOCK_NUM_Y], const int kp_cnt[BLOCK_NUM_X][BLOCK_NUM_Y]);
    void safeStrategy(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height);
    // void advStrategy(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height);
    vector<vector<int>> advStrategy(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height);
    // void ttcMean_plus(const double ttc_sum[BLOCK_NUM_X][BLOCK_NUM_Y], const int kp_cnt[BLOCK_NUM_X][BLOCK_NUM_Y]);
    void balanceStrategy(const vector<cv::Point2f>& optical_flow);
    //输入信号，窗口大小，输出结果大小
    //Filter function
    vector<double> median_filter(const vector<double> signal, int window_n, int result_n);
    vector<vector<double>> Gaussian(const int img_width, const int img_height);
    void drawOptFlowMap(const cv::Mat &flow, cv::Mat &cflowmap, int step, double, const cv::Scalar &color);
    double TTC_messgae(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const int img_width, const int img_height, const vector<vector<double>> gaussian_result);
    double TTC_messgae_V2(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const int img_width, const int img_height, const vector<vector<double>> gaussian_result);
    double OF_messgae(const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height, const vector<vector<double>> gaussian_result);
    geometry_msgs::Point pub_trance(int row, int col, double u);
};

CalcTTC::CalcTTC() : it_(nh_)
{
    // get_attitude_rate_ = nh_.subscribe("attitude_rate", 1, &CalcTTC::attitudeRateCb, this);
    get_attitude_rate_ = nh_.subscribe("mavros/imu/data", 1, &CalcTTC::attitudeRateCb, this);
    balance_force_pub_ = nh_.advertise<std_msgs::Float64>("balance_force", 1);
    ttc_mean_pub_ = nh_.advertise<std_msgs::Float64>("ttc_mean", 1);
    TTC_message_pub_ = nh_.advertise<swarm_msgs::Index>("TTC_message", 1);
    OF_message_pub_ = nh_.advertise<swarm_msgs::Index>("OF_message", 1);
    //ttc_mean_plus_ = nh_.advertise<std_msgs::Float64>("ttc_mean", 1);
    // image_sub_ = it_.subscribe("camera/image_raw", 1, &CalcTTC::imageCb, this);
    // image_sub_ = it_.subscribe("airsim/image_raw", 1, &CalcTTC::imageCb, this);
    // image_sub_ = it_.subscribe("/camera/left", 1, &CalcTTC::imageCb, this);
    image_sub_ = nh_.subscribe("/camera/rgb", 1, &CalcTTC::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(TTC_WINDOW);
    // cv::namedWindow(of_WINDOW);
    // cv::namedWindow(ttc_WINDOW);
    // cv::namedWindow(optical_WINDOW);
    // cv::namedWindow("flow");
}

CalcTTC::~CalcTTC()
{
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(TTC_WINDOW);
    // cv::destroyWindow(of_WINDOW);
    // cv::destroyWindow(ttc_WINDOW);
    // cv::destroyWindow(optical_WINDOW);
    // cv::namedWindow("flow");
}

void CalcTTC::attitudeRateCb(const sensor_msgs::Imu& msg)
{
    // att_rate.x = msg.angular_velocity.y;
    // att_rate.y = msg.angular_velocity.z;
    // att_rate.z = msg.angular_velocity.x;
    
    //2021/07/04
    // att_rate.x = msg.angular_velocity.y;
    // att_rate.y = -msg.angular_velocity.z;
    // att_rate.z = msg.angular_velocity.x;
    att_rate.x = 0;
    att_rate.y = 0;
    att_rate.z = 0;
}

vector<double> CalcTTC::median_filter(const vector<double> signal, int window_n, int result_n)
{
    //   Check arguments
    if (signal.empty() || signal.size() < 1)
        return vector<double>(1, 0);
    //   Treat special case N = 1
    if (signal.size() == 1)
        return signal;
    int signal_size = signal.size();
    vector<double> window(window_n, 0.0);
    vector<double> result(result_n, 0.0);
    for (int i = 2; i < signal.size(); ++i)
    {
        for (int j = 0; j < window_n; j++)
        {
            window[j] = signal[i + j - 2];
            // cout << "window[j] = " << window[j] << "  " << "j = " << j << endl;
        }
        // sort(window.begin(), window.end());
        //排序，取最小值
        for (int j = 0; j < (window.size() + 1) / 2; ++j)
        {
            int min = j;
            for (int k = j + 1; k < window.size(); ++k)
            {
                if (window[k] < window[min])
                    min = k;
            }
            double temp = window[j];
            window[j] = window[min];
            window[min] = temp;
        }
        // int num = (int)(window_n / 2);
        // cout << "window.size = " << window.size() << endl;
        int num = window.size() % 2;
        // cout << "window.num = " << num << endl;
        result[i - 2] = window[num];
    }

    // result = signal;
    return result;
}

vector<vector<double>> CalcTTC::Gaussian(const int img_width, const int img_height)
{
    vector<vector<double>> result(BLOCK_NUM_X, vector<double>(BLOCK_NUM_Y, 0.0));
    std::default_random_engine e;              //引擎
    int col = (int)(foe_y * BLOCK_NUM_Y / img_height);
    int row = (int)(foe_x * BLOCK_NUM_X / img_width);
    std::normal_distribution<float> n_x(BLOCK_NUM_X / 2, 3);
    std::normal_distribution<float> n_y(BLOCK_NUM_Y / 2, 1.5); // n(BLOCK_NUM_Y/2, 1.5) 均值, 方差，均值是靠近的中心点；方差越小，中心点越集中，方差越大，分布越平均
    
    std::vector<float> rows(BLOCK_NUM_X);
    std::vector<float> cows(BLOCK_NUM_Y);
    // float foe_x = 0;
    // float foe_y = 0;
    // int index_x = (int)(iter->x * BLOCK_NUM_X / img_width);
    // int index_y = (int)(iter->y * BLOCK_NUM_Y / img_height);

    for (std::size_t i = 0; i != 100; ++i)
    {
        unsigned vy = std::lround(n_y(e)); //取整-最近的整数
        unsigned vx = std::lround(n_x(e)); //取整-最近的整数
        if (vx < rows.size())
            ++rows[vx];
        if (vy < cows.size())
            ++cows[vy];
    }

    
    double sum = 0;
    for (int i = 0; i < result.size(); ++i)
        for (int j = 0; j < result[0].size(); ++j)
        {
            result[i][j] = (rows[i] / 100);
            result[i][j] = result[i][j] * (cows[j] / 100);
            sum += result[i][j];
        }
    // cout << " The_total_gaussian: " << sum << endl;
    return result;
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
    ttc_mean.data = ttc_mean.data/10;//1000
    tc[0] = ttc_mean.data / 10;      // 1000
    cout << "ttc_mean:" << tc[0] << endl << endl;
    ttc_mean_pub_.publish(ttc_mean);
}

void CalcTTC::balanceStrategy(const vector<cv::Point2f>& optical_flow)
{
    int img_width = color.cols;
    int img_height = color.rows;  //int(foe.at<double>(0,0))
    int half_width = img_width / 2;
    double left_sum, right_sum, left_mean, right_mean;
    left_sum = right_sum = left_mean = right_mean = 0;
    int left_cnt, right_cnt;
    left_cnt = right_cnt = 0;

    auto iter_key = keypoints.begin();
    for (int i=0; i<optical_flow.size(); i++)
    {
        // if (iter_key->x < half_width)
        if (iter_key->x < color_x)
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
    cout << "color_x:" << color_x << endl << endl;
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
    foc[0] = force.data;
    cout << "force:" << foc[0] << endl << endl;
    balance_force_pub_.publish(force);
}

void CalcTTC::safeStrategy(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height)
{
    double ttc_max = 0;
    double flow_max = 0;
    double u_OF[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
    double u_TTC[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
    double u_ttc_max = 0;
    double u_flow_max = 0;
    vector<double> result;
    // vector<vector<vector<double>>> ttc_block_median(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)
    vector<vector<vector<double>>> ttc_block_result(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)
    vector<vector<double>> gaussian_result(BLOCK_NUM_X, vector<double>(BLOCK_NUM_Y, 0.0));

    gaussian_result = Gaussian(img_width, img_height);

    for (int row = 0; row < BLOCK_NUM_X; row++)
    {
        for (int col = 0; col < BLOCK_NUM_Y; col++)
        {
            ttc_block_median[row][col].push_back(ttc_block[row][col]);
            //filter averiage result
            ttc_block_result[row][col] = median_filter(ttc_block_median[row][col], 5, 5);
            if (ttc_block_result[row][col][3] > ttc_safe)
                ttc_max = max(ttc_max, ttc_block_result[row][col][3]);
            if (row == 8 && col == 8)
            {
                cout << "ttc_block:" << ttc_block_result[row][col][3] << endl;
            }
            ttc_block_median[row][col].erase(ttc_block_median[row][col].begin());
        }
    }
    // calculate the maxium ttc messgae
    if(ttc_max > 0)
        for (int row = 0; row < BLOCK_NUM_X; row++)
        {
            for (int col = 0; col < BLOCK_NUM_Y; col++)
            {
                if (ttc_block_result[row][col][3] > ttc_safe)
                    continue;
                else
                {
                    u_TTC[row][col] = gaussian_result[row][col] * ttc_block_result[row][col][3] / ttc_max;
                    if (u_ttc_max < u_TTC[row][col])
                    {
                        u_ttc_max = u_TTC[row][col];
                        u_ttc_safe[0] = row;
                        u_ttc_safe[1] = col;
                    }
                }
            }
        }
}

geometry_msgs::Point CalcTTC::pub_trance(int row, int col, double u)
{
    geometry_msgs::Point result;
    result.x = row;
    result.y = col;
    result.z = u;

    return result;
}

double CalcTTC::TTC_messgae_V2(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const int img_width, const int img_height, const vector<vector<double>> gaussian_result)
{
    double TTC_result;
    double ttc_max = 0;
    double u_TTC[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
    double u_ttc_max = 0;
    int ttc_zero = 0;
    double u_ttc_on = 0.01;
    vector<vector<vector<double>>> ttc_block_result(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)

    for (int row = 0; row < BLOCK_NUM_X; row++)
    {
        for (int col = 0; col < BLOCK_NUM_Y; col++)
        {
            ttc_block_median[row][col].push_back(ttc_block[row][col]);
            // filter averiage result
            ttc_block_result[row][col] = median_filter(ttc_block_median[row][col], 5, 5);
            if (ttc_block_result[row][col][3] == 0)
            {
                ttc_zero += 1;
            }
            ttc_max = max(ttc_max, ttc_block_result[row][col][3]);
            
            ttc_block_median[row][col].erase(ttc_block_median[row][col].begin());
        }
    }

    // calculate the maxium ttc messgae
    if (ttc_max > 0)
        for (int row = 0; row < BLOCK_NUM_X; row++)
        {
            for (int col = 0; col < BLOCK_NUM_Y; col++)
            {
                u_TTC[row][col] = gaussian_result[row][col] * (1 - ttc_block_result[row][col][3] / ttc_max);
                if (u_TTC[row][col] > u_ttc_on)
                    is_TTC_u[row][col] = true;
                // if (ttc_block_result[row][col][3] > ttc_safe || ttc_block_result[row][col][3] == 0)
                //     continue;
                // else
                // {
                //     u_TTC[row][col] = (1 - ttc_block_result[row][col][3]/ttc_max); // * ttc_min / ttc_block_result[row][col][3];
                //     if (u_ttc_max < u_TTC[row][col])
                //     {
                //         u_ttc_max = u_TTC[row][col];
                //         u_ttc_safe[0] = row;
                //         u_ttc_safe[1] = col;
                //     }
                // }
            }
        }
    return u_ttc_max;
}

double CalcTTC::TTC_messgae(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const int img_width, const int img_height, const vector<vector<double>> gaussian_result)
{
    double TTC_result;
    double ttc_min = 10 * ttc_safe;
    double u_TTC[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
    double u_ttc_max = 0;
    int ttc_zero = 0;
    double u_ttc_on = 0.02;//0.02
    
    vector<vector<vector<double>>> ttc_block_result(BLOCK_NUM_X, vector<vector<double>>(BLOCK_NUM_Y, vector<double>(6, 0.0))); // filter matrix  (BLOCK_NUM_X*BLOCK_NUM_Y*6)

    for (int row = 0; row < BLOCK_NUM_X; row++)
    {
        for (int col = 0; col < BLOCK_NUM_Y; col++)
        {
            ttc_block_median[row][col].push_back(ttc_block[row][col]);
            // filter averiage result
            ttc_block_result[row][col] = median_filter(ttc_block_median[row][col], 5, 5);
            // if (ttc_block_result[row][col][3] < 1)
            // {
            //     is_TTC_u[row][col] = true;
            // }
            if (ttc_block_result[row][col][3] == 0)
            {
                // is_TTC_u[row][col] = true;
                ttc_zero += 1;
            }
            if ((ttc_block_result[row][col][3] < ttc_safe) && (ttc_block_result[row][col][3] > 0))
            {
                // ttc_min = min(ttc_min, ttc_block_result[row][col][3]);
                if (ttc_min > ttc_block_result[row][col][3])
                {
                    ttc_min = ttc_block_result[row][col][3];
                    ttc_min_pos[0] = row;
                    ttc_min_pos[1] = col;
                }
            }
            ttc_block_median[row][col].erase(ttc_block_median[row][col].begin());
        }
    }
    // calculate the maxium ttc messgae
    if (ttc_min > 0)
        for (int row = 0; row < BLOCK_NUM_X; row++)
        {
            for (int col = 0; col < BLOCK_NUM_Y; col++)
            {
                if (ttc_block_result[row][col][3] > ttc_safe || ttc_block_result[row][col][3] == 0)
                {
                    // if (ttc_block_result[row][col][3] == 0)
                    //     u_TTC[row][col] = gaussian_result[row][col];

                    // if (u_TTC[row][col] > u_ttc_on)
                    // {
                    //     is_TTC_u[row][col] = true;
                    // }
                    continue;
                }
                else
                {
                    // u_TTC[row][col] = gaussian_result[row][col] * ttc_block_result[row][col][3] / ttc_min;
                    u_TTC[row][col] = gaussian_result[row][col] * ttc_min / ttc_block_result[row][col][3];
                    if (u_TTC[row][col] > u_ttc_on)
                    {
                        ttc_pub.data.push_back(pub_trance(row * BLOCK_NUM_X / img_width, col * BLOCK_NUM_Y / img_height, u_TTC[row][col]));
                        is_TTC_u[row][col] = true;
                    }
                    if(row == 8 && col ==8)
                    {
                        cout << "u_TTC:" << u_TTC[row][col]<< endl;
                    }

                    if (u_ttc_max < u_TTC[row][col])
                    {
                        u_ttc_max = u_TTC[row][col];
                        u_ttc_safe[0] = row;
                        u_ttc_safe[1] = col;
                    }
                }
            }
        }
    return u_ttc_max;
}

double CalcTTC::OF_messgae(const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height, const vector<vector<double>> gaussian_result)
{
    double OF_result;
    double flow_max = 0;
    double u_OF[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};
    double u_flow_max = 0;
    double u_OF_on = 0.001;

    auto iter_key = keypoints.begin();
    double flow_block[BLOCK_NUM_X][BLOCK_NUM_Y] = {0}; // 计算块中平均optical_flow
    int kp_flow_cnt[BLOCK_NUM_X][BLOCK_NUM_Y] = {0};   // 统计图像块中关键点个数
    
    // calculate the total flow value for each block
    for (int i = 0; i < optical_flow.size(); i++)
    {
        int index_x = (int)(iter_key->x * BLOCK_NUM_X / img_width);
        int index_y = (int)(iter_key->y * BLOCK_NUM_Y / img_height);
        // if (iter_key->x < half_width)
        flow_block[index_x][index_y] += optical_flow[i].x;
        kp_flow_cnt[index_x][index_y]++;
        iter_key++;
    }
    // calculate the average ttc for each block
    double flow_average[BLOCK_NUM_X][BLOCK_NUM_Y] = {0}; // 计算块中平均碰撞时间
    for (int i = 0; i < BLOCK_NUM_X; i++)
    {
        for (int j = 0; j < BLOCK_NUM_Y; j++)
        {
            if (kp_flow_cnt[i][j])
            {
                flow_average[i][j] = flow_block[i][j] / kp_flow_cnt[i][j];
            }
        }
    }

    // calculate the maxium flow in all block
    int max_row = 0;
    int max_col = 0;
    for (int row = 0; row < BLOCK_NUM_X; row++)
    {
        for (int col = 0; col < BLOCK_NUM_Y; col++)
        {
            if (abs(flow_average[row][col]) > flow_safe)
            {
                if (flow_max < abs(flow_average[row][col]))
                {
                    flow_max = abs(flow_average[row][col]);
                    max_row = row;
                    max_col = col;
                }
            }
            if (row == 8 && col == 8)
            {
                cout << "flow_block:" << abs(flow_average[row][col]) << endl;
            }
        }
    }

    cout << "max_row:" << max_row << endl;
    cout << "max_col:" << max_col << endl;
    cout << "flow_max:" << flow_max << endl;
    // calculate the maxium ttc messgae
    if (flow_max > 0)
        for (int row = 0; row < BLOCK_NUM_X; row++)
        {
            for (int col = 0; col < BLOCK_NUM_Y; col++)
            {
                if (abs(flow_average[row][col]) < flow_safe)
                {
                    // if (abs(flow_average[row][col]) == 0)
                    //     is_OF_u[row][col] = true;
                    continue;
                }
                else
                {
                    // is_OF_u[row][col] = true;
                    u_OF[row][col] = gaussian_result[row][col]*abs(flow_average[row][col]) / flow_max;
                    // u_OF[row][col] = abs(flow_average[row][col]) / flow_max;
                    if (u_OF[row][col] > u_OF_on)
                    {
                        of_pub.data.push_back(pub_trance(row * BLOCK_NUM_X / img_width, col * BLOCK_NUM_Y / img_height, u_OF[row][col]));
                        is_OF_u[row][col] = true;
                    }
                    // u_OF[row][col] = abs(flow_average[row][col]) / flow_max;
                    // u_flow_max = max(u_flow_max, flow_average[row][col]);
                    if (u_flow_max < u_OF[row][col])
                    {
                        u_flow_max = u_OF[row][col];
                        u_flow_safe[0] = row;
                        u_flow_safe[1] = col;
                    }
                }
            }
        }

    return u_flow_max;
}

vector<vector<int>> CalcTTC::advStrategy(const double ttc_block[BLOCK_NUM_X][BLOCK_NUM_Y], const vector<cv::Point2f> &optical_flow, const int img_width, const int img_height)
{
    vector<vector<int>> on_of_result;
    double ttc_result;
    double of_result;
    
    vector<double> result;
    vector<vector<double>> gaussian_result(BLOCK_NUM_X, vector<double>(BLOCK_NUM_Y, 0.0));

    gaussian_result = Gaussian(img_width, img_height);
    ttc_result = TTC_messgae(ttc_block, img_width, img_height, gaussian_result);
    // ttc_result = TTC_messgae_V2(ttc_block, img_width, img_height, gaussian_result);
    of_result = OF_messgae(optical_flow, img_width, img_height, gaussian_result);
    for(int row = 0; row < BLOCK_NUM_X; row++)
        for (int col = 0; col < BLOCK_NUM_Y; col++)
        {
            vector<int> of_on;
            of_on.push_back(row);
            of_on.push_back(col);
            if (is_TTC_u[row][col] || is_OF_u[row][col])
            {
                on_of_result.push_back(of_on);
                is_TTC_u[row][col] = false;
                is_OF_u[row][col] = false;
            }
        }
    TTC_message_pub_.publish(ttc_pub);
    OF_message_pub_.publish(of_pub);
    of_pub.data.clear(); // = geometry_msgs::Point[];
    ttc_pub.data.clear(); // = geometry_msgs::Point[];
    return on_of_result;
}

void CalcTTC::drawOptFlowMap(const cv::Mat &flow, cv::Mat &cflowmap, int step, double, const cv::Scalar &color)
{
    for (int y = 0; y < cflowmap.rows; y += step)
        for (int x = 0; x < cflowmap.cols; x += step)
        {
            const cv::Point2f &fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflowmap, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
                 color);
            cv::circle(cflowmap, cv::Point(x, y), 2, color, -1);

            // return 0;
        }
}

void CalcTTC::imageCb(const sensor_msgs::ImageConstPtr& msg)
// void CalcTTC::imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
{
    if (num % FRAME_STEP != 0) //每隔两个周期执行一次
    {
        num++;
        return;
    }
    try
    {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);  //ROS图像和OpenCV图像的转换
        // cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);//压缩图像解码
        color = cv_ptr->image.clone();
        // color = cv_ptr_compressed->image.clone();
        cv::imshow(OPENCV_WINDOW, color);
        cv::waitKey(1);
        cout << num << endl;
        cout << msg->header;

        int img_width = color.cols;
        int img_height = color.rows;
        // cout << "img_width" << img_width << "    img_height" << img_height << endl;

        /**********************2022-03-12*****************/
        // cv::Mat gray, flow, cflow,color_V2;
        // color_V2 = color.clone();
        // cv::cvtColor(color_V2, gray, cv::COLOR_BGR2GRAY);
        // // cv::calcOpticalFlowFarneback(prevgray, gray, flow_calc, 0.5, 3, 15, 3, 5, 1.2, 0);
        // if (prevgray.data)
        // {
        //     cv::calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        //     cv::cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
        //     // drawOptFlowMap(flow, color_V2, 16, 1.5, cv::Scalar(0, 255, 0));
        //     for(int row = 0; row < color_V2.rows; row++)
        //         for(int col = 0; col < color_V2.cols; col++)
        //         {
        //             const cv::Point2f fxy = flow.at<cv::Point2f>(row,col);
        //             if(fxy.x > 5 || fxy.y > 5)
        //             {
        //                 cv::circle(color_V2,cv::Point(row,col),1,cv::Scalar(0,255,0),1);
        //                 // cv::circle(img_show, cv::Point(int(foe.at<double>(0, 0)), int(foe.at<double>(1, 0))), 5, cv::Scalar(0, 0, 240), 3);
        //             }
        //         }
        //     cv::imshow("flow", color_V2);
        //     cv::waitKey(1);
        // }

        // // if (cv::waitKey(30) >= 0)
        // // {
        // //     break;
        // // }
        // cv::waitKey(10);
        // prevgray = gray.clone();
        /**********************2022-03-12*****************/

        if (num % (15 * FRAME_STEP) == 0) //每隔16个周期执行一次  num % (15*FRAME_STEP) == 0
        {
            keypoints.clear();
            vector<cv::KeyPoint> kps;
            // cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps);
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
        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error, winSize,
                                 3, termcrit, 0, 0.001);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        // cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;

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
            pxvec[0] = optical_flow[i].x + (iter_key->x * iter_key->y * att_rate.x  
                    -  (1 + iter_key->x * iter_key->x) * att_rate.y  +  iter_key->y * att_rate.z);           // 减去旋转分量
            pxvec[1] = optical_flow[i].y - ((1 + iter_key->y * iter_key->y) * att_rate.x  
                    -  iter_key->x * iter_key->y * att_rate.y  -  iter_key->x * att_rate.z);
            iter_key ++;
            
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
        cout << "FOE:" << foe << endl << endl;  
        // cout << "foe:" << [foe.at<double>(0,0),foe.at<double>(1,0),0] << endl << endl;
        foe_x = foe.at<double>(0,0);
        foe_y = foe.at<double>(1,0);
        color_x = foe_x;
        cout << "foex:" << foe_x << endl << endl;
        cout << "foey:" << foe_y << endl << endl;
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
                    ttc_block[i][j] = ttc_ratio*ttc_sum[i][j] / kp_cnt[i][j];
                }
            }
        }
        
        // 画出 keypoints和光流
        cv::Mat img_show = color.clone();
        cv::Scalar line_color;

        // cv::Mat of_show = color.clone();
        // cv::Mat ttc_show = color.clone();
        // cv::Mat optical_show = color.clone();

        cv::circle(img_show, cv::Point(int(foe.at<double>(0,0)), int(foe.at<double>(1,0))), 5, cv::Scalar(0, 0, 240), 3);     // 画FOE
        
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
        //ttcMean_plus(ttc_sum, kp_cnt);

        // 光流平衡
        balanceStrategy(optical_flow);
        // safeStrategy(ttc_block, optical_flow, img_width, img_height);

        vector<vector<int>> show_block;
        vector<cv::Point> start_ttc;
        vector<cv::Point> end_ttc;
        show_block = advStrategy(ttc_block, optical_flow, img_width, img_height);
        for(int i = 0; i < show_block.size(); i++)
        {
            cv::Point i_start_ttc = cv::Point(img_width / BLOCK_NUM_X * (show_block[i][0] - 0.5), img_height / BLOCK_NUM_Y * (show_block[i][1] - 0.5));
            cv::Point i_end_ttc = cv::Point(img_width / BLOCK_NUM_X * (show_block[i][0] + 0.5), img_height / BLOCK_NUM_Y * (show_block[i][1] + 0.5));
            start_ttc.push_back(i_start_ttc);
            end_ttc.push_back(i_end_ttc);
            cv::rectangle(img_show, start_ttc[i], end_ttc[i], cv::Scalar(255, 0, 0), 2);
        }

       
        last_color = color.clone();
        cv::imshow(TTC_WINDOW, img_show);
        cv::waitKey(1);
        
        vw.write(img_show);
        // vw.release();
        // cv::waitKey(1);
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
    vw.open("/home/nvidia/video/out.mp4",      //路径
            cv::VideoWriter::fourcc('X', '2', '6', '4'), //编码格式  fourcc('X', '2', '6', '4')  fourcc('M', 'J', 'P', 'G')
            15,                                          //帧率
            cv::Size(720, 405),                          // 720, 405                   //尺寸
            true);
    // ofw.open("/home/wsh/flow_obs/src/video/of.mp4",      //路径
    //         cv::VideoWriter::fourcc('X', '2', '6', '4'), //编码格式  fourcc('X', '2', '6', '4')  fourcc('M', 'J', 'P', 'G')
    //         15,                                          //帧率
    //         cv::Size(720, 405),                          // 720, 405                   //尺寸
    //         true);
    // ttcw.open("/home/wsh/flow_obs/src/video/ttc.mp4",      //路径
    //         cv::VideoWriter::fourcc('X', '2', '6', '4'), //编码格式  fourcc('X', '2', '6', '4')  fourcc('M', 'J', 'P', 'G')
    //         15,                                          //帧率
    //         cv::Size(720, 405),                          // 720, 405                   //尺寸
    //         true);
    // opticalw.open("/home/wsh/flow_obs/src/video/optical.mp4",      //路径
    //         cv::VideoWriter::fourcc('X', '2', '6', '4'), //编码格式  fourcc('X', '2', '6', '4')  fourcc('M', 'J', 'P', 'G')
    //         15,                                          //帧率
    //         cv::Size(720, 405),                          // 720, 405                   //尺寸
    //         true);
    ros::spin();
    return 0;
}