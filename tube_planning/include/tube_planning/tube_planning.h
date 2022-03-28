#ifndef TUBE_PLANNING_H_
#define TUBE_PLANNING_H_

#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "swarm_msgs/Pipeline.h"



namespace Tube_planning
{
    class GeneratorFist
    {
    private:
        double r_max;
        double total_time;
        int n_order, n_coef, n_poly, n_obj;
        int path_length;

        Eigen::MatrixXd Path, Poly_x, Poly_y;
        Eigen::ArrayXd ts;
        Eigen::RowVector2d p0, v0, a0, pe, ve, ae;

        struct GridMap
        {
            float resolution;
            int width;
            int height;
            Eigen::MatrixXi data;
        };
        GridMap gridmap;

        Eigen::ArrayXi seg_time_k;
        Eigen::ArrayXd max_cuv_ts;
        Eigen::ArrayXi max_cuv_k;
        int rad_n_poly, rad_n_coef;
        double rad_p0, rad_pe;
        struct Radius
        {
            Eigen::MatrixXd min_dis_mat;
            Eigen::ArrayXd max_cuv;
            Eigen::MatrixXd Poly;
        };
        Radius radiusRight, radiusLeft;

        ros::Subscriber path_sub;
        ros::Subscriber gridmap_sub;
        ros::Publisher gen_pub;
        ros::Publisher pipeline_gen_pub;
        ros::Publisher cuv_right_pub, dis_right_pub;
        ros::Publisher tube_right, tube_left;
        

        // swarm_msgs::Pipeline tube;
        // swarm_msgs::Pipeunit unit;
        
        void pathCallback(const nav_msgs::Path::ConstPtr &msg);
        void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

        void allocateTime( const Eigen::MatrixXd& Path);
        void generatorPlan(const Eigen::MatrixXd& waypts, const int axis);
        void generatorObjectFunction(Eigen::MatrixXd& Q_all);
        void generatorConstraint(const Eigen::MatrixXd& waypts, Eigen::MatrixXd& Aeq, Eigen::ArrayXd& beq, const int axis);
        void generatorCalculation();
        void radiusPlan(Eigen::MatrixXd& Poly, const Eigen::ArrayXd& max_cuv,  const Eigen::MatrixXd& min_dis_mat);
        void radiusObjectFunction(Eigen::MatrixXd& Q_all);
        void radiusConstraint(const Eigen::MatrixXd& min_dis_mat, const Eigen::ArrayXd& max_cuv, Eigen::MatrixXd &Aeq, Eigen::ArrayXd &ub, Eigen::ArrayXd &lb);
        void radiusCalculation();
        
        void findObstacle(Eigen::MatrixXd& min_dis_k, const Eigen::ArrayXi& seg_time_k, const Eigen::ArrayXd& tt, const Eigen::MatrixXd& ff, const Eigen::MatrixXd& sign_normal);
        void findMaxCurvature(Eigen::ArrayXi& seg_time, Eigen::ArrayXd& ts, Eigen::ArrayXi& max_cuv_k, Eigen::ArrayXd& route_r, Eigen::ArrayXd& route_l, const Eigen::ArrayXd& flag_dir, const Eigen::MatrixXd& ff1, const Eigen::MatrixXd& ff2, const Eigen::ArrayXd& tt);
        void calcTvec(const double t, const int r, Eigen::MatrixXd& tvec);
        void computeQ(Eigen::MatrixXd& Q_temp, double t1, double t2, const int r);
        void polys_val(const Eigen::MatrixXd& Poly, Eigen::ArrayXd& vals, const Eigen::ArrayXd& tt, const int r);
        double poly_val(const Eigen::MatrixXd& poly, const double t, const int r);
        void polys_val_rad(const Eigen::MatrixXd &Poly, Eigen::ArrayXd &vals, const Eigen::ArrayXd &tt, const int r);
        void wait_for_key();
    public:
        GeneratorFist()
        {
        }
        ~GeneratorFist()
        {
        }
        void init(ros::NodeHandle& nh);
    };
}

#endif