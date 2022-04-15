#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include "tube_planning/tube_planning.h"
// #include <nlopt.hpp>
#include <math.h>
#include <algorithm>
// #include "QuadProg++/Array.hh"
// #include "QuadProg++/QuadProg++.hh"
// #include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
// #include "mgl2/mgl.h"
#include "tube_planning/gnuplot_i.hpp"
#include "swarm_msgs/Pipeline.h"

/* program tree
GeneratorFist::init
|_ pathCallback
|      |_allocateTime
|      |_generatorPlan
|      |   |_generatorObjectFunction
|      |   |     |_computeQ
|      |   |_generatorConstraint
|      |         |_calcTvec
|      |_generatorCalculation
|      |   |_polys_val
|      |   |    |_poly_val
|      |   |_findMaxCurvature
|      |   |_findObstacle
|      |_radiusPlan
|           |_radiusObjectFunction
|           |_radiusConstraint
|_gridmapCallback
*/
namespace Tube_planning
{
    void GeneratorFist::init(ros::NodeHandle &nh)
    {
        nh.param("uav_id", uav_id, 0);
        nh.param("virtual_tube/max_radius", r_max, 0.1);
        nh.param("generator_curve/total_time", total_time, 10.0);
        nh.param("generator_curve/n_order", n_order, 5);
        nh.param("map/info/resolution", gridmap.resolution, (float)0.01);
        nh.param("map/info/width", gridmap.width, 37);
        nh.param("map/info/height", gridmap.height, 30);

        // gridmap_sub = nh.subscribe("/Map/OccupancyGrid", 10, &GeneratorFist::gridmapCallback, this);
        // path_sub = nh.subscribe("/generator_curve/paths", 10, &GeneratorFist::pathCallback, this);
        gridmap_sub = nh.subscribe("/visualization/map", 10, &GeneratorFist::gridmapCallback, this);
        path_sub = nh.subscribe("/expect_pos"+std::to_string(uav_id), 10, &GeneratorFist::pathCallback, this);

        gen_pub = nh.advertise<nav_msgs::Path>("/tube/generator_curve", 10);
        pipeline_gen_pub = nh.advertise<swarm_msgs::Pipeline>("/pipeline/paths", 1, true);
        cuv_right_pub = nh.advertise<nav_msgs::Path>("/tube/max_cuv_r", 10);
        dis_right_pub = nh.advertise<nav_msgs::Path>("/tube/right_dis", 10);
        tube_right = nh.advertise<nav_msgs::Path>("/tube/tube_right", 10);
        tube_left =  nh.advertise<nav_msgs::Path>("/tube/tube_left", 10);
    }

    void GeneratorFist::gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        gridmap.resolution = msg->info.resolution;
        gridmap.width = msg->info.width;
        gridmap.height = msg->info.height;
        gridmap.data = Eigen::MatrixXi::Zero(gridmap.height, gridmap.width);
        gridmap.position_x = msg->info.origin.position.x;
        gridmap.position_y = msg->info.origin.position.y;
        gridmap.position_z = msg->info.origin.position.z;

        for (int i = 0; i < gridmap.height; i++)
        {
            for (int j = 0; j < gridmap.width; j++)
            {
                gridmap.data(i, j) = msg->data[i * gridmap.width + j];
            }
        }
        // std::cout << gridmap.data << std::endl << std::endl;
    }

    void GeneratorFist::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        path_length = end(msg->poses) - begin(msg->poses);
        Path.resize(path_length, 3);
        for (int i = 0; i < path_length; i++)
        { /* Path是一个N*3的矩阵，每一列代表一个维数 */
            Path(i, 0) = msg->poses[i].pose.position.x;
            Path(i, 1) = msg->poses[i].pose.position.y;
            // Path(i,2) = msg->poses[i].pose.position.z;
            Path(i, 2) = r_max;
        }

        allocateTime(Path);

        /* 初始化起止点参数 */
        p0 = Path.block(0, 0, 1, 2);
        v0 = Path.block(1, 0, 1, 2) - Path.block(0, 0, 1, 2);
        v0 = v0 / v0.norm();
        a0 << 0, 0;
        pe = Path.block(path_length - 1, 0, 1, 2);
        ve = Path.block(path_length - 1, 0, 1, 2) - Path.block(path_length - 2, 0, 1, 2);
        ve = ve / ve.norm();
        ae << 0, 0;
        /* 规划X轴 */
        generatorPlan(Path.block(0, 0, path_length, 1), 0);
        /* 规划Y轴 */
        generatorPlan(Path.block(0, 1, path_length, 1), 1);

        generatorCalculation();

        /* 规划右侧半径 */
        radiusPlan(radiusRight.Poly, radiusRight.max_cuv, radiusRight.min_dis_mat);
        /* 规划左侧半径 */
        radiusPlan(radiusLeft.Poly, radiusLeft.max_cuv, radiusLeft.min_dis_mat);

        radiusCalculation();


    }

    void GeneratorFist::radiusCalculation()
    {
        /* 数值计算生成线及其导数 */
        Eigen::ArrayXd rad_r, rad_tt, rad_l;
        rad_tt = Eigen::VectorXd::LinSpaced(30 * n_poly, max_cuv_ts(0), max_cuv_ts(max_cuv_ts.size() - 1));
        polys_val_rad(radiusRight.Poly, rad_r, rad_tt, 0);
        polys_val_rad(radiusLeft.Poly, rad_l, rad_tt, 0);

        Eigen::ArrayXd xx, xx1, yy, yy1, tt;
        tt = Eigen::VectorXd::LinSpaced(30 * n_poly, ts(0), ts(path_length - 1));
        // tt = Eigen::VectorXd::LinSpaced(30, ts(0), ts(1));
        polys_val(Poly_x, xx, tt, 0);
        polys_val(Poly_y, yy, tt, 0);
        polys_val(Poly_x, xx1, tt, 1);
        polys_val(Poly_y, yy1, tt, 1);

        Eigen::Matrix2d rot;
        rot << 0, -1, 1, 0;
        Eigen::MatrixXd ff(2, 30 * n_poly), ff1(2, 30 * n_poly);
        ff << xx.transpose(), yy.transpose();
        ff1 << xx1.transpose(), yy1.transpose();
        
        // 公式法向量
        Eigen::MatrixXd ff1_norm(2, 30 * n_poly);
        ff1_norm << ff1.colwise().norm(), ff1.colwise().norm();
        Eigen::MatrixXd ff1_unit = ff1.array() / ff1_norm.array();
        // 定向法向量
        Eigen::MatrixXd sign_normal = rot * ff1_unit;

        
        nav_msgs::Path tube_right_curve, tube_left_curve;
        geometry_msgs::PoseStamped this_pose_stamped;

        swarm_msgs::Pipeline tube;
        swarm_msgs::Pipeunit unit;
        Eigen::ArrayXd fo_x_r(30 * n_poly), fo_y_r(30 * n_poly);
        Eigen::ArrayXd fo_x_l(30 * n_poly), fo_y_l(30 * n_poly);
        for (int i = 0; i < 30 * n_poly; i++)
        {
            fo_x_r(i) = ff(0, i) + 1 / rad_r(i) * sign_normal(0, i);
            fo_y_r(i) = ff(1, i) + 1 / rad_r(i) * sign_normal(1, i);
            fo_x_l(i) = ff(0, i) - 1 / rad_l(i) * sign_normal(0, i);
            fo_y_l(i) = ff(1, i) - 1 / rad_l(i) * sign_normal(1, i);

            unit.right.x = fo_x_r(i);
            unit.right.y = fo_y_r(i);
            unit.right.z = 1.8;
            unit.left.x = fo_x_l(i);
            unit.left.y = fo_y_l(i);
            unit.left.z = 1.8;
            unit.middle.x = ff(0, i);
            unit.middle.y = ff(1, i);
            unit.middle.z = 1.8;
            tube.units.push_back(unit);

            this_pose_stamped.pose.position.x = fo_x_r(i);
            this_pose_stamped.pose.position.y = fo_y_r(i);
            this_pose_stamped.pose.position.z = 1.8;
            tube_right_curve.poses.push_back(this_pose_stamped);

            this_pose_stamped.pose.position.x = fo_x_l(i);
            this_pose_stamped.pose.position.y = fo_y_l(i);
            this_pose_stamped.pose.position.z = 1.8;
            tube_left_curve.poses.push_back(this_pose_stamped);
        }
        count = count + 1;
        if (count > 10)
        {
            count = 1;
            pipeline_gen_pub.publish(tube);
            tube_right.publish(tube_right_curve);
            tube_left.publish(tube_left_curve);
        }

        


            

        // Gnuplot g1("lines");
        // std::vector<double> x_t, y_t;
        // for (int i = 0; i < path_length; i++)
        // {
        //     x_t.push_back(Path(i, 0));
        //     y_t.push_back(Path(i, 1));
        // }
        // g1.set_grid();
        // g1.set_pointsize(3);
        // g1.set_style("points").plot_xy(x_t, y_t, "Path point"); //设置形状
        // g1.set_style("lines").plot_xy(xx, yy,"generator curve"); //设置形状
        // g1.set_style("lines").plot_xy(fo_x_r, fo_y_r,"offset curve");
        // g1.set_style("lines").plot_xy(fo_x_l, fo_y_l,"offset curve");
        // g1.set_xautoscale();
        // g1.set_yautoscale();
        // wait_for_key();
        



    }

    void GeneratorFist::polys_val_rad(const Eigen::MatrixXd &Poly, Eigen::ArrayXd &vals, const Eigen::ArrayXd &tt, const int r)
    {

        int idx = 0;
        int N = tt.size();
        vals = Eigen::ArrayXd::Zero(N);
        for (int i = 0; i < N; i++)
        {
            double t = tt(i);
            if (t < max_cuv_ts(idx))
            {
                vals(i) = 0;
            }
            else
            {
                while (idx < max_cuv_ts.size() - 1 && t > max_cuv_ts(idx + 1) + 0.0001)
                {
                    idx = idx + 1;
                }
                vals(i) = poly_val(Poly.block(0, idx, n_coef, 1), t, r);
            }
        }
    }

    void GeneratorFist::radiusPlan(Eigen::MatrixXd& Poly, const Eigen::ArrayXd& max_cuv,  const Eigen::MatrixXd& min_dis_mat)
    {
        rad_p0 = max_cuv(0);
        rad_pe = max_cuv(max_cuv.size() - 1);

        rad_n_poly = max_cuv.size() - 1;
        rad_n_coef = n_order + 1;

        Eigen::MatrixXd Q_all = Eigen::MatrixXd::Zero(rad_n_poly * rad_n_coef, rad_n_poly * rad_n_coef);
        radiusObjectFunction(Q_all);
        Eigen::ArrayXd b_all = Eigen::ArrayXd::Zero(Q_all.rows());

        Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(4*rad_n_poly+4,rad_n_coef*rad_n_poly);
        Eigen::ArrayXd ub = Eigen::ArrayXd::Zero(4*rad_n_poly+4);
        Eigen::ArrayXd lb = Eigen::ArrayXd::Zero(4*rad_n_poly+4);
        radiusConstraint(min_dis_mat, max_cuv, Aeq, ub, lb);

        Eigen::VectorXd QPSolution;
        Eigen::SparseMatrix<double> hessian;
        Eigen::SparseMatrix<double> linearMatrix;
        hessian = Q_all.sparseView();
        linearMatrix = Aeq.sparseView();

        OsqpEigen::Solver solver;

        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(rad_n_poly * rad_n_coef);
        solver.data()->setNumberOfConstraints(4*rad_n_poly+4);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(b_all);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);

        solver.initSolver();
        solver.solveProblem();
        QPSolution = solver.getSolution();
        Eigen::Map<Eigen::MatrixXd> poly_axis(QPSolution.data(), rad_n_coef, rad_n_poly);
        Poly = poly_axis;
    }

    void GeneratorFist::radiusConstraint(const Eigen::MatrixXd& min_dis_mat, const Eigen::ArrayXd& max_cuv, Eigen::MatrixXd &Aeq, Eigen::ArrayXd &ub, Eigen::ArrayXd &lb)
    {
        // start/terminal pva constraints
        Eigen::MatrixXd tvec;

        calcTvec(max_cuv_ts(0), 0, tvec);
        Aeq.block(0, 0, 1, rad_n_coef) = tvec;
        calcTvec(max_cuv_ts(0), 1, tvec);
        Aeq.block(1, 0, 1, rad_n_coef) = tvec;

        calcTvec(max_cuv_ts(max_cuv_ts.size() - 1), 0, tvec);
        Aeq.block(2, rad_n_coef * (rad_n_poly - 1), 1, n_coef) = tvec;
        calcTvec(max_cuv_ts(max_cuv_ts.size() - 1), 1, tvec);
        Aeq.block(3, rad_n_coef * (rad_n_poly - 1), 1, n_coef) = tvec;

        ub.head(4) << 100, 0,  100, 0;
        lb.head(4) << rad_p0, 0,  rad_pe, 0;

        // mid p constraints    (n_ploy-1 equations)
        int neq = 3;
        for (int i = 0; i < rad_n_poly - 1; i++)
        {
            Eigen::MatrixXd tvec_p, tvec_v;
            calcTvec(max_cuv_ts(i + 1), 0, tvec_p);
            calcTvec(max_cuv_ts(i + 1), 1, tvec_v);
            neq = neq + 1;
            Aeq.block(neq, rad_n_coef * i, 1, 2 * rad_n_coef) << tvec_p, -1 * tvec_p;
            ub(neq) = 0;
            lb(neq) = 0;
            neq = neq + 1;
            Aeq.block(neq, rad_n_coef * i, 1, 2 * rad_n_coef) << tvec_v, -1 * tvec_v;
            ub(neq) = 0;
            lb(neq) = 0;
        }
        // std::cout << min_dis_mat.rows() << std::endl;
        // std::cout << rad_n_poly << std::endl;
        for (int i = 0; i < rad_n_poly; i++)
        {
            neq = neq + 1;
            Eigen::MatrixXd tvec;
            calcTvec(max_cuv_ts(i), 0, tvec);
            Aeq.block(neq, rad_n_coef * i, 1, rad_n_coef) = tvec;
            // lb(neq) = max_cuv(i);
            lb(neq) = 1 / r_max;
            ub(neq) = 1000;
            
            neq = neq + 1;
            calcTvec(min_dis_mat(i, 2), 0, tvec);
            Aeq.block(neq, rad_n_coef * i, 1, rad_n_coef) = tvec;
            lb(neq) = 1 / (min_dis_mat(i, 1)+0.01);
            ub(neq) = 1000;
        }
    }

    void GeneratorFist::radiusObjectFunction(Eigen::MatrixXd& Q_all)
    {
        Eigen::MatrixXd Q_all0 = Eigen::MatrixXd::Zero(rad_n_poly * rad_n_coef, rad_n_poly * rad_n_coef);
        for (int i = 0; i < rad_n_poly; i++)
        {
            Eigen::MatrixXd Q_temp;
            computeQ(Q_temp, max_cuv_ts(i), max_cuv_ts(i + 1), 1);
            Q_all0.block(i * rad_n_coef, i * rad_n_coef, rad_n_coef, rad_n_coef) = Q_temp;
        }

        Eigen::MatrixXd Q_all1 = Eigen::MatrixXd::Zero(rad_n_poly * rad_n_coef, rad_n_poly * rad_n_coef);
        for (int i = 0; i < rad_n_poly; i++)
        {
            Eigen::MatrixXd Q_temp;
            computeQ(Q_temp, max_cuv_ts(i), max_cuv_ts(i + 1), 0);
            Q_all1.block(i * rad_n_coef, i * rad_n_coef, rad_n_coef, rad_n_coef) = Q_temp;
        }
        
        double w1 = 1;
        double w2 = 0.2;
        w1 = w1 / (w1 + w2);
        w2 = w2 / (w1 + w2);
        Q_all = w1 * Q_all0 + w2 * Q_all1;

    }

    void GeneratorFist::allocateTime(const Eigen::MatrixXd &Path)
    { /* 分配时间，最终输出一个时间分段列向量 */
        Eigen::RowVector2d Path_first_col = Path.block(0, 0, 1, 2);
        Eigen::MatrixXd delta_Path = Path.block(0, 0, path_length, 2).rowwise() - Path_first_col;
        Eigen::ArrayXd dis = delta_Path.rowwise().norm();
        ts = total_time * dis / dis(path_length - 1);
    }

    void GeneratorFist::generatorPlan(const Eigen::MatrixXd &waypts, const int axis)
    {
        n_poly = waypts.rows() - 1;
        n_coef = n_order + 1;
        n_obj = 2;

        Eigen::MatrixXd Q_all = Eigen::MatrixXd::Zero(n_poly * n_coef, n_poly * n_coef);
        generatorObjectFunction(Q_all);
        Eigen::ArrayXd b_all = Eigen::ArrayXd::Zero(Q_all.rows());
        // std::cout << Q_all << std::endl;

        Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(4 * n_poly + 2, n_poly * n_coef);
        Eigen::ArrayXd beq = Eigen::ArrayXd::Zero(4 * n_poly + 2);
        generatorConstraint(waypts, Aeq, beq, axis);
        // Eigen::MatrixXd Aieq = Eigen::MatrixXd::Zero(1, n_poly * n_coef);
        // Eigen::ArrayXd bieq = Eigen::ArrayXd::Zero(1);
        // std::cout << "Aeq" << std::endl;
        // std::cout << Aeq << std::endl;
        // std::cout << "beq" << std::endl;
        // std::cout << beq << std::endl;

        Eigen::VectorXd QPSolution;
        Eigen::SparseMatrix<double> hessian;
        Eigen::SparseMatrix<double> linearMatrix;
        hessian = Q_all.sparseView();
        linearMatrix = Aeq.sparseView();
        // std::cout << "linearMatrix" << linearMatrix << std::endl;

        OsqpEigen::Solver solver;

        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n_poly * n_coef);
        solver.data()->setNumberOfConstraints(4 * n_poly + 2);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(b_all);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setLowerBound(beq);
        solver.data()->setUpperBound(beq);

        solver.initSolver();
        solver.solveProblem();
        QPSolution = solver.getSolution();
        Eigen::Map<Eigen::MatrixXd> poly_axis(QPSolution.data(), n_coef, n_poly);

        if (axis == 0)
        {
            Poly_x = poly_axis;
            // std::cout << "x: " << Poly_x << std::endl;
        }
        else
        {
            Poly_y = poly_axis;
            // std::cout << "y: " << Poly_y << std::endl;
        }
    }

    void GeneratorFist::generatorObjectFunction(Eigen::MatrixXd &Q_all)
    { /* 生成目标函数 */
        for (int i = 0; i < n_poly; i++)
        {
            Eigen::MatrixXd Q_temp;
            computeQ(Q_temp, ts(i), ts(i + 1), n_obj);
            Q_all.block(i * n_coef, i * n_coef, n_coef, n_coef) = Q_temp;
        }
    }

    void GeneratorFist::computeQ(Eigen::MatrixXd &Q_temp, double t1, double t2, const int r)
    {
        Eigen::ArrayXd T = Eigen::ArrayXd::Zero((n_order - r) * 2 + 1, 1);
        for (int i = 0; i < (n_order - r) * 2 + 1; i++)
        {
            T(i) = pow(t2, i + 1) - pow(t1, i + 1);
        }

        Q_temp = Eigen::MatrixXd::Zero(n_coef, n_coef);
        for (int i = r + 1; i <= n_order + 1; i++)
        {
            for (int j = i; j <= n_order + 1; j++)
            {
                int k1 = i - r - 1;
                int k2 = j - r - 1;
                int k = k1 + k2 + 1;
                Q_temp(i - 1, j - 1) = std::tgamma(k1 + r + 1) / std::tgamma(k1 + 1) * std::tgamma(k2 + r + 1) / std::tgamma(k2 + 1) / k * T(k - 1);
                Q_temp(j - 1, i - 1) = Q_temp(i - 1, j - 1);
            }
        }
    }

    void GeneratorFist::generatorConstraint(const Eigen::MatrixXd &waypts, Eigen::MatrixXd &Aeq, Eigen::ArrayXd &beq, const int axis)
    { /* 生成约束 */
        // start/terminal pva constraints
        Eigen::MatrixXd tvec;

        calcTvec(ts(0), 0, tvec);
        Aeq.block(0, 0, 1, n_coef) = tvec;
        calcTvec(ts(0), 1, tvec);
        Aeq.block(1, 0, 1, n_coef) = tvec;
        calcTvec(ts(0), 2, tvec);
        Aeq.block(2, 0, 1, n_coef) = tvec;

        calcTvec(ts(ts.size() - 1), 0, tvec);
        Aeq.block(3, n_coef * (n_poly - 1), 1, n_coef) = tvec;
        calcTvec(ts(ts.size() - 1), 1, tvec);
        Aeq.block(4, n_coef * (n_poly - 1), 1, n_coef) = tvec;
        calcTvec(ts(ts.size() - 1), 2, tvec);
        Aeq.block(5, n_coef * (n_poly - 1), 1, n_coef) = tvec;

        beq.head(6) << p0(axis), v0(axis), a0(axis), pe(axis), ve(axis), ae(axis);

        // mid p constraints    (n_ploy-1 equations)
        int neq = 5;
        for (int i = 0; i < n_poly - 1; i++)
        {
            neq = neq + 1;
            calcTvec(ts(i + 1), 0, tvec);
            Aeq.block(neq, n_coef * (i + 1), 1, n_coef) = tvec;
            beq(neq) = waypts(i + 1);
        }

        // continuous constraints  ((n_poly-1)*3 equations)
        for (int i = 0; i < n_poly - 1; i++)
        {
            neq = neq + 1;
            calcTvec(ts(i + 1), 0, tvec);
            Aeq.block(neq, n_coef * i, 1, 2 * n_coef) << tvec, -1 * tvec;
            neq = neq + 1;
            calcTvec(ts(i + 1), 1, tvec);
            Aeq.block(neq, n_coef * i, 1, 2 * n_coef) << tvec, -1 * tvec;
            neq = neq + 1;
            calcTvec(ts(i + 1), 2, tvec);
            Aeq.block(neq, n_coef * i, 1, 2 * n_coef) << tvec, -1 * tvec;
        }
    }

    void GeneratorFist::calcTvec(const double t, const int r, Eigen::MatrixXd &tvec)
    {
        tvec = Eigen::MatrixXd::Zero(1, n_order + 1);
        for (int i = r + 1; i <= n_order + 1; i++)
        {
            tvec(i - 1) = std::tgamma(i) / std::tgamma(i - r) * pow(t, i - r - 1);
        }
    }

    void GeneratorFist::generatorCalculation()
    {

        /* 数值计算生成线及其导数 */
        Eigen::ArrayXd xx, xx1, xx2, yy, yy1, yy2, tt;
        tt = Eigen::VectorXd::LinSpaced(30 * n_poly, ts(0), ts(path_length - 1));
        // tt = Eigen::VectorXd::LinSpaced(30, ts(0), ts(1));
        polys_val(Poly_x, xx, tt, 0);
        polys_val(Poly_y, yy, tt, 0);
        polys_val(Poly_x, xx1, tt, 1);
        polys_val(Poly_y, yy1, tt, 1);
        polys_val(Poly_x, xx2, tt, 2);
        polys_val(Poly_y, yy2, tt, 2);

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped this_pose_stamped;
         for (int i = 0; i < 30 * n_poly; i++)
        { /* 装填路径点 */
            this_pose_stamped.header.stamp = ros::Time::now();
            this_pose_stamped.header.frame_id = "map";
            this_pose_stamped.pose.position.x = xx(i);
            this_pose_stamped.pose.position.y = yy(i);
            this_pose_stamped.pose.position.z = 1.8;

            path.poses.push_back(this_pose_stamped);
        }
        if (count > 5)
        {
            gen_pub.publish(path);
        }
        
        // pipeline_pub.publish(tube);

        // Gnuplot g1("lines");
        // std::vector<double> x_t, y_t;
        // for (int i = 0; i < path_length; i++)
        // {
        //     x_t.push_back(Path(i, 0));
        //     y_t.push_back(Path(i, 1));
        // }
        // g1.set_grid();
        // g1.set_pointsize(3);
        // g1.set_style("points").plot_xy(x_t, y_t, "Path point"); //设置形状
        // g1.set_style("lines").plot_xy(xx, yy, "generator curve"); //设置形状
        // g1.set_xautoscale();
        // g1.set_yautoscale();
        // wait_for_key();

        Eigen::Matrix2d rot;
        rot << 0, -1, 1, 0;
        Eigen::MatrixXd ff(2, 30 * n_poly), ff1(2, 30 * n_poly), ff2(2, 30 * n_poly);
        ff << xx.transpose(), yy.transpose();
        ff1 << xx1.transpose(), yy1.transpose();
        ff2 << xx2.transpose(), yy2.transpose();
        
        // 公式法向量
        Eigen::MatrixXd ff1_norm(2, 30 * n_poly);
        ff1_norm << ff1.colwise().norm(), ff1.colwise().norm();
        Eigen::MatrixXd ff1_unit = ff1.array() / ff1_norm.array();
        // 定向法向量
        Eigen::MatrixXd sign_normal = rot * ff1_unit;
        Eigen::MatrixXd normal_tmp = ff1_unit.block(0, 1, 2, 30 * n_poly - 1) - ff1_unit.block(0, 0, 2, 30 * n_poly - 1);
        Eigen::MatrixXd normal(2, 30 * n_poly);
        normal << normal_tmp, normal_tmp.block(0, 30 * n_poly - 2, 2, 1);
        Eigen::ArrayXd flag_dir = (sign_normal.array() * normal.array()).colwise().sum();

        Eigen::ArrayXi k_cuv;
        Eigen::ArrayXd route_r, route_l;
        Eigen::ArrayXd ts_cuv;
        // Eigen::ArrayXi seg_time_k;
        findMaxCurvature(seg_time_k, ts_cuv, k_cuv, route_r, route_l, flag_dir, ff1, ff2, tt);
        // std::cout << "重新分段处下标 " << std::endl
        //           << seg_time_k << std::endl;

        /* 虽然进行了重新分段，但是最终优化时的分段是以曲率最大处作为断点，因此
        需要对曲率最大处数组进行扩充，保证其涵盖了头尾。 */
        int max_cuv_tail = k_cuv(k_cuv.size() - 1);
        int max_cuv_head = k_cuv(0);
        int seg_time_k_tail = seg_time_k(seg_time_k.size() - 1);
        int seg_time_k_head = 0;
        Eigen::ArrayXd max_cuv_r, max_cuv_l;

        /* 临时修补 */
        route_r(0) = 1 / r_max;
        route_r(route_r.size() - 1) = 1 / r_max;
        route_l(0) = 1 / r_max;
        route_l(route_l.size() - 1) = 1 / r_max;
        
        if ((max_cuv_tail < seg_time_k_tail) && (max_cuv_head > seg_time_k_head))
        {
            Eigen::ArrayXd max_cuv_ts_temp(k_cuv.size() + 2);
            Eigen::ArrayXi max_cuv_k_temp(k_cuv.size() + 2);
            Eigen::ArrayXd max_cuv_r_temp(k_cuv.size() + 2);
            Eigen::ArrayXd max_cuv_l_temp(k_cuv.size() + 2);
            max_cuv_ts_temp << 0, ts_cuv, ts(ts.size() - 1); 
            max_cuv_k_temp << seg_time_k_head, k_cuv, seg_time_k_tail;
            max_cuv_r_temp << 1 / r_max, route_r, 1 / r_max;
            max_cuv_l_temp << 1 / r_max, route_l, 1 / r_max;
            max_cuv_ts = max_cuv_ts_temp;
            max_cuv_k = max_cuv_k_temp;
            max_cuv_r = max_cuv_r_temp;
            max_cuv_l = max_cuv_l_temp;
        }
        else if (max_cuv_tail < seg_time_k_tail)
        {
            Eigen::ArrayXd max_cuv_ts_temp(k_cuv.size() + 1);
            Eigen::ArrayXi max_cuv_k_temp(k_cuv.size() + 1);
            Eigen::ArrayXd max_cuv_r_temp(k_cuv.size() + 1);
            Eigen::ArrayXd max_cuv_l_temp(k_cuv.size() + 1);
            max_cuv_ts_temp << ts_cuv, ts(ts.size() - 1);
            max_cuv_k_temp << k_cuv, seg_time_k_tail;
            max_cuv_r_temp << route_r, 1 / r_max;
            max_cuv_l_temp << route_l, 1 / r_max;
            max_cuv_ts = max_cuv_ts_temp;
            max_cuv_k = max_cuv_k_temp;
            max_cuv_r = max_cuv_r_temp;
            max_cuv_l = max_cuv_l_temp;
        }
        else if (max_cuv_head > seg_time_k_head) 
        {
            Eigen::ArrayXd max_cuv_ts_temp(k_cuv.size() + 1);
            Eigen::ArrayXi max_cuv_k_temp(k_cuv.size() + 1);
            Eigen::ArrayXd max_cuv_r_temp(k_cuv.size() + 1);
            Eigen::ArrayXd max_cuv_l_temp(k_cuv.size() + 1);
            max_cuv_ts_temp << 0, ts_cuv;
            max_cuv_k_temp << seg_time_k_head, k_cuv;
            max_cuv_r_temp << 1 / r_max, route_r;
            max_cuv_l_temp << 1 / r_max, route_l;
            max_cuv_ts = max_cuv_ts_temp;
            max_cuv_k = max_cuv_k_temp;
            max_cuv_r = max_cuv_r_temp;
            max_cuv_l = max_cuv_l_temp;
        }
        else
        {
            Eigen::ArrayXd max_cuv_ts_temp(k_cuv.size());
            Eigen::ArrayXi max_cuv_k_temp(k_cuv.size());
            Eigen::ArrayXd max_cuv_r_temp(k_cuv.size());
            Eigen::ArrayXd max_cuv_l_temp(k_cuv.size());
            max_cuv_ts_temp << ts_cuv;
            max_cuv_k_temp << k_cuv;
            max_cuv_r_temp << route_r;
            max_cuv_l_temp << route_l;
            max_cuv_ts = max_cuv_ts_temp;
            max_cuv_k = max_cuv_k_temp;
            max_cuv_r = max_cuv_r_temp;
            max_cuv_l = max_cuv_l_temp;
        }
        radiusLeft.max_cuv = max_cuv_l;
        radiusRight.max_cuv = max_cuv_r;

        // std::cout << "记录曲率最大处对应时刻 " << std::endl
        //           << ts_cuv << std::endl;
        // std::cout << "记录曲率最大处对应时刻补全 " << std::endl
        //           << max_cuv_ts << std::endl;
        // // std::cout << "记录曲率最大处下标 " << std::endl
        //           << k_cuv << std::endl;
        // std::cout << "记录曲率最大处下标补全" << std::endl
        //           << max_cuv_k << std::endl;
        // std::cout << "记录曲率最大处曲率大小r " << std::endl
        //           << route_r << std::endl;
        // std::cout << "记录曲率最大处曲率大小r补全 " << std::endl
        //           << max_cuv_r << std::endl;          
        // std::cout << "记录曲率最大处曲率大小l " << std::endl
        //           << route_l << std::endl;
        // std::cout << "记录曲率最大处曲率大小l补全" << std::endl
        //           << max_cuv_l << std::endl;

        // Gnuplot g1("lines");
        // std::vector<double> x_1, y_1;
        // for (int i = 0; i < seg_time_k.size(); i++)
        // {
        //     x_1.push_back(ff(0, seg_time_k(i)));
        //     y_1.push_back(ff(1, seg_time_k(i)));
        // }
        // g1.set_style("points").plot_xy(x_1, y_1, "new segment time"); 
        // std::vector<double> x_2, y_2;
        // for (int i = 0; i < max_cuv_k.size(); i++)
        // {
        //     x_2.push_back(ff(0, max_cuv_k(i)));
        //     y_2.push_back(ff(1, max_cuv_k(i)));

        //     std::vector<double> x_3, y_3;
        //     x_3.push_back(ff(0, max_cuv_k(i)) + 1 / max_cuv_r(i) * sign_normal(0, max_cuv_k(i)));
        //     y_3.push_back(ff(1, max_cuv_k(i)) + 1 / max_cuv_r(i) * sign_normal(1, max_cuv_k(i)));

        //     x_3.push_back(ff(0, max_cuv_k(i)));
        //     y_3.push_back(ff(1, max_cuv_k(i)));

        //     g1.set_style("lines").plot_xy(x_3, y_3);

        //     std::vector<double> x_4, y_4;
        //     x_4.push_back(ff(0, max_cuv_k(i)) - 1 / max_cuv_l(i) * sign_normal(0, max_cuv_k(i)));
        //     y_4.push_back(ff(1, max_cuv_k(i)) - 1 / max_cuv_l(i) * sign_normal(1, max_cuv_k(i)));

        //     x_4.push_back(ff(0, max_cuv_k(i)));
        //     y_4.push_back(ff(1, max_cuv_k(i)));

        //     g1.set_style("lines").plot_xy(x_4, y_4);


        // }
        // g1.set_style("points").plot_xy(x_2, y_2, "max cuv point"); 

        // wait_for_key();

        nav_msgs::Path cuv_r;
        for (int i = 0; i < max_cuv_k.size(); i++)
        {
            geometry_msgs::PoseStamped cuv_r_stamped;
            cuv_r_stamped.pose.position.x = ff(0, max_cuv_k(i));
            cuv_r_stamped.pose.position.y = ff(1, max_cuv_k(i));
            cuv_r_stamped.pose.position.z = 1.8;
            cuv_r.poses.push_back(cuv_r_stamped);
            cuv_r_stamped.pose.position.x = ff(0, max_cuv_k(i)) + 1 / max_cuv_r(i) * sign_normal(0, max_cuv_k(i));
            cuv_r_stamped.pose.position.y = ff(1, max_cuv_k(i)) + 1 / max_cuv_r(i) * sign_normal(1, max_cuv_k(i));
            cuv_r_stamped.pose.position.z = 1.8;
            cuv_r.poses.push_back(cuv_r_stamped);

            cuv_r_stamped.pose.position.x = ff(0, max_cuv_k(i));
            cuv_r_stamped.pose.position.y = ff(1, max_cuv_k(i));
            cuv_r_stamped.pose.position.z = 1.8;
            cuv_r.poses.push_back(cuv_r_stamped);
            cuv_r_stamped.pose.position.x = ff(0, max_cuv_k(i)) - 1 / max_cuv_l(i) * sign_normal(0, max_cuv_k(i));
            cuv_r_stamped.pose.position.y = ff(1, max_cuv_k(i)) - 1 / max_cuv_l(i) * sign_normal(1, max_cuv_k(i));
            cuv_r_stamped.pose.position.z = 1.8;
            cuv_r.poses.push_back(cuv_r_stamped);
        }
        cuv_right_pub.publish(cuv_r);


        

        radiusRight.min_dis_mat = Eigen::MatrixXd::Zero(max_cuv_k.size() - 1, 3);
        radiusLeft.min_dis_mat = Eigen::MatrixXd::Zero(max_cuv_k.size() - 1, 3);
        if (gridmap.data.size() > 0)
        {
            findObstacle(radiusRight.min_dis_mat, max_cuv_k, tt, ff, sign_normal);
            findObstacle(radiusLeft.min_dis_mat, max_cuv_k, tt, ff, -1 * sign_normal);
            // std::cout << "radiusRight.min_dis_mat" << std::endl;
            // std::cout << radiusRight.min_dis_mat << std::endl;
            // std::cout << "radiusLeft.min_dis_mat" << std::endl;
            // std::cout << radiusLeft.min_dis_mat << std::endl;
            // wait_for_key();

            nav_msgs::Path max_dis;
            for (int i = 0; i < radiusLeft.min_dis_mat.rows(); i++)
            {
                int index = radiusLeft.min_dis_mat(i, 0);
                double radius = radiusLeft.min_dis_mat(i, 1);
                geometry_msgs::PoseStamped max_dis_temp;
                max_dis_temp.pose.position.x = ff(0, index);
                max_dis_temp.pose.position.y = ff(1, index);
                max_dis_temp.pose.position.z = 1.8;
                max_dis.poses.push_back(max_dis_temp);

                max_dis_temp.pose.position.x = ff(0, index) - radius * sign_normal(0, index);
                max_dis_temp.pose.position.y = ff(1, index) - radius * sign_normal(1, index);
                max_dis_temp.pose.position.z = 1.8;
                max_dis.poses.push_back(max_dis_temp);
            }
            for (int i = 0; i < radiusRight.min_dis_mat.rows(); i++)
            {
                int index = radiusRight.min_dis_mat(i, 0);
                double radius = radiusRight.min_dis_mat(i, 1);
                geometry_msgs::PoseStamped max_dis_temp;
                max_dis_temp.pose.position.x = ff(0, index);
                max_dis_temp.pose.position.y = ff(1, index);
                max_dis_temp.pose.position.z = 1.8;
                max_dis.poses.push_back(max_dis_temp);

                max_dis_temp.pose.position.x = ff(0, index) + radius * sign_normal(0, index);
                max_dis_temp.pose.position.y = ff(1, index) + radius * sign_normal(1, index);
                max_dis_temp.pose.position.z = 1.8;
                max_dis.poses.push_back(max_dis_temp);
            }

            dis_right_pub.publish(max_dis);


        }
        

    }

    void GeneratorFist::findObstacle(Eigen::MatrixXd& min_dis_k, const Eigen::ArrayXi& seg_time_k, const Eigen::ArrayXd& tt, const Eigen::MatrixXd& ff, const Eigen::MatrixXd& sign_normal)
    {
        for (int k = 0; k < seg_time_k.size() - 1; k++) // 遍历所有段
        {
            double mindis = r_max;
            min_dis_k(k, 0) = seg_time_k(k);     //最大距离对应下标
            min_dis_k(k, 1) = r_max;             //最大距离
            min_dis_k(k, 2) = tt(seg_time_k(k)); //最大距离对应时刻
            for (int k1 = seg_time_k(k); k1 < seg_time_k(k + 1); k1++)
            {
                /* 计算倾斜角 */
                double xp = ff(0, k1) - gridmap.position_x;
                double yp = ff(1, k1) - gridmap.position_y;
                double theta = atan2(sign_normal(1, k1), sign_normal(0, k1));
                // std::cout << "ff(0, k1) " << ff(0, k1) << std::endl;
                // std::cout << "xp " << xp << std::endl;
                // std::cout << "x " << gridmap.position_x << std::endl;
                // std::cout << "ff(1, k1) " << ff(1, k1) << std::endl;
                // std::cout << "yp " << yp << std::endl;
                // std::cout << "y " << gridmap.position_y << std::endl;
                /* 寻找障碍物 */
                int k2;
                for (k2 = 0; k2 < r_max / gridmap.resolution; k2++)
                {
                    int bw_x = std::min(std::max(static_cast<int>(k2 * cos(theta) + xp / gridmap.resolution), 0), gridmap.width - 1);
                    int bw_y = std::min(std::max(static_cast<int>(k2 * sin(theta) + yp / gridmap.resolution), 0), gridmap.height - 1);
                    // std::cout << gridmap.width - 1 << std::endl;
                    // std::cout << gridmap.data.size() << std::endl;
                    
                    if (gridmap.data(bw_y, bw_x) > 50)
                    {
                        // std::cout << "bw_x: " << bw_x << std::endl;
                        // std::cout << "bw_y: " << bw_y << std::endl;
                        // std::cout << "find the obstacle: " << k2 * gridmap.resolution << std::endl;
                        // std::cout << "index: " << k1 << std::endl;
                        break;
                    }
                }
                double mindis_tmp = k2 * gridmap.resolution;
                if (mindis_tmp < mindis)
                {
                    min_dis_k(k, 0) = k1;     //最大距离对应下标
                    min_dis_k(k, 1) = std::max(mindis_tmp, 0.1) - gridmap.resolution;             //最大距离
                    min_dis_k(k, 2) = tt(k1); //最大距离对应时刻
                    mindis = mindis_tmp;
                }
            } 

        }
    }

    void GeneratorFist::findMaxCurvature(Eigen::ArrayXi &seg_time, Eigen::ArrayXd &ts, Eigen::ArrayXi &max_cuv_k, Eigen::ArrayXd &route_r, Eigen::ArrayXd &route_l, const Eigen::ArrayXd &flag_dir, const Eigen::MatrixXd &ff1, const Eigen::MatrixXd &ff2, const Eigen::ArrayXd &tt)
    {
        /* 查询弯曲方向转折处 */
        std::vector<int> ts_k;
        ts_k.push_back(0);
        int count = 0;
        for (int k = 1; k < flag_dir.size() - 1; k++)
        {
            if (flag_dir(k) * flag_dir(k + 1) < 0)
            {
                ts_k.push_back(k);
                count++;
            }
        }
        if (ts_k[count - 1] != flag_dir.size() - 1)
        {
            ts_k.push_back(flag_dir.size() - 1);
        }
        // seg_time << ts_k;
        seg_time = Eigen::ArrayXi::Zero(ts_k.size());
        for (int i = 0; i < ts_k.size(); i++)
        {
            seg_time(i) = ts_k[i];
        }
        // std::cout << ts_k.size() << std::endl;

        max_cuv_k = Eigen::ArrayXi::Zero(ts_k.size() - 1); //记录曲率最大处下标
        route_r = Eigen::ArrayXd::Zero(ts_k.size() - 1);
        route_l = Eigen::ArrayXd::Zero(ts_k.size() - 1); //记录曲率最大处曲率大小
        ts = Eigen::ArrayXd::Zero(ts_k.size() - 1);      //对应时刻
        for (int k = 0; k < ts_k.size() - 1; k++)
        {
            double curvature = 1 / r_max;
            int tmp_k1 = ts_k[k];
            /* 在分段中找到曲率最大的地方的k和曲率大小 */
            for (int k1 = ts_k[k]; k1 < ts_k[k + 1]; k1++)
            {
                Eigen::Matrix2d gamma_cross;
                gamma_cross << ff2.block(0, k1, 2, 1), ff1.block(0, k1, 2, 1);
                double tmp = gamma_cross.norm() / (ff1.block(0, k1, 2, 1).norm() * ff1.block(0, k1, 2, 1).norm() * ff1.block(0, k1, 2, 1).norm());
                if (tmp > curvature)
                {
                    curvature = tmp;
                    tmp_k1 = k1;
                }
            }

            max_cuv_k(k) = tmp_k1;
            ts(k) = tt(tmp_k1);
            if (flag_dir(tmp_k1) > 0)
            {
                route_r(k) = curvature;
                route_l(k) = 1 / r_max;
            }
            else
            {
                route_l(k) = curvature;
                route_r(k) = 1 / r_max;
            }
        }
    }

    void GeneratorFist::polys_val(const Eigen::MatrixXd &Poly, Eigen::ArrayXd &vals, const Eigen::ArrayXd &tt, const int r)
    {

        int idx = 0;
        int N = tt.size();
        vals = Eigen::ArrayXd::Zero(N);
        for (int i = 0; i < N; i++)
        {
            double t = tt(i);
            if (t < ts(idx))
            {
                vals(i) = 0;
            }
            else
            {
                while (idx < ts.size() - 1 && t > ts(idx + 1) + 0.0001)
                {
                    idx = idx + 1;
                }
                vals(i) = poly_val(Poly.block(0, idx, n_coef, 1), t, r);
            }
        }
        // std::cout << "vals" << std::endl << vals << std::endl;
        // std::cout << "idx" << std::endl << Poly.block(0, idx, n_coef, 1) << std::endl;
    }

    double GeneratorFist::poly_val(const Eigen::MatrixXd &poly, const double t, const int r)
    {
        double val = 0;
        int n = poly.size() - 1;
        if (r <= 0)
        {
            for (int i = 0; i <= n; i++)
            {
                val = val + poly(i) * pow(t, i);
            }
        }
        else
        {
            for (int i = r; i <= n; i++)
            {
                double a = poly(i) * std::tgamma(i + 1) / std::tgamma(i - r + 1) * pow(t, i - r);
                val = val + a;
            }
        }
        return val;
    }

    void GeneratorFist::wait_for_key()
    {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) // every keypress registered, also arrow keys
        cout << endl
             << "Press any key to continue..." << endl;

        FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
        _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
        std::cout << std::endl
                  << "Press ENTER to continue..." << std::endl;

        std::cin.clear();
        std::cin.ignore(std::cin.rdbuf()->in_avail());
        std::cin.get();
#endif
    }

}
