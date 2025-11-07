// class MPC
#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
#include <numeric>
 
#include <OsqpEigen/OsqpEigen.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "mpc_controller/msg/se2_traj.hpp"

#include "utils/traj_anal.hpp"

#define OMINI 0
#define DIFF  1
#define ACKER 2

using namespace std;
using namespace Eigen;

class MPCState
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;
};

class MPCInput
{
public:
    double vx = 0;
    double vy = 0;
    double w = 0;
    double delta = 0;
};

typedef pair<MPCState, MPCInput> MPCNode;

class MPC
{
private:
    // parameters
    /// algorithm param
    int model_type = OMINI;
    double du_th = 0.1;
    double dt = 0.2;
    int T = 5;
    int max_iter = 3;
    int delay_num;
    vector<double> Q = {10, 10, 0.5};
    vector<double> R = {0.01, 0.01};
    vector<double> Rd = {0.01, 1.0};
    /// constraints
    double max_omega = M_PI / 4;
    double max_domega = M_PI / 6;
    double max_comega = M_PI / 6 * 0.2;
    double max_steer = M_PI / 4;
    double max_dsteer = M_PI / 6;
    double max_csteer = M_PI / 6 * 0.2;
    double max_speed = 55.0 / 3.6;
    double min_speed = -55.0 / 3.6;
    double max_cv = 0.2;
    double max_accel = 1.0; 
    double wheel_base = 0.5; 

    // MPC dataset
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;
    MPCNode xbar[500];
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;
    std::vector<Eigen::Vector2d> output_buff;

    // control data
    bool has_odom;
    bool receive_traj = false;
    double tolerance = 0.1;
    double traj_duration_;
    double t_track = 0.0;
    MPCState now_state;
    MPCInput now_input;
    TrajAnalyzer traj_analyzer;

    // ros interface (ROS2)
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pos_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub, predict_pub, ref_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<mpc_controller::msg::SE2Traj>::SharedPtr traj_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
    geometry_msgs::msg::Twist cmd;
    void cmdCallback(); // ROS2: 无参版本
    void rcvOdomCallBack(nav_msgs::msg::Odometry::SharedPtr msg);
    void rcvTrajCallBack(mpc_controller::msg::SE2Traj::SharedPtr msg);
    void rcvTriggerCallBack(const geometry_msgs::msg::PoseStamped msg);

    // for test tracking performance
    bool test_mpc;
    vector<TrajPoint> eight_path;

    // for benchmark
    bool bk_mode;
    double mean_err_all = 0.0;
    std::vector<double> errs;
    string traj_file;
    std::ofstream outfile;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr gazebo_pub;

    // MPC function
    void getLinearModel(const MPCNode& node);
    void stateTrans(MPCNode& node);
    void predictMotion(void);
    void predictMotion(MPCState *b);
    void solveMPCDiff(void);
    void solveMPCAcker(void);
    void getCmd(void);

    // utils
    MPCState xopt[500];
    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }
    void smooth_yaw(void)
    {
        double dyaw = xref(2, 0) - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            xref(2, 0) -= M_PI * 2;
            dyaw = xref(2, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            xref(2, 0) += M_PI * 2;
            dyaw = xref(2, 0) - now_state.theta;
        }

        for (int i=0; i<T-1; i++)
        {
            dyaw = xref(2, i+1) - xref(2, i);
            while (dyaw >= M_PI / 2)
            {
                xref(2, i+1) -= M_PI * 2;
                dyaw = xref(2, i+1) - xref(2, i);
            }
            while (dyaw <= -M_PI / 2)
            {
                xref(2, i+1) += M_PI * 2;
                dyaw = xref(2, i+1) - xref(2, i);
            }
        }
    }
    void drawFollowPath(void)
    {
        int id = 0;
        double sc = 0.02;
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = node_->get_clock()->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 1;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::msg::Point pt;
        
        for (auto p:eight_path)
        {
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        vis_pub->publish(line_strip);
    }
    void drawPredictPath(MPCState *b)
    {
        int id = 0;
        double sc = 0.02;
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = node_->get_clock()->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 1;
        sphere.color.b = line_strip.color.b = 0;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::msg::Point pt;
        
        for (int i=0; i<T; i++)
        {
            pt.x = b[i].x;
            pt.y = b[i].y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub->publish(line_strip);
    }
    void drawRefPath(void)
    {
        int id = 0;
        double sc = 0.02;
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = node_->get_clock()->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::msg::Point pt;
        
        for (int i=0; i<T; i++)
        {
            pt.x = xref(0, i);
            pt.y = xref(1, i);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub->publish(line_strip);
    }

public:
	MPC() {}
    void init(std::shared_ptr<rclcpp::Node> node);
	~MPC() {}
};