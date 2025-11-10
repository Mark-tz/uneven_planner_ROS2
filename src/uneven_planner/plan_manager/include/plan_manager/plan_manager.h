// class PlanManager
#pragma once

#include <fstream>
#include <string.h>
#include <random>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "uneven_map/uneven_map.h"
#include "front_end/kino_astar.h"
#include "back_end/alm_traj_opt.h"
#include "mpc_controller/msg/se2_traj.hpp"

namespace uneven_planner
{
    class PlanManager
    {
        private:
            bool in_plan = false;
            double piece_len;
            double mean_vel;
            double init_time_times;
            double yaw_piece_times;
            double init_sig_vel;
            Eigen::Vector3d odom_pos;
            std::string bk_dir;

            UnevenMap::Ptr uneven_map;
            KinoAstar::Ptr kino_astar;
            ALMTrajOpt traj_opt;
            SE2Trajectory opted_traj;

            rclcpp::Publisher<mpc_controller::msg::SE2Traj>::SharedPtr traj_pub;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub;
            std::shared_ptr<rclcpp::Node> node_;
        public:
            void init(std::shared_ptr<rclcpp::Node> node);
            void rcvOdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
            void rcvWpsCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    };
}
