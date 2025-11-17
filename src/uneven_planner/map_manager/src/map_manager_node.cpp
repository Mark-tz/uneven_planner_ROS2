#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lightning/srv/save_map.hpp"
#include "uneven_planner_interfaces/srv/reload_map.hpp" // We will create this service in a new interface package

#include <memory>
#include <string>
#include <cmath>

class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode()
    : Node("map_manager_node"), last_pos_x_(0.0), last_pos_y_(0.0), last_yaw_(0.0)
    {
        this->declare_parameter<double>("update_distance_thresh", 5.0);
        this->declare_parameter<double>("update_angle_thresh", 0.52); // 30 degrees in radians
        this->declare_parameter<std::string>("map_save_path", "/tmp/current_map.pcd");

        this->get_parameter("update_distance_thresh", dist_thresh_);
        this->get_parameter("update_angle_thresh", angle_thresh_);
        this->get_parameter("map_save_path", map_save_path_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapManagerNode::odomCallback, this, std::placeholders::_1));

        save_map_client_ = this->create_client<lightning::srv::SaveMap>("/lightning/save_map");
        reload_map_client_ = this->create_client<uneven_planner_interfaces::srv::ReloadMap>("/uneven_planner/reload_map");

        RCLCPP_INFO(this->get_logger(), "Map Manager Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Update distance threshold: %.2f m", dist_thresh_);
        RCLCPP_INFO(this->get_logger(), "Update angle threshold: %.2f rad", angle_thresh_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_pos_x = msg->pose.pose.position.x;
        double current_pos_y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double current_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        double dist_moved = std::sqrt(std::pow(current_pos_x - last_pos_x_, 2) + std::pow(current_pos_y - last_pos_y_, 2));
        double angle_rotated = std::abs(current_yaw - last_yaw_);
        
        // Normalize angle difference to be within [0, PI]
        if (angle_rotated > M_PI) {
            angle_rotated = 2 * M_PI - angle_rotated;
        }

        if (dist_moved > dist_thresh_ || angle_rotated > angle_thresh_)
        {
            RCLCPP_INFO(this->get_logger(), "Threshold triggered. Requesting map save...");
            last_pos_x_ = current_pos_x;
            last_pos_y_ = current_pos_y;
            last_yaw_ = current_yaw;

            if (!save_map_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_ERROR(this->get_logger(), "Service /lightning/save_map not available.");
                return;
            }

            auto request = std::make_shared<lightning::srv::SaveMap::Request>();
            request->map_id = map_save_path_; // Use map_id instead of map_path

            auto result_future = save_map_client_->async_send_request(request, 
                std::bind(&MapManagerNode::saveMapResponseCallback, this, std::placeholders::_1));
        }
    }

    void saveMapResponseCallback(rclcpp::Client<lightning::srv::SaveMap>::SharedFuture future)
    {
        auto result = future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Map saved successfully to %s. Triggering reload.", map_save_path_.c_str());
            
            if (!reload_map_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_ERROR(this->get_logger(), "Service /uneven_planner/reload_map not available.");
                return;
            }

            auto request = std::make_shared<uneven_planner_interfaces::srv::ReloadMap::Request>();
            request->map_path = map_save_path_;
            reload_map_client_->async_send_request(request); // Fire and forget

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map.");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Client<lightning::srv::SaveMap>::SharedPtr save_map_client_;
    rclcpp::Client<uneven_planner_interfaces::srv::ReloadMap>::SharedPtr reload_map_client_;
    
    double last_pos_x_, last_pos_y_, last_yaw_;
    double dist_thresh_, angle_thresh_;
    std::string map_save_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapManagerNode>());
    rclcpp::shutdown();
    return 0;
}