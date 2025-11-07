// function main()
#include "plan_manager/plan_manager.h"
#include <rclcpp/rclcpp.hpp>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("manager_node");

    uneven_planner::PlanManager plan_manager;
    plan_manager.init(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}