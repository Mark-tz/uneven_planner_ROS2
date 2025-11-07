// function main()
#include "uneven_map/uneven_map.h"
#include <rclcpp/rclcpp.hpp>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("uneven_map_node");

  uneven_planner::UnevenMap uneven_map;
  uneven_map.init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}