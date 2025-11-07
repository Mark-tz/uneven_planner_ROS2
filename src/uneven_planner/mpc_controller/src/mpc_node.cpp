// function main()
#include "mpc/mpc.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mpc_node");

  MPC mpc_tracker;
  mpc_tracker.init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}