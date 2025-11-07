#include "front_end/kino_astar.h"
#include <rclcpp/rclcpp.hpp>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("front_end_node");

    KinoAstar kino_astar;
    UnevenMap uneven_map;
    UnevenMap::Ptr uneven_map_ptr = make_shared<UnevenMap>(uneven_map);
    
    uneven_map_ptr->init(node);
    kino_astar.init(node);
    kino_astar.setEnvironment(uneven_map_ptr);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}