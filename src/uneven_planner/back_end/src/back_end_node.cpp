#include "back_end/alm_traj_opt.h"
#include <rclcpp/rclcpp.hpp>

using namespace uneven_planner;

int main( int argc, char * argv[] )
{ 
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("back_end_node");

    ALMTrajOpt traj_opt;
    KinoAstar kino_astar;
    UnevenMap uneven_map;
    UnevenMap::Ptr uneven_map_ptr = make_shared<UnevenMap>(uneven_map);
    KinoAstar::Ptr kino_astar_ptr = make_shared<KinoAstar>(kino_astar);
    
    uneven_map_ptr->init(node);
    kino_astar_ptr->init(node);
    kino_astar_ptr->setEnvironment(uneven_map_ptr);
    traj_opt.init(node);
    traj_opt.setFrontend(kino_astar_ptr);
    traj_opt.setEnvironment(uneven_map_ptr);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}