cmake_minimum_required(VERSION 3.5)
project(plan_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(ompl REQUIRED)
find_package(uneven_map REQUIRED)
find_package(front_end REQUIRED)
find_package(back_end REQUIRED)
find_package(mpc_controller REQUIRED)

include_directories(
  SYSTEM
  include
  ${OMPL_INCLUDE_DIRS}
)

add_library(plan_manager
  src/plan_manager.cpp
)
ament_target_dependencies(plan_manager
  rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs uneven_map front_end back_end mpc_controller ompl OsqpEigen
)
target_link_libraries(plan_manager
  ${OMPL_LIBRARIES}
)

add_executable(manager_node src/manager_node.cpp)
ament_target_dependencies(manager_node
  rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs uneven_map front_end back_end mpc_controller ompl OsqpEigen
)
target_link_libraries(manager_node plan_manager ${OMPL_LIBRARIES})

install(TARGETS plan_manager manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  uneven_map
  front_end
  back_end
  mpc_controller
  ompl
  OsqpEigen
)
ament_package()
