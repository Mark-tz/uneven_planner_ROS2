cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lightning REQUIRED)
find_package(uneven_planner_interfaces REQUIRED)

add_executable(map_manager_node src/map_manager_node.cpp)
target_include_directories(map_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(map_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  map_manager_node
  "rclcpp"
  "nav_msgs"
  "lightning"
  "uneven_planner_interfaces"
)

install(TARGETS
  map_manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
