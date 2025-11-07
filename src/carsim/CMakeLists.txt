cmake_minimum_required(VERSION 3.8)
project(carsim)

## Compile as C++17, supported in ROS2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(searchForSetPoint src/searchForSetPoint.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(searchForSetPoint
  ${PCL_LIBRARIES}
)

# Add ROS2 dependencies
ament_target_dependencies(searchForSetPoint
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  gazebo_msgs
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS searchForSetPoint
  DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  meshes
  models
  scripts
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

## Install Python scripts
install(PROGRAMS
  scripts/cmdvel2gazebo.py
  scripts/keyboard_control.py
  scripts/true_state_pub.py
  scripts/world_tf_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

## Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  gazebo_msgs
  PCL
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
