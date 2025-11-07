cmake_minimum_required(VERSION 3.8)
project(front_end)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uneven_map REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(front_end 
    src/kino_astar.cpp
)

# Set include directories for the library
target_include_directories(front_end
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(front_end
    rclcpp
    std_msgs
    nav_msgs
    uneven_map
    visualization_msgs
    pcl_conversions
)

target_link_libraries(front_end
    ${OMPL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(front_end_node src/front_end_node.cpp)

target_link_libraries(front_end_node
    front_end
    ${OMPL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

ament_target_dependencies(front_end_node
    rclcpp
    std_msgs
    nav_msgs
    uneven_map
    visualization_msgs
    pcl_conversions
)

install(TARGETS
    front_end
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    front_end_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    DESTINATION include
)

# Export include directories and targets for downstream packages
ament_export_include_directories(include)
ament_export_libraries(front_end)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  uneven_map
  visualization_msgs
  pcl_conversions
  ompl
)

ament_package()
