cmake_minimum_required(VERSION 3.8)
project(uneven_map C CXX)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# 添加 MPI 支持以解决 VTK 依赖问题
find_package(MPI QUIET)
if(MPI_FOUND)
  message(STATUS "Found MPI")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV 4 REQUIRED)

include_directories(
  SYSTEM
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(uneven_map
  src/uneven_map.cpp
)

# Set include directories for the library
target_include_directories(uneven_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uneven_map
  rclcpp sensor_msgs std_msgs nav_msgs visualization_msgs pcl_conversions
)

add_executable(uneven_map_node src/uneven_map_node.cpp)
ament_target_dependencies(uneven_map_node
  rclcpp sensor_msgs std_msgs nav_msgs visualization_msgs pcl_conversions
)

# 链接库
target_link_libraries(uneven_map
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# 如果找到 MPI，添加 MPI 链接
if(MPI_FOUND)
  target_link_libraries(uneven_map ${MPI_LIBRARIES})
endif()

target_link_libraries(uneven_map_node
  uneven_map
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# 如果找到 MPI，添加 MPI 链接
if(MPI_FOUND)
  target_link_libraries(uneven_map_node ${MPI_LIBRARIES})
endif()

install(TARGETS uneven_map
  EXPORT uneven_map_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS uneven_map_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_export_include_directories(include)
ament_export_libraries(uneven_map)
ament_export_targets(uneven_map_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  Eigen3
  PCL
  OpenCV
)
ament_package()