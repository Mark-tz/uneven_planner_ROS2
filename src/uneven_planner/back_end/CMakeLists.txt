cmake_minimum_required(VERSION 3.8)
project(back_end)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(uneven_map REQUIRED)
find_package(front_end REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(back_end 
    src/alm_traj_opt.cpp
)

ament_target_dependencies(back_end
    rclcpp
    std_msgs
    visualization_msgs
    uneven_map
    front_end
)

target_link_libraries(back_end
    ${OMPL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(back_end_node src/back_end_node.cpp)

target_link_libraries(back_end_node
    back_end
)

ament_target_dependencies(back_end_node
    rclcpp
    std_msgs
    visualization_msgs
    uneven_map
    front_end
)

install(TARGETS
    back_end
    EXPORT back_end_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS
    back_end_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    DESTINATION include
)

install(EXPORT back_end_targets
    FILE back_end_targets.cmake
    NAMESPACE back_end::
    DESTINATION lib/cmake/back_end
)

ament_export_include_directories(include)
ament_export_libraries(back_end)
ament_export_targets(back_end_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
    visualization_msgs
    uneven_map
    front_end
    Eigen3
    ompl
    PCL
    OpenCV
)

ament_package()
