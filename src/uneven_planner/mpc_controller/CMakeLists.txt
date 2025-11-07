cmake_minimum_required(VERSION 3.5)
project(mpc_controller)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OsqpEigen QUIET)

# # 包含 FetchContent 模块
# include(FetchContent)

# message(STATUS "OsqpEigen_FOUND: ${OsqpEigen_FOUND}")
# if(OsqpEigen_FOUND)
#   message(STATUS "Found OsqpEigen, using system installation")
# else()
#   message(STATUS "OsqpEigen not found, will build from source")
# endif()

# if(NOT OsqpEigen_FOUND)
#   message(STATUS "OsqpEigen not found, downloading and building from source...")
  
#   # 下载并构建 OSQP
#   FetchContent_Declare(
#     osqp
#     GIT_REPOSITORY https://github.com/osqp/osqp.git
#     GIT_TAG v0.6.2
#   )
  
#   # 设置构建选项
#   set(OSQP_BUILD_SHARED_LIB ON CACHE BOOL "Build shared library")
#   set(OSQP_BUILD_STATIC_LIB OFF CACHE BOOL "Build static library")
  
#   message(STATUS "About to build OSQP...")
  
#   # 先构建OSQP
#   FetchContent_MakeAvailable(osqp)
  
#   message(STATUS "OSQP build completed")
  
#   # 调试信息：检查OSQP构建结果
#   message(STATUS "=== OSQP Build Debug Info ===")
#   message(STATUS "OSQP source dir: ${osqp_SOURCE_DIR}")
#   message(STATUS "OSQP binary dir: ${osqp_BINARY_DIR}")
  
#   # 检查配置文件
#   if(EXISTS "${osqp_BINARY_DIR}/osqp-config.cmake")
#     message(STATUS "Found osqp-config.cmake at: ${osqp_BINARY_DIR}/osqp-config.cmake")
#   else()
#     message(STATUS "osqp-config.cmake NOT found in ${osqp_BINARY_DIR}")
#   endif()
  
#   if(EXISTS "${osqp_BINARY_DIR}/osqpConfig.cmake")
#     message(STATUS "Found osqpConfig.cmake at: ${osqp_BINARY_DIR}/osqpConfig.cmake")
#   else()
#     message(STATUS "osqpConfig.cmake NOT found in ${osqp_BINARY_DIR}")
#   endif()
  
#   # 列出binary目录的内容
#   file(GLOB osqp_binary_contents "${osqp_BINARY_DIR}/*")
#   message(STATUS "Contents of OSQP binary dir:")
#   foreach(item ${osqp_binary_contents})
#     message(STATUS "  ${item}")
#   endforeach()
  
#   # 设置osqp路径
#   set(osqp_DIR ${osqp_BINARY_DIR} CACHE PATH "Path to osqp build directory")
#   list(APPEND CMAKE_PREFIX_PATH ${osqp_BINARY_DIR})
  
#   message(STATUS "Set osqp_DIR to: ${osqp_DIR}")
#   message(STATUS "Current CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
#   message(STATUS "=== End OSQP Debug Info ===")
  
#   # 下载并构建 OsqpEigen
#   FetchContent_Declare(
#     osqp-eigen
#     GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
#     GIT_TAG v0.8.0
#   )
  
#   set(OSQP_EIGEN_USES_SYSTEM_OSQP OFF CACHE BOOL "Use system OSQP")
  
#   # 在构建osqp-eigen之前再次确认路径
#   message(STATUS "=== Before building OsqpEigen ===")
#   message(STATUS "osqp_DIR: ${osqp_DIR}")
#   message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
  
#   message(STATUS "About to build OsqpEigen...")
#   FetchContent_MakeAvailable(osqp-eigen)
  
#   message(STATUS "Successfully built OSQP and OsqpEigen from source")
# endif()

# 生成ROS2接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SE2Traj.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# 获取生成的接口目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  SYSTEM
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

# 使用不同的名称避免与rosidl_generate_interfaces创建的目标冲突
add_library(mpc_controller_lib
  src/mpc.cpp
)

# 链接 OsqpEigen 库
target_link_libraries(mpc_controller_lib
  OsqpEigen::OsqpEigen
  ${cpp_typesupport_target}
)

ament_target_dependencies(mpc_controller_lib
  rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs Eigen3
)

add_executable(mpc_controller_node
  src/mpc_node.cpp
)

# 链接库
target_link_libraries(mpc_controller_node
  mpc_controller_lib
  ${cpp_typesupport_target}
)

ament_target_dependencies(mpc_controller_node
  rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs Eigen3
)

install(TARGETS
  mpc_controller_lib
  EXPORT mpc_controller_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)

ament_export_include_directories(include)
ament_export_libraries(mpc_controller_lib)
ament_export_targets(mpc_controller_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  OsqpEigen
  rosidl_default_runtime
)
ament_package()