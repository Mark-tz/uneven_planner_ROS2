# ROS1 → ROS2 迁移总体方案（Uneven Planner）

目标与假设
- 目标：迁移到 ROS2 Humble（Ubuntu 22.04），使用 `colcon + ament_cmake`。
- 范围：carsim、uneven_map、front_end、back_end、plan_manager、mpc_controller。
- 约束：尽量保持话题、参数命名与接口一致；Gazebo 采用 ROS2 gazebo_ros_pkgs（Classic）。

步骤总览
1. 构建系统切换到 `colcon/ament_cmake`
   - 将各包的 `CMakeLists.txt`/`package.xml` 迁移为 ROS2 版本。
   - 构建命令改用 `colcon build --symlink-install`，运行时 `source install/setup.bash`。
2. 自定义消息（`mpc_controller/SE2Traj.msg`）迁移
   - 使用 `rosidl_default_generators` 生成接口，消费者使用 `rosidl_default_runtime`。
   - 代码中引用类型改为 `mpc_controller::msg::SE2Traj`。
3. C++ 节点 API 替换
   - `ros::init` → `rclcpp::init`，`ros::NodeHandle` → `rclcpp::Node`。
   - 发布/订阅/定时器：`advertise/subscribe/createTimer` → `create_publisher/create_subscription/create_wall_timer`。
   - 参数：`getParam/param` → `declare_parameter/get_parameter`。
   - 日志：`ROS_INFO` → `RCLCPP_INFO`。
4. QoS 策略
   - 传感器/里程计类话题使用 `rclcpp::SensorDataQoS()`，其他使用默认可靠 QoS。
5. 启动文件从 XML 转 Python
   - `<node>` → `launch_ros.actions.Node`；`rosparam load` → `parameters=[param_yaml]`。
   - RViz 使用 `rviz2`，Gazebo 用 ROS2 `gazebo_ros_pkgs`。
6. 参数 YAML 改 ROS2 格式
   - 结构：`<node_name> -> ros__parameters -> 键值`，键可沿用 `ns/key` 或改为点分（如 `ns.key`）。
7. Gazebo 适配（carsim）
   - `gazebo_msgs/SetModelState` → ROS2 `gazebo_msgs::srv::SetEntityState`。
   - 模型路径通过 `GAZEBO_MODEL_PATH` 指向包 `share/carsim/models`。
8. 验证顺序（分阶段构建测试）
   - 依次迁移并测试：`uneven_map` → `front_end` → `back_end` → `mpc_controller` → `plan_manager` → `carsim`。
   - 每阶段 `colcon build --packages-select <pkg>` 并 `ros2 launch`最小场景。
9. 本轮执行范围
   - 按优先级迁移并给出具体改动：`mpc_controller`、`plan_manager`、`uneven_map`。
   - 后续轮次覆盖：`front_end`、`back_end`、`carsim` 与所有启动/参数文件全面迁移。