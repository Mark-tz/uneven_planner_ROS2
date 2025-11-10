from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():
    # Get package directories
    carsim_pkg = FindPackageShare('carsim')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    
    # Set Gazebo environment variables
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [PathJoinSubstitution([carsim_pkg, 'models']), ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )
    
    gazebo_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        [PathJoinSubstitution([carsim_pkg, '..', '..', 'lib']), ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')]
    )
    
    # Declare arguments
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false')  # 关闭详细输出
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='desert')
    
    # Car position arguments
    x_pos_arg = DeclareLaunchArgument('x_pos', default_value='0.0')
    y_pos_arg = DeclareLaunchArgument('y_pos', default_value='0.0')
    z_pos_arg = DeclareLaunchArgument('z_pos', default_value='10.0')
    R_pos_arg = DeclareLaunchArgument('R_pos', default_value='0')
    P_pos_arg = DeclareLaunchArgument('P_pos', default_value='0')
    Y_pos_arg = DeclareLaunchArgument('Y_pos', default_value='0')

    # Robot description
    robot_description = Command([
        'xacro ', 
        PathJoinSubstitution([carsim_pkg, 'urdf', 'car_model_description.xacro'])
    ])

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'verbose': 'false',
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': PathJoinSubstitution([
                carsim_pkg, 'worlds', 
                ['map_', LaunchConfiguration('map_name'), '.world']
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='/racebot',
        parameters=[{'robot_description': robot_description}],
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        namespace='/racebot',
        arguments=[
            '-entity', 'racebot',
            '-topic', '/racebot/robot_description',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-R', LaunchConfiguration('R_pos'),
            '-P', LaunchConfiguration('P_pos'),
            '-Y', LaunchConfiguration('Y_pos'),
            '--ros-args', '--log-level', 'warn'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='log'
    )

    # 工具节点（与ROS1等价）
    cmdvel2gazebo_node = Node(
        package='carsim',
        executable='cmdvel2gazebo.py',
        name='cmdvel2gazebo',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # keyboard_control_node = Node(
    #     package='carsim',
    #     executable='keyboard_control.py',
    #     name='keyboard_control',
    #     output='log',
    #     arguments=['--ros-args', '--log-level', 'warn']
    # )

    true_state_pub_node = Node(
        package='carsim',
        executable='true_state_pub.py',
        name='true_state_pub',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    world_tf_pub_node = Node(
        package='carsim',
        executable='world_tf_pub.py',
        name='world_tf_pub',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ROS2控制器spawner（对应ROS1的controller_manager/spawner）
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    rr_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_right_rear_velocity_controller',
        arguments=['right_rear_velocity_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    lr_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_left_rear_velocity_controller',
        arguments=['left_rear_velocity_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    rf_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_right_front_velocity_controller',
        arguments=['right_front_velocity_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    lf_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_left_front_velocity_controller',
        arguments=['left_front_velocity_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    rf_pos_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_right_front_steering_position_controller',
        arguments=['right_front_steering_position_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )
    lf_pos_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_left_front_steering_position_controller',
        arguments=['left_front_steering_position_controller', '--controller-manager', '/racebot/controller_manager'],
        output='log'
    )

    # 将车辆定位到地图中的设定点（ROS2版本C++节点）
    search_for_setpoint_node = Node(
        package='carsim',
        executable='searchForSetPoint',
        name='searchForSetPoint',
        parameters=[{
            'map_file': PathJoinSubstitution([
                FindPackageShare('uneven_map'), 'maps',
                [LaunchConfiguration('map_name'), '.pcd']
            ])
        }],
        output='log',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        # Environment variables
        gazebo_model_path,
        gazebo_plugin_path,
        
        # Launch arguments
        use_rviz_arg,
        paused_arg,
        verbose_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        map_name_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        R_pos_arg,
        P_pos_arg,
        Y_pos_arg,
        
        # Main processes
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        # 工具节点
        cmdvel2gazebo_node, 
        # keyboard_control_node, 
        true_state_pub_node,
        world_tf_pub_node, 
        search_for_setpoint_node,
        joint_state_spawner, rr_vel_spawner, lr_vel_spawner,
        rf_vel_spawner, lf_vel_spawner, rf_pos_spawner, lf_pos_spawner,
        # RViz (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])