from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_topic  = LaunchConfiguration('cmd_topic')
    traj_topic = LaunchConfiguration('traj_topic')
    goal_topic = LaunchConfiguration('goal_topic')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_topic',  default_value='/racebot/cmd_vel'),
        DeclareLaunchArgument('goal_topic',  default_value='/racebot/goal'),
        DeclareLaunchArgument('traj_topic', default_value='/ugv/trajectory'),

        # Map Manager Node
        Node(
            package='map_manager',
            executable='map_manager_node',
            name='map_manager_node',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='plan_manager',
            executable='manager_node',
            name='manager_node',
            parameters=[
                {'use_sim_time': False},
                join(get_package_share_directory('plan_manager'), 'params', 'run_custom.yaml')
            ],
            remappings=[
                ('cmd',  cmd_topic),
                ('odom', odom_topic),
                ('traj', traj_topic),
                ('goal', goal_topic),
            ],
        ),
        Node(
            package='mpc_controller',
            executable='mpc_controller_node',
            name='mpc_controller_node',
            parameters=[
                {'use_sim_time': False},
                join(get_package_share_directory('mpc_controller'), 'params', 'controller_custom.yaml')
            ],
            remappings=[
                ('cmd',  cmd_topic),
                ('odom', odom_topic),
                ('traj', traj_topic),
            ],
        ),
        
        # # Include Gazebo simulation
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         join(get_package_share_directory('carsim'), 'launch', 'spawn_car.launch.py')
        #     ]),
        #     launch_arguments={
        #         'use_rviz': 'false',
        #         'gui': 'true',
        #         'map_name': map_name,
        #         'x_pos': x_pos,
        #         'y_pos': y_pos,
        #         'z_pos': z_pos,
        #         'Y_pos': Y_pos,
        #     }.items()
        # ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'src/uneven_planner/src/uneven_planner/plan_manager/rviz/rviz_custom.rviz'],
            output='screen'
        ),
    ])