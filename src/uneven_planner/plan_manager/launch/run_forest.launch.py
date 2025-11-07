from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_topic  = LaunchConfiguration('cmd_topic')
    traj_topic = LaunchConfiguration('traj_topic')
    map_name   = LaunchConfiguration('map_name')
    x_pos      = LaunchConfiguration('x_pos')
    y_pos      = LaunchConfiguration('y_pos')
    z_pos      = LaunchConfiguration('z_pos')
    Y_pos      = LaunchConfiguration('Y_pos')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/racebot/true_state/odom'),
        DeclareLaunchArgument('cmd_topic',  default_value='/racebot/cmd_vel'),
        DeclareLaunchArgument('traj_topic', default_value='/ugv/trajectory'),
        DeclareLaunchArgument('map_name',   default_value='forest'),
        DeclareLaunchArgument('x_pos',      default_value='-4.5'),
        DeclareLaunchArgument('y_pos',      default_value='-4.5'),
        DeclareLaunchArgument('z_pos',      default_value='2.0'),
        DeclareLaunchArgument('Y_pos',      default_value='0.0'),

        Node(
            package='plan_manager',
            executable='manager_node',
            name='manager_node',
            parameters=[ join(get_package_share_directory('plan_manager'), 'params', 'run_forest.yaml') ],
            remappings=[
                ('cmd',  cmd_topic),
                ('odom', odom_topic),
                ('traj', traj_topic),
            ],
        ),
        Node(
            package='mpc_controller',
            executable='mpc_controller_node',
            name='mpc_controller_node',
            remappings=[
                ('cmd',  cmd_topic),
                ('odom', odom_topic),
                ('traj', traj_topic),
            ],
            parameters=[ join(get_package_share_directory('mpc_controller'), 'params', 'controller.yaml') ],
        ),
        
        # Include Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                join(get_package_share_directory('carsim'), 'launch', 'spawn_car.launch.py')
            ]),
            launch_arguments={
                'use_rviz': 'false',
                'gui': 'true',
                'map_name': map_name,
                'x_pos': x_pos,
                'y_pos': y_pos,
                'z_pos': z_pos,
                'Y_pos': Y_pos,
            }.items()
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', join(get_package_share_directory('plan_manager'), 'rviz', 'default.rviz')],
            output='screen'
        ),
    ])