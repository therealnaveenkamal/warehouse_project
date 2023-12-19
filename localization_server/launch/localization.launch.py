import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    nav2_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    nav2_yaml_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')
    map_file = 'warehouse_map_sim.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Name of the Mapfile (without path)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rb1_ros2_description'), 'launch', 'rb1_ros2_xacro.launch.py')])
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('localization_server'), 'rviz_config', 'finalv1.rviz'])],
        ),
    
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}, 
                        {'yaml_filename': PathJoinSubstitution([FindPackageShare('map_server'), 'config', LaunchConfiguration('map_file')])}
                    ]
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml_sim],
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml_real],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                                {'autostart': True},
                                {'node_names': ['map_server', 'amcl']}]
                ),
                Node(
                    package='localization_server',
                    executable='initial_pose_publisher_cpp',
                    output='screen'
                ),
            ],
        )
    ])
