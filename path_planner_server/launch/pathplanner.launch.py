import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    controller_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real_controller.yaml')
    bt_navigator_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real_bt.yaml')
    planner_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real_planner_server.yaml')
    recovery_yaml_real = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real_recovery.yaml')
    nav2_yaml_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')
    map_file_real = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')

    controller_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_controller.yaml')
    bt_navigator_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_bt.yaml')
    planner_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_planner_server.yaml')
    recovery_yaml_sim = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_recovery.yaml')
    nav2_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    map_file_sim = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_sim.yaml')
    
    filters_sim_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters_sim.yaml')
    filters_real_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filter_real.yaml')

    waypoint_follower_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'waypoint_follower.yaml')

    return LaunchDescription([     
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('path_planner_server'), 'rviz_config', 'pathplanning.rviz'])],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': True}, 
                                {'yaml_filename':map_file_sim}],
                    condition=IfCondition(LaunchConfiguration('use_sim_time'))
                ),
                
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': False}, 
                                {'yaml_filename':map_file_real}],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),

                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='filter_mask_server',
                    output='screen',
                    emulate_tty=True,
                    parameters=[filters_sim_yaml],
                    condition=IfCondition(LaunchConfiguration('use_sim_time'))
                ),

                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='costmap_filter_info_server',
                    output='screen',
                    emulate_tty=True,
                    parameters=[filters_sim_yaml],
                    condition=IfCondition(LaunchConfiguration('use_sim_time'))),

                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='filter_mask_server',
                    output='screen',
                    emulate_tty=True,
                    parameters=[filters_real_yaml],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))),

                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='costmap_filter_info_server',
                    output='screen',
                    emulate_tty=True,
                    parameters=[filters_real_yaml],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))),
                    
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml_sim],
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                    remappings=[('/cmd_vel', '/robot/cmd_vel')]
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml_real],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),


                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml_sim],
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                    remappings=[('/cmd_vel', '/robot/cmd_vel')]
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml_real],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),


                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml_sim],
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                    remappings=[('/cmd_vel', '/robot/cmd_vel')]
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml_real],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),


                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    parameters=[recovery_yaml_sim],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                    remappings=[('/cmd_vel', '/robot/cmd_vel')]
                ),
                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    parameters=[recovery_yaml_real],
                    output='screen',
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml_sim],
                    condition=IfCondition(LaunchConfiguration('use_sim_time')),
                    remappings=[('/cmd_vel', '/robot/cmd_vel')]
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml_real],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),

                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[waypoint_follower_yaml]
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[{'autostart': True},
                                {'node_names': ['map_server',
                                                'amcl',
                                                'controller_server',
                                                'planner_server',
                                                'recoveries_server',
                                                'bt_navigator',
                                                'filter_mask_server',
                                                'costmap_filter_info_server', 'waypoint_follower']}])
            ],
        )


    ])