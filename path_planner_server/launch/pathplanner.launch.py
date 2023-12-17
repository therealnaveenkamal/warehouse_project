import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_sim.yaml')
    
    return LaunchDescription([     
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('path_planner_server'), 'config', 'pathplanning.rviz'])]
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
                            {'yaml_filename':map_file}],
                remappings=[('/cmd_vel', '/robot/cmd_vel')]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml],
                remappings=[('/cmd_vel', '/robot/cmd_vel')]
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml],
                remappings=[('/cmd_vel', '/robot/cmd_vel')]
            ),
            Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml],
            remappings=[('/cmd_vel', '/robot/cmd_vel')]
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                parameters=[recovery_yaml],
                remappings=[('/cmd_vel', '/robot/cmd_vel')],
                output='screen'
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml],
                remappings=[('/cmd_vel', '/robot/cmd_vel')]
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
                                            'bt_navigator']}]
            ),
            Node(
                    package='localization_server',
                    executable='initial_pose_publisher_cpp',
                    output='screen'
                )
        ],
        )
    ])