from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    map_file = 'warehouse_map_sim.yaml'
    rviz_config_file_name = 'map_rviz.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Name of the Mapfile (without path)'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': PathJoinSubstitution([FindPackageShare('map_server'), 'config', LaunchConfiguration('map_file')])}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('map_server'), 'config', rviz_config_file_name])]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        )            
    ])
