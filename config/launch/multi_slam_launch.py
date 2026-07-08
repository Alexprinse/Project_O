import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/home/alex/Documents/Omokai_Project/config/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use'
    )

    # Robot 1 (tb3_0) SLAM Toolbox Node
    tb3_0_slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='tb3_0',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    # Robot 2 (tb3_1) SLAM Toolbox Node
    tb3_1_slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='tb3_1',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    # Robot 3 (tb3_2) SLAM Toolbox Node
    tb3_2_slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='tb3_2',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(tb3_0_slam)
    ld.add_action(tb3_1_slam)
    ld.add_action(tb3_2_slam)
    return ld
