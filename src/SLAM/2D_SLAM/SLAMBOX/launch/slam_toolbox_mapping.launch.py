"""
SLAM Toolbox Mapping Launch File

Usage:
    ros2 launch slam_2d slam_toolbox_mapping.launch.py
    ros2 launch slam_2d slam_toolbox_mapping.launch.py use_sim_time:=true
    ros2 launch slam_2d slam_toolbox_mapping.launch.py use_sim_time:=true rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 현재 파일 기준으로 패키지 경로 자동 감지 (launch/ 상위 디렉토리)
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    default_params_file = os.path.join(
        pkg_src, 'config', 'mapper_params_online_async.yaml'
    )
    default_rviz_file = os.path.join(
        pkg_src, 'rviz', 'slam_toolbox.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    odom_tf = LaunchConfiguration('odom_tf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the SLAM Toolbox parameters file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_file,
        description='Full path to the RViz config file'
    )

    declare_odom_tf = DeclareLaunchArgument(
        'odom_tf',
        default_value='true',
        description='Enable odom to base_link TF publishing'
    )

    # Odom to TF: Publish odom -> base_link TF from /odom topic
    odom_to_tf_node = Node(
        package='tm_gazebo',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(odom_tf)
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_rviz,
        declare_rviz_config,
        declare_odom_tf,
        odom_to_tf_node,
        slam_toolbox_node,
        rviz_node,
    ])
