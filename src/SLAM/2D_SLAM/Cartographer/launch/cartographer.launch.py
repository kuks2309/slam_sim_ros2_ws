import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    odom_tf = LaunchConfiguration('odom_tf')

    # 현재 파일 기준으로 패키지 경로 자동 감지 (launch/ 상위 디렉토리)
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    cartographer_config_dir = os.path.join(pkg_src, 'config')
    configuration_basename = 'cartographer.lua'

    rviz_config_file = os.path.join(pkg_src, 'rviz', 'cartographer.rviz')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        declare_odom_tf,

        odom_to_tf_node,

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
