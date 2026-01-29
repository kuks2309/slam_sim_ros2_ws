import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 현재 파일 기준으로 패키지 경로 자동 감지 (launch/ 상위 디렉토리)
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    rviz_config = os.path.join(pkg_src, 'rviz', 'rtabmap_lidar.rviz')
    # workspace root: pkg_src → 2D_SLAM → SLAM → src → workspace
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_src))))
    default_db_path = os.path.join(workspace_root, 'maps', 'rtab-map', 'rtabmap.db')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true for Gazebo/ROS bag)'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=default_db_path,
        description='Path to rtabmap database file (.db)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # ICP Odometry 노드
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom_rtabmap',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            # ICP 파라미터 (2D LiDAR용)
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlane': 'false',
            'Odom/ScanKeyFrameThr': '0.6',
            'OdomF2M/ScanSubtractRadius': '0.05',
            'OdomF2M/ScanMaxSize': '5000',
        }],
        remappings=[
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # RTABMAP Localization 노드
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom_rtabmap',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'queue_size': 10,
            # Localization 모드 파라미터
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'database_path': database_path,
            # Grid Map publish 설정 (Localization에서 맵 표시용)
            'RGBD/CreateOccupancyGrid': 'true',
            'Grid/FromDepth': 'false',
            'Grid/RangeMax': '5.0',
            'Grid/RayTracing': 'true',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/rtabmap/odom'),
        ],
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        rviz_arg,
        icp_odometry,
        rtabmap_localization,
        rviz_node,
    ])
