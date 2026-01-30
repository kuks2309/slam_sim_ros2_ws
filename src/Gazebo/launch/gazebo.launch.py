import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 현재 파일 기준으로 패키지 경로 자동 감지
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    world_file = os.path.join(pkg_src, 'worlds', 'my_world.sdf')
    models_path = os.path.join(pkg_src, 'models')
    rviz_config = os.path.join(pkg_src, 'rviz', 'gazebo.rviz')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )

    odom_tf_arg = DeclareLaunchArgument(
        'odom_tf',
        default_value='true',
        description='Enable odom to base_link TF publishing'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Set IGN_GAZEBO_RESOURCE_PATH for Ignition Gazebo to find models
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_path
    )

    # Fix for WSL2 Ogre rendering issues
    set_ogre_fix1 = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )

    set_ogre_fix2 = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.3'
    )

    set_ogre_fix3 = SetEnvironmentVariable(
        name='MESA_GLSL_VERSION_OVERRIDE',
        value='330'
    )

    # Ignition Gazebo (Fortress)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    # ROS-Gazebo Bridge
    # Bridge cmd_vel (ROS2 -> Gazebo) and odometry, lidar, camera (Gazebo -> ROS2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Odom to TF: Publish odom -> base_link TF from /odom topic
    # CRITICAL: Always use simulation time for Gazebo
    odom_to_tf_script = os.path.join(pkg_src, 'scripts', 'odom_to_tf.py')
    odom_to_tf = Node(
        package='tm_gazebo',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': True}],  # Always True for simulation
        output='screen',
        condition=IfCondition(LaunchConfiguration('odom_tf'))
    )

    # Static TF: base_link -> lidar_link (lidar is 0.09m above base_link)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static TF: base_link -> camera_link (camera is 0.25m forward, 0.10m up from base_link)
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.25', '0', '0.10', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # rqt_robot_steering for manual control
    rqt_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ogre_fix1,
        set_ogre_fix2,
        set_ogre_fix3,
        world_arg,
        odom_tf_arg,
        use_sim_time_arg,
        gazebo,
        bridge,
        odom_to_tf,
        static_tf_lidar,
        static_tf_camera,
        rviz,
        rqt_steering,
    ])
