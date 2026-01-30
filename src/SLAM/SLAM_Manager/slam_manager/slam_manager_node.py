#!/usr/bin/env python3
"""
SLAM Manager Node - ROS2 node for managing 2D SLAM processes

This module handles:
- Process management for SLAM launch files
- ROS2 node with odometry subscriptions
- ROS Bag playback control
"""

import os
import subprocess
import signal
import time
import tempfile
import threading
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import math


def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (radians)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ProcessTracker:
    """Track a detached process by PID"""
    def __init__(self, pid):
        self.pid = pid

    def poll(self):
        """Check if process is still running"""
        try:
            os.kill(self.pid, 0)
            return None  # Still running
        except OSError:
            return 0  # Process ended


class SlamManagerNode(Node):
    """ROS2 Node for SLAM Manager"""

    def __init__(self, ui_window):
        super().__init__('slam_manager_node')
        self.ui = ui_window

        # Dictionary to store running processes
        self.processes = {
            'gazebo': None,
            'livox': None,
            'slamtoolbox_mapping': None,
            'slamtoolbox_loc': None,
            'carto_mapping': None,
            'carto_loc': None,
            'kslam': None,
            'rtabmap2d_mapping': None,
            'rtabmap2d_loc': None,
            'rtabmap2d_camera_mapping': None,
            'rtabmap2d_camera_loc': None,
        }

        # Store launch file paths
        self.launch_files = {
            'gazebo': None,
            'livox': None,
            'slamtoolbox_mapping': None,
            'slamtoolbox_loc': None,
            'carto_mapping': None,
            'carto_loc': None,
            'kslam': None,
            'rtabmap2d_mapping': None,
            'rtabmap2d_loc': None,
            'rtabmap2d_camera_mapping': None,
            'rtabmap2d_camera_loc': None,
        }

        # Workspace path (현재 파일 기준으로 자동 감지)
        # slam_manager_node.py → slam_manager → SLAM_Manager → SLAM → src → workspace
        current_file = Path(os.path.realpath(__file__))
        self.workspace_path = current_file.parent.parent.parent.parent.parent

        # Current positions
        self.slamtoolbox_x = 0.0
        self.slamtoolbox_y = 0.0
        self.slamtoolbox_yaw = 0.0

        self.carto_x = 0.0
        self.carto_y = 0.0
        self.carto_yaw = 0.0

        self.kslam_x = 0.0
        self.kslam_y = 0.0
        self.kslam_yaw = 0.0

        self.rtabmap2d_x = 0.0
        self.rtabmap2d_y = 0.0
        self.rtabmap2d_yaw = 0.0

        self.rtabmap2d_camera_x = 0.0
        self.rtabmap2d_camera_y = 0.0
        self.rtabmap2d_camera_yaw = 0.0

        # QoS profile for odometry
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to SLAM Toolbox pose (it publishes to /pose)
        self.slamtoolbox_pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.slamtoolbox_odom_callback,
            odom_qos
        )

        # Subscribe to Cartographer odometry (when available)
        self.carto_odom_sub = self.create_subscription(
            Odometry,
            '/carto/odom',
            self.carto_odom_callback,
            odom_qos
        )

        # Subscribe to K-SLAM pose
        self.kslam_pose_sub = self.create_subscription(
            Odometry,
            '/kslam/odom',
            self.kslam_odom_callback,
            odom_qos
        )

        # Subscribe to RTAB-Map 2D pose
        self.rtabmap2d_pose_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.rtabmap2d_odom_callback,
            odom_qos
        )

        self.get_logger().info('SLAM Manager Node initialized')

    def set_use_sim_time(self, enabled: bool):
        """Set use_sim_time parameter"""
        try:
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, enabled)])
            self.get_logger().info(f'use_sim_time set to {enabled}')
        except Exception as e:
            self.get_logger().warn(f'Failed to set use_sim_time: {e}')

    def slamtoolbox_odom_callback(self, msg):
        """Callback for SLAM Toolbox odometry"""
        self.slamtoolbox_x = msg.pose.pose.position.x
        self.slamtoolbox_y = msg.pose.pose.position.y
        self.slamtoolbox_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ui.update_slamtoolbox_position(
            self.slamtoolbox_x, self.slamtoolbox_y, self.slamtoolbox_yaw)

    def carto_odom_callback(self, msg):
        """Callback for Cartographer odometry"""
        self.carto_x = msg.pose.pose.position.x
        self.carto_y = msg.pose.pose.position.y
        self.carto_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ui.update_carto_position(
            self.carto_x, self.carto_y, self.carto_yaw)

    def kslam_odom_callback(self, msg):
        """Callback for K-SLAM odometry"""
        self.kslam_x = msg.pose.pose.position.x
        self.kslam_y = msg.pose.pose.position.y
        self.kslam_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ui.update_kslam_position(
            self.kslam_x, self.kslam_y, self.kslam_yaw)

    def rtabmap2d_odom_callback(self, msg):
        """Callback for RTAB-Map 2D odometry"""
        self.rtabmap2d_x = msg.pose.pose.position.x
        self.rtabmap2d_y = msg.pose.pose.position.y
        self.rtabmap2d_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ui.update_rtabmap2d_position(
            self.rtabmap2d_x, self.rtabmap2d_y, self.rtabmap2d_yaw)
        # Also update RTAB-Map 2D Camera position (same odom topic)
        self.rtabmap2d_camera_x = self.rtabmap2d_x
        self.rtabmap2d_camera_y = self.rtabmap2d_y
        self.rtabmap2d_camera_yaw = self.rtabmap2d_yaw
        self.ui.update_rtabmap2d_camera_position(
            self.rtabmap2d_camera_x, self.rtabmap2d_camera_y, self.rtabmap2d_camera_yaw)

    def start_launch_file(self, launch_key, launch_file_path, extra_args=None):
        """Start a ROS2 launch file"""
        if self.processes[launch_key] is not None:
            self.ui.log(f"Launch '{launch_key}' is already running!")
            return False

        if not launch_file_path or not os.path.exists(launch_file_path):
            self.ui.log(f"Launch file not found: {launch_file_path}")
            return False

        try:
            cmd = ['ros2', 'launch', launch_file_path]
            if extra_args:
                cmd.extend(extra_args)

            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'

            # PID tracking
            pid_file = tempfile.NamedTemporaryFile(mode='w', suffix='.pid', delete=False)
            pid_file_path = pid_file.name
            pid_file.close()

            # Create launch script
            script_content = f"""#!/bin/bash
set -e
source {self.workspace_path}/install/setup.bash
echo $$ > {pid_file_path}
cd {os.path.expanduser('~')}
exec {' '.join(cmd)}
"""
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                script_path = f.name
                f.write(script_content)

            os.chmod(script_path, 0o755)

            process = subprocess.Popen(
                ['setsid', 'bash', script_path],
                env=env,
                stdin=subprocess.DEVNULL,
                cwd=os.path.expanduser('~'),
                preexec_fn=os.setpgrp
            )

            time.sleep(0.5)

            actual_pid = None
            try:
                with open(pid_file_path, 'r') as f:
                    actual_pid = int(f.read().strip())
            except:
                actual_pid = process.pid

            # Cleanup temp files
            def cleanup_files():
                time.sleep(5)
                try:
                    os.unlink(script_path)
                    os.unlink(pid_file_path)
                except:
                    pass
            threading.Thread(target=cleanup_files, daemon=True).start()

            self.processes[launch_key] = ProcessTracker(actual_pid)
            self.ui.log(f"Started: {launch_file_path}")
            if extra_args:
                self.ui.log(f"  args: {' '.join(extra_args)}")
            self.get_logger().info(f"Started {launch_key}: PID={actual_pid}")
            return True

        except Exception as e:
            self.ui.log(f"Failed to start: {str(e)}")
            self.get_logger().error(f"Failed to start {launch_key}: {str(e)}")
            return False

    def stop_launch_file(self, launch_key):
        """Stop a running launch file"""
        if self.processes[launch_key] is None:
            self.ui.log(f"Launch '{launch_key}' is not running!")
            return False

        try:
            process = self.processes[launch_key]

            def get_process_tree(pid):
                """Get all child processes of a given PID"""
                try:
                    result = subprocess.run(
                        ['pgrep', '-P', str(pid)],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    child_pids = [int(p) for p in result.stdout.strip().split('\n') if p]
                    all_pids = child_pids.copy()
                    for child_pid in child_pids:
                        all_pids.extend(get_process_tree(child_pid))
                    return all_pids
                except:
                    return []

            all_pids = [process.pid] + get_process_tree(process.pid)
            self.ui.log(f"Stopping process tree: {all_pids}")

            # Kill specific ROS nodes for cartographer
            if launch_key in ['carto_mapping', 'carto_loc']:
                self.ui.log("Killing cartographer nodes...")
                subprocess.run(['pkill', '-9', '-f', 'cartographer_node'], timeout=5)
                subprocess.run(['pkill', '-9', '-f', 'cartographer_occupancy_grid_node'], timeout=5)
                time.sleep(0.5)

            # Kill specific ROS nodes for rtabmap
            if launch_key in ['rtabmap2d_mapping', 'rtabmap2d_loc',
                              'rtabmap2d_camera_mapping', 'rtabmap2d_camera_loc']:
                self.ui.log("Killing rtabmap nodes...")
                subprocess.run(['pkill', '-9', '-f', 'rtabmap_odom'], timeout=5)
                subprocess.run(['pkill', '-9', '-f', 'rtabmap_slam'], timeout=5)
                subprocess.run(['pkill', '-9', '-f', 'icp_odometry'], timeout=5)
                time.sleep(0.5)

            # Try to kill the entire process group first (setsid creates new group)
            try:
                pgid = os.getpgid(process.pid)
                self.ui.log(f"Sending SIGINT to process group {pgid}")
                os.killpg(pgid, signal.SIGINT)
            except (ProcessLookupError, OSError) as e:
                self.ui.log(f"Process group kill failed: {e}, trying individual PIDs")
                # Fallback: Send SIGINT to individual processes
                for pid in reversed(all_pids):
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass

            time.sleep(2)

            # Force kill surviving processes
            surviving_pids = []
            for pid in all_pids:
                try:
                    os.kill(pid, 0)
                    surviving_pids.append(pid)
                except ProcessLookupError:
                    pass

            if surviving_pids:
                self.ui.log(f"Force killing: {surviving_pids}")
                # Try process group SIGKILL first
                try:
                    pgid = os.getpgid(process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    # Fallback: kill individual processes
                    for pid in reversed(surviving_pids):
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass

                time.sleep(1)

            # Final cleanup for cartographer - ensure rviz is also killed if launched together
            if launch_key in ['carto_mapping', 'carto_loc']:
                # Kill any remaining cartographer-related processes
                subprocess.run(['pkill', '-9', '-f', 'cartographer'], timeout=5)

            # Final cleanup for rtabmap
            if launch_key in ['rtabmap2d_mapping', 'rtabmap2d_loc',
                              'rtabmap2d_camera_mapping', 'rtabmap2d_camera_loc']:
                # Kill any remaining rtabmap-related processes
                subprocess.run(['pkill', '-9', '-f', 'rtabmap'], timeout=5)
                subprocess.run(['pkill', '-9', '-f', 'icp_odometry'], timeout=5)

            self.processes[launch_key] = None
            self.ui.log(f"Stopped: {launch_key}")
            self.get_logger().info(f"Stopped {launch_key}")

            time.sleep(1)
            return True

        except Exception as e:
            self.ui.log(f"Failed to stop: {str(e)}")
            self.get_logger().error(f"Failed to stop {launch_key}: {str(e)}")
            return False

    def stop_all_launches(self):
        """Stop all running launch files"""
        for key in self.processes.keys():
            if self.processes[key] is not None:
                self.stop_launch_file(key)
        self.ui.log("All launches stopped")

    def is_running(self, launch_key):
        """Check if a launch file is currently running"""
        if self.processes[launch_key] is None:
            return False

        poll = self.processes[launch_key].poll()
        if poll is not None:
            self.processes[launch_key] = None
            return False

        return True

    def save_slamtoolbox_map(self, map_name):
        """Save SLAM Toolbox map using service call"""
        try:
            # Create map directory
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'map' / 'slam_toolbox' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Call slam_toolbox save map service
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{"filename": "{map_path}"}}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.ui.log(f"Map saved: {map_path}")
                self.get_logger().info(f"Map saved: {map_path}")
                return True, map_path
            else:
                self.ui.log(f"Failed to save map: {result.stderr}")
                return False, result.stderr

        except Exception as e:
            self.ui.log(f"Failed to save map: {str(e)}")
            return False, str(e)

    def save_carto_map(self, map_name):
        """Save Cartographer map (placeholder for future implementation)"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'map' / 'cartographer' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Cartographer uses finish_trajectory and write_state services
            # This is a placeholder - actual implementation depends on Cartographer setup
            self.ui.log(f"Cartographer map save not implemented yet")
            self.ui.log(f"Map would be saved to: {map_path}")
            return False, "Not implemented"

        except Exception as e:
            self.ui.log(f"Failed to save map: {str(e)}")
            return False, str(e)

    def save_kslam_map(self, map_name):
        """Save K-SLAM map using service call"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'map' / 'kslam' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Call K-SLAM save map service
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/k_slam/save_map',
                 'std_srvs/srv/Trigger',
                 '{}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.ui.log(f"K-SLAM map saved: {map_path}")
                self.get_logger().info(f"K-SLAM map saved: {map_path}")
                return True, map_path
            else:
                self.ui.log(f"Failed to save K-SLAM map: {result.stderr}")
                return False, result.stderr

        except Exception as e:
            self.ui.log(f"Failed to save K-SLAM map: {str(e)}")
            return False, str(e)

    def save_rtabmap2d_map(self, map_name):
        """Save RTAB-Map 2D map using service call"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = self.workspace_path / 'map' / 'rtabmap' / timestamp
            os.makedirs(map_dir, exist_ok=True)

            map_path = str(map_dir / map_name)

            # Call RTAB-Map save service (rtabmap_ros publishes map on demand)
            # First trigger map save
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/rtabmap/save_map',
                 'std_srvs/srv/Empty',
                 '{}'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.ui.log(f"RTAB-Map 2D map saved: {map_path}")
                self.get_logger().info(f"RTAB-Map 2D map saved: {map_path}")
                return True, map_path
            else:
                self.ui.log(f"Failed to save RTAB-Map 2D map: {result.stderr}")
                return False, result.stderr

        except Exception as e:
            self.ui.log(f"Failed to save RTAB-Map 2D map: {str(e)}")
            return False, str(e)


def main(args=None):
    """Main entry point"""
    import sys
    from PyQt5 import QtWidgets
    from PyQt5.QtCore import QTimer
    from slam_manager.slam_manager_ui import SlamManagerUI

    # Initialize ROS2
    rclpy.init(args=args)

    # Create Qt Application
    app = QtWidgets.QApplication(sys.argv)

    # Create UI
    ui = SlamManagerUI()

    # Create ROS2 Node
    node = SlamManagerNode(ui)
    ui.set_node(node)

    # Show UI
    ui.show()

    # Timer for ROS2 spinning
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
