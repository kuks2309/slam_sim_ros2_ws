#!/usr/bin/env python3
"""
SLAM Manager UI - PyQt5 UI class for SLAM Manager

This module handles:
- UI event handling and button connections
- ROS Bag playback control
- Launch file auto-detection
- Button state management
"""

import os
import sys
import signal
import time
import subprocess
import math
from pathlib import Path
from datetime import datetime

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtWidgets import QFileDialog, QMessageBox


class SlamManagerUI(QtWidgets.QMainWindow):
    """PyQt5 Main Window for SLAM Manager"""

    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = Path(__file__).parent / 'ui' / 'slam_manager.ui'
        uic.loadUi(ui_file, self)

        # ROS2 node (will be set later)
        self.node = None

        # Workspace path (현재 파일 기준으로 자동 감지)
        # slam_manager_ui.py → slam_manager → SLAM_Manager → SLAM → src → workspace
        current_file = Path(os.path.realpath(__file__))
        self.workspace_path = current_file.parent.parent.parent.parent.parent

        # ROS Bag state
        self.bag_process = None
        self.bag_paused = False
        self.bag_duration = 0.0
        self.bag_start_time = None

        # Connect buttons - Data Source tab
        self.btnStartGazebo.clicked.connect(self.on_start_gazebo)
        self.btnStopGazebo.clicked.connect(self.on_stop_gazebo)
        self.btnStartLivox.clicked.connect(self.on_start_livox)
        self.btnStopLivox.clicked.connect(self.on_stop_livox)
        self.btnBrowseBag.clicked.connect(self.on_browse_bag)
        self.btnPlayBag.clicked.connect(self.on_play_bag)
        self.btnPauseBag.clicked.connect(self.on_pause_bag)
        self.btnStopBag.clicked.connect(self.on_stop_bag)

        # Connect buttons - SLAM Toolbox tab
        self.btnStartSlamToolboxMapping.clicked.connect(self.on_start_slamtoolbox_mapping)
        self.btnStopSlamToolboxMapping.clicked.connect(self.on_stop_slamtoolbox_mapping)
        self.btnSaveSlamToolboxMap.clicked.connect(self.on_save_slamtoolbox_map)
        self.btnBrowseSlamToolboxMap.clicked.connect(self.on_browse_slamtoolbox_map)
        self.btnStartSlamToolboxLoc.clicked.connect(self.on_start_slamtoolbox_loc)
        self.btnStopSlamToolboxLoc.clicked.connect(self.on_stop_slamtoolbox_loc)

        # Connect buttons - Cartographer tab
        self.btnStartCartoMapping.clicked.connect(self.on_start_carto_mapping)
        self.btnStopCartoMapping.clicked.connect(self.on_stop_carto_mapping)
        self.btnSaveCartoMap.clicked.connect(self.on_save_carto_map)
        self.btnBrowseCartoMap.clicked.connect(self.on_browse_carto_map)
        self.btnStartCartoLoc.clicked.connect(self.on_start_carto_loc)
        self.btnStopCartoLoc.clicked.connect(self.on_stop_carto_loc)

        # Connect buttons - K-SLAM tab
        self.btnStartKSlam.clicked.connect(self.on_start_kslam)
        self.btnStopKSlam.clicked.connect(self.on_stop_kslam)
        self.btnSaveKSlamMap.clicked.connect(self.on_save_kslam_map)

        # Connect buttons - RTAB-Map 2D tab
        self.btnStartRtabmap2DMapping.clicked.connect(self.on_start_rtabmap2d_mapping)
        self.btnStopRtabmap2DMapping.clicked.connect(self.on_stop_rtabmap2d_mapping)
        self.btnSaveRtabmap2DMap.clicked.connect(self.on_save_rtabmap2d_map)
        self.btnBrowseRtabmap2DMap.clicked.connect(self.on_browse_rtabmap2d_map)
        self.btnStartRtabmap2DLoc.clicked.connect(self.on_start_rtabmap2d_loc)
        self.btnStopRtabmap2DLoc.clicked.connect(self.on_stop_rtabmap2d_loc)

        # Connect buttons - RTAB-Map 2D Camera tab
        self.btnStartRtabmap2DCameraMapping.clicked.connect(self.on_start_rtabmap2d_camera_mapping)
        self.btnStopRtabmap2DCameraMapping.clicked.connect(self.on_stop_rtabmap2d_camera_mapping)
        self.btnSaveRtabmap2DCameraMap.clicked.connect(self.on_save_rtabmap2d_camera_map)
        self.btnBrowseRtabmap2DCameraMap.clicked.connect(self.on_browse_rtabmap2d_camera_map)
        self.btnStartRtabmap2DCameraLoc.clicked.connect(self.on_start_rtabmap2d_camera_loc)
        self.btnStopRtabmap2DCameraLoc.clicked.connect(self.on_stop_rtabmap2d_camera_loc)

        # Connect common buttons
        self.btnStopAll.clicked.connect(self.on_stop_all)

        # Status timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)

        # Set default map paths (workspace 기준)
        self.txtSlamToolboxMapPath.setText(
            str(self.workspace_path / "map" / "slam_toolbox") + "/")
        self.txtCartoMapPath.setText(
            str(self.workspace_path / "map" / "cartographer") + "/")
        self.txtRtabmap2DMapPath.setText(
            str(self.workspace_path / "map" / "rtabmap") + "/")
        self.txtRtabmap2DCameraMapPath.setText(
            str(self.workspace_path / "map" / "rtabmap") + "/")

        self.log("SLAM Manager UI Ready")

    def set_node(self, node):
        """Set the ROS2 node"""
        self.node = node
        self.auto_detect_launch_files()
        self.update_button_states()

    def auto_detect_launch_files(self):
        """Auto-detect launch files in the workspace"""
        src_path = self.workspace_path / 'src'

        # Gazebo simulation launch file
        gazebo_launch = src_path / 'Gazebo' / 'launch' / 'gazebo.launch.py'
        if gazebo_launch.exists():
            self.node.launch_files['gazebo'] = str(gazebo_launch)
            self.log(f"Found Gazebo: {gazebo_launch}")

        # Livox MID-360 launch file
        livox_launch = Path(os.path.expanduser(
            '~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py'))
        if livox_launch.exists():
            self.node.launch_files['livox'] = str(livox_launch)
            self.log(f"Found Livox: {livox_launch}")

        # SLAM Toolbox mapping
        slamtoolbox_mapping = src_path / 'SLAM' / '2D_SLAM' / 'SLAMBOX' / 'launch' / 'slam_toolbox_mapping.launch.py'
        if slamtoolbox_mapping.exists():
            self.node.launch_files['slamtoolbox_mapping'] = str(slamtoolbox_mapping)
            self.log(f"Found SLAM Toolbox Mapping: {slamtoolbox_mapping}")

        # SLAM Toolbox localization
        slamtoolbox_loc = src_path / 'SLAM' / '2D_SLAM' / 'SLAMBOX' / 'launch' / 'slam_toolbox_localization.launch.py'
        if slamtoolbox_loc.exists():
            self.node.launch_files['slamtoolbox_loc'] = str(slamtoolbox_loc)
            self.log(f"Found SLAM Toolbox Localization: {slamtoolbox_loc}")

        # Cartographer mapping
        carto_mapping = src_path / 'SLAM' / '2D_SLAM' / 'Cartographer' / 'launch' / 'cartographer.launch.py'
        if carto_mapping.exists():
            self.node.launch_files['carto_mapping'] = str(carto_mapping)
            self.log(f"Found Cartographer Mapping: {carto_mapping}")

        # Cartographer localization
        carto_loc = src_path / 'SLAM' / '2D_SLAM' / 'Cartographer' / 'launch' / 'cartographer_localization.launch.py'
        if carto_loc.exists():
            self.node.launch_files['carto_loc'] = str(carto_loc)
            self.log(f"Found Cartographer Localization: {carto_loc}")

        # K-SLAM
        kslam_launch = src_path / 'SLAM' / '2D_SLAM' / 'k_slam_ros2' / 'k_mapping' / 'launch' / 'k_slam.launch.py'
        if kslam_launch.exists():
            self.node.launch_files['kslam'] = str(kslam_launch)
            self.log(f"Found K-SLAM: {kslam_launch}")

        # RTAB-Map 2D mapping
        rtabmap2d_mapping = src_path / 'SLAM' / '2D_SLAM' / 'rtab_map' / 'launch' / 'rtabmap_lidar.launch.py'
        if rtabmap2d_mapping.exists():
            self.node.launch_files['rtabmap2d_mapping'] = str(rtabmap2d_mapping)
            self.log(f"Found RTAB-Map 2D Mapping: {rtabmap2d_mapping}")

        # RTAB-Map 2D localization
        rtabmap2d_loc = src_path / 'SLAM' / '2D_SLAM' / 'rtab_map' / 'launch' / 'rtabmap_lidar_localization.launch.py'
        if rtabmap2d_loc.exists():
            self.node.launch_files['rtabmap2d_loc'] = str(rtabmap2d_loc)
            self.log(f"Found RTAB-Map 2D Localization: {rtabmap2d_loc}")

        # RTAB-Map 2D Camera mapping
        rtabmap2d_camera_mapping = src_path / 'SLAM' / '2D_SLAM' / 'rtab_map' / 'launch' / 'rtabmap_lidar_camera.launch.py'
        if rtabmap2d_camera_mapping.exists():
            self.node.launch_files['rtabmap2d_camera_mapping'] = str(rtabmap2d_camera_mapping)
            self.log(f"Found RTAB-Map 2D Camera Mapping: {rtabmap2d_camera_mapping}")

        # RTAB-Map 2D Camera localization
        rtabmap2d_camera_loc = src_path / 'SLAM' / '2D_SLAM' / 'rtab_map' / 'launch' / 'rtabmap_lidar_camera_localization.launch.py'
        if rtabmap2d_camera_loc.exists():
            self.node.launch_files['rtabmap2d_camera_loc'] = str(rtabmap2d_camera_loc)
            self.log(f"Found RTAB-Map 2D Camera Localization: {rtabmap2d_camera_loc}")

    def log(self, message):
        """Add message to log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.txtLog.append(f"[{timestamp}] {message}")

    def update_slamtoolbox_position(self, x, y, yaw):
        """Update SLAM Toolbox position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblSlamToolboxPosX.setText(f"X: {x:.3f}")
        self.lblSlamToolboxPosY.setText(f"Y: {y:.3f}")
        self.lblSlamToolboxPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    def update_carto_position(self, x, y, yaw):
        """Update Cartographer position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblCartoPosX.setText(f"X: {x:.3f}")
        self.lblCartoPosY.setText(f"Y: {y:.3f}")
        self.lblCartoPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== Data Source Tab ====================

    def on_start_gazebo(self):
        """Start Gazebo simulation"""
        if self.node.launch_files.get('gazebo'):
            extra_args = ['rviz:=false']  # Gazebo가 odom_to_tf 실행 (원래대로)
            if self.node.start_launch_file('gazebo', self.node.launch_files['gazebo'], extra_args):
                self.chkUseSimTime.setChecked(True)
                self.update_button_states()
                self.log("Gazebo simulation started")
        else:
            self.log("Gazebo launch file not found!")
            QMessageBox.warning(self, "Error", "Gazebo launch file not found!")

    def on_stop_gazebo(self):
        """Stop Gazebo simulation"""
        if self.node.stop_launch_file('gazebo'):
            self.update_button_states()
            self.log("Gazebo simulation stopped")

    def on_start_livox(self):
        if self.node.launch_files['livox']:
            if self.node.start_launch_file('livox', self.node.launch_files['livox']):
                self.chkUseSimTime.setChecked(False)
                self.update_button_states()
        else:
            self.log("Livox launch file not configured!")
            QMessageBox.warning(self, "Error", "Livox launch file not found!")

    def on_stop_livox(self):
        if self.node.stop_launch_file('livox'):
            self.update_button_states()

    def on_browse_bag(self):
        """Browse for ROS bag file"""
        default_path = self.workspace_path / 'bag'
        if not default_path.exists():
            default_path = Path.home()

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select ROS Bag",
            str(default_path),
            "ROS Bag (*.db3 *.mcap);;All Files (*)"
        )
        if file_path:
            if file_path.endswith('.db3') or file_path.endswith('.mcap'):
                bag_dir = str(Path(file_path).parent)
                self.txtBagFile.setText(bag_dir)
            else:
                self.txtBagFile.setText(file_path)

    def _get_bag_info(self, bag_path):
        """Get bag duration using ros2 bag info"""
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Duration:' in line:
                        duration_str = line.split('Duration:')[1].strip()
                        total_seconds = 0.0
                        if 'm' in duration_str:
                            parts = duration_str.replace('s', '').split('m')
                            total_seconds = float(parts[0]) * 60 + float(parts[1].strip())
                        else:
                            total_seconds = float(duration_str.replace('s', ''))
                        return total_seconds
        except Exception as e:
            self.log(f"Failed to get bag info: {e}")
        return 0.0

    def on_play_bag(self):
        """Play ROS bag"""
        bag_path = self.txtBagFile.text()
        if not bag_path:
            self.log("Please select a ROS bag file!")
            QMessageBox.warning(self, "Error", "Please select a ROS bag file!")
            return

        if not os.path.exists(bag_path):
            self.log(f"Bag path not found: {bag_path}")
            QMessageBox.warning(self, "Error", f"Bag path not found:\n{bag_path}")
            return

        try:
            self.bag_duration = self._get_bag_info(bag_path)
            self.log(f"Bag duration: {self.bag_duration:.1f}s")

            cmd = ['ros2', 'bag', 'play', bag_path]

            if self.chkBagLoop.isChecked():
                cmd.append('--loop')

            rate = self.spinBagRate.value()
            if rate != 1.0:
                cmd.extend(['--rate', str(rate)])

            if self.chkBagClock.isChecked():
                cmd.append('--clock')
                self.node.set_use_sim_time(True)

            env = os.environ.copy()

            self.bag_process = subprocess.Popen(
                cmd,
                env=env,
                stdin=subprocess.PIPE,
                preexec_fn=os.setpgrp
            )

            self.bag_paused = False
            self.bag_start_time = QDateTime.currentDateTime()
            self.progressBag.setValue(0)
            self.chkUseSimTime.setChecked(True)
            self.log(f"Playing bag: {bag_path}")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to play bag: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to play bag:\n{str(e)}")

    def on_pause_bag(self):
        """Pause/Resume ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.bag_paused = False
                self.log("Bag playback resumed")
            else:
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.bag_paused = True
                self.log("Bag playback paused")

            self._update_bag_button_states()

        except Exception as e:
            self.log(f"Failed to pause/resume bag: {str(e)}")

    def on_stop_bag(self):
        """Stop ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)

            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            time.sleep(1)

            if self.bag_process.poll() is None:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)

            self.bag_process = None
            self.bag_paused = False
            self.bag_start_time = None
            self.node.set_use_sim_time(False)
            self.log("Bag playback stopped")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to stop bag: {str(e)}")
            self.bag_process = None
            self.bag_paused = False
            self.node.set_use_sim_time(False)
            self.bag_start_time = None
            self._update_bag_button_states()

    # ==================== SLAM Toolbox Tab ====================

    def on_start_slamtoolbox_mapping(self):
        self.log("Start Mapping button clicked!")
        if self.node.launch_files['slamtoolbox_mapping']:
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            if self.node.start_launch_file('slamtoolbox_mapping',
                                           self.node.launch_files['slamtoolbox_mapping'],
                                           extra_args):
                self.update_button_states()
        else:
            self.log("SLAM Toolbox mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox mapping launch file not found!")

    def on_stop_slamtoolbox_mapping(self):
        if self.node.stop_launch_file('slamtoolbox_mapping'):
            self.update_button_states()

    def on_save_slamtoolbox_map(self):
        """Save SLAM Toolbox map"""
        success, result = self.node.save_slamtoolbox_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_slamtoolbox_map(self):
        """Browse for SLAM Toolbox map file"""
        default_path = str(self.workspace_path / "map" / "slam_toolbox")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select SLAM Toolbox Map (.posegraph)",
            default_path,
            "Posegraph Files (*.posegraph);;All Files (*)"
        )
        if file_path:
            self.txtSlamToolboxMapPath.setText(file_path)
            self.log(f"SLAM Toolbox map selected: {file_path}")

    def on_start_slamtoolbox_loc(self):
        if self.node.launch_files['slamtoolbox_loc']:
            map_path = self.txtSlamToolboxMapPath.text()

            # Check if path is a .posegraph file
            if not map_path.endswith('.posegraph'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .posegraph file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .posegraph file.")
                return

            # SLAM Toolbox requires map_file_name WITHOUT extension
            map_path_no_ext = map_path[:-10]  # Remove '.posegraph'

            # Also check if .data file exists
            data_file = map_path_no_ext + '.data'
            if not os.path.exists(data_file):
                self.log(f"Map data file not found: {data_file}")
                QMessageBox.warning(self, "Error",
                                    f"Map data file not found:\n{data_file}\n\n"
                                    f"Both .posegraph and .data files are required.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'map_file:={map_path_no_ext}')

            if self.node.start_launch_file('slamtoolbox_loc',
                                           self.node.launch_files['slamtoolbox_loc'],
                                           extra_args):
                self.log(f"SLAM Toolbox Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("SLAM Toolbox localization launch file not configured!")
            QMessageBox.warning(self, "Error", "SLAM Toolbox localization launch file not found!")

    def on_stop_slamtoolbox_loc(self):
        if self.node.stop_launch_file('slamtoolbox_loc'):
            self.update_button_states()

    # ==================== Cartographer Tab ====================

    def on_start_carto_mapping(self):
        if self.node.launch_files['carto_mapping']:
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')

            if self.node.start_launch_file('carto_mapping',
                                           self.node.launch_files['carto_mapping'],
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("Cartographer mapping launch file not configured!")
            QMessageBox.warning(self, "Error",
                                "Cartographer mapping launch file not found!\n\n"
                                "Please configure Cartographer first.")

    def on_stop_carto_mapping(self):
        if self.node.stop_launch_file('carto_mapping'):
            self.update_button_states()

    def on_save_carto_map(self):
        """Save Cartographer map"""
        success, result = self.node.save_carto_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Info", f"Cartographer map save:\n{result}")

    def on_browse_carto_map(self):
        """Browse for Cartographer map file"""
        default_path = str(self.workspace_path / "map" / "cartographer")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Cartographer Map (.pbstream)",
            default_path,
            "PBStream Files (*.pbstream);;All Files (*)"
        )
        if file_path:
            self.txtCartoMapPath.setText(file_path)
            self.log(f"Cartographer map selected: {file_path}")

    def on_start_carto_loc(self):
        if self.node.launch_files['carto_loc']:
            map_path = self.txtCartoMapPath.text()
            if not map_path or not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .pbstream file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            extra_args.append(f'load_state_filename:={map_path}')

            if self.node.start_launch_file('carto_loc',
                                           self.node.launch_files['carto_loc'],
                                           extra_args):
                self.log(f"Cartographer Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("Cartographer localization launch file not configured!")
            QMessageBox.warning(self, "Error",
                                "Cartographer localization launch file not found!\n\n"
                                "Please configure Cartographer first.")

    def on_stop_carto_loc(self):
        if self.node.stop_launch_file('carto_loc'):
            self.update_button_states()

    # ==================== K-SLAM Tab ====================

    def on_start_kslam(self):
        if self.node.launch_files['kslam']:
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            if self.node.start_launch_file('kslam',
                                           self.node.launch_files['kslam'],
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("K-SLAM launch file not configured!")
            QMessageBox.warning(self, "Error", "K-SLAM launch file not found!")

    def on_stop_kslam(self):
        if self.node.stop_launch_file('kslam'):
            self.update_button_states()

    def on_save_kslam_map(self):
        """Save K-SLAM map"""
        success, result = self.node.save_kslam_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def update_kslam_position(self, x, y, yaw):
        """Update K-SLAM position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblKSlamPosX.setText(f"X: {x:.3f}")
        self.lblKSlamPosY.setText(f"Y: {y:.3f}")
        self.lblKSlamPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== RTAB-Map 2D Tab ====================

    def on_start_rtabmap2d_mapping(self):
        if self.node.launch_files['rtabmap2d_mapping']:
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            if self.node.start_launch_file('rtabmap2d_mapping',
                                           self.node.launch_files['rtabmap2d_mapping'],
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D mapping launch file not found!")

    def on_stop_rtabmap2d_mapping(self):
        if self.node.stop_launch_file('rtabmap2d_mapping'):
            self.update_button_states()

    def on_save_rtabmap2d_map(self):
        """Save RTAB-Map 2D map"""
        success, result = self.node.save_rtabmap2d_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_rtabmap2d_map(self):
        """Browse for RTAB-Map 2D map file"""
        default_path = str(self.workspace_path / "map" / "rtabmap")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRtabmap2DMapPath.setText(file_path)
            self.log(f"RTAB-Map 2D map selected: {file_path}")

    def on_start_rtabmap2d_loc(self):
        if self.node.launch_files['rtabmap2d_loc']:
            map_path = self.txtRtabmap2DMapPath.text()

            # Check if path is a .db file
            if not map_path.endswith('.db'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .db file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .db file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'database_path:={map_path}')

            if self.node.start_launch_file('rtabmap2d_loc',
                                           self.node.launch_files['rtabmap2d_loc'],
                                           extra_args):
                self.log(f"RTAB-Map 2D Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D localization launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D localization launch file not found!")

    def on_stop_rtabmap2d_loc(self):
        if self.node.stop_launch_file('rtabmap2d_loc'):
            self.update_button_states()

    def update_rtabmap2d_position(self, x, y, yaw):
        """Update RTAB-Map 2D position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblRtabmap2DPosX.setText(f"X: {x:.3f}")
        self.lblRtabmap2DPosY.setText(f"Y: {y:.3f}")
        self.lblRtabmap2DPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== RTAB-Map 2D Camera Tab ====================

    def on_start_rtabmap2d_camera_mapping(self):
        if self.node.launch_files['rtabmap2d_camera_mapping']:
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')

            if self.node.start_launch_file('rtabmap2d_camera_mapping',
                                           self.node.launch_files['rtabmap2d_camera_mapping'],
                                           extra_args if extra_args else None):
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D Camera mapping launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D Camera mapping launch file not found!")

    def on_stop_rtabmap2d_camera_mapping(self):
        if self.node.stop_launch_file('rtabmap2d_camera_mapping'):
            self.update_button_states()

    def on_save_rtabmap2d_camera_map(self):
        """Save RTAB-Map 2D Camera map"""
        success, result = self.node.save_rtabmap2d_map("map")
        if success:
            QMessageBox.information(self, "Success",
                                    f"Map saved successfully!\n\nLocation: {result}")
        else:
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{result}")

    def on_browse_rtabmap2d_camera_map(self):
        """Browse for RTAB-Map 2D Camera map file"""
        default_path = str(self.workspace_path / "map" / "rtabmap")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRtabmap2DCameraMapPath.setText(file_path)
            self.log(f"RTAB-Map 2D Camera map selected: {file_path}")

    def on_start_rtabmap2d_camera_loc(self):
        if self.node.launch_files['rtabmap2d_camera_loc']:
            map_path = self.txtRtabmap2DCameraMapPath.text()

            # Check if path is a .db file
            if not map_path.endswith('.db'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Please select a .db file, not a folder.\n\n"
                                    f"Use Browse button to select the map file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error",
                                    f"Map file not found:\n{map_path}\n\nPlease select a valid .db file.")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            extra_args.append(f'database_path:={map_path}')

            if self.node.start_launch_file('rtabmap2d_camera_loc',
                                           self.node.launch_files['rtabmap2d_camera_loc'],
                                           extra_args):
                self.log(f"RTAB-Map 2D Camera Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map 2D Camera localization launch file not configured!")
            QMessageBox.warning(self, "Error", "RTAB-Map 2D Camera localization launch file not found!")

    def on_stop_rtabmap2d_camera_loc(self):
        if self.node.stop_launch_file('rtabmap2d_camera_loc'):
            self.update_button_states()

    def update_rtabmap2d_camera_position(self, x, y, yaw):
        """Update RTAB-Map 2D Camera position labels"""
        yaw_deg = math.degrees(yaw)
        self.lblRtabmap2DCameraPosX.setText(f"X: {x:.3f}")
        self.lblRtabmap2DCameraPosY.setText(f"Y: {y:.3f}")
        self.lblRtabmap2DCameraPosYaw.setText(f"Yaw: {yaw_deg:.1f}°")

    # ==================== Common ====================

    def on_stop_all(self):
        reply = QMessageBox.question(
            self,
            "Confirm",
            "Stop all running processes?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.node.stop_all_launches()
            if self.bag_process is not None:
                self.on_stop_bag()
            self.update_button_states()

    def _format_time(self, seconds):
        """Format seconds to MM:SS"""
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes:02d}:{secs:02d}"

    def _update_bag_button_states(self):
        """Update ROS bag button states"""
        bag_running = self.bag_process is not None and self.bag_process.poll() is None

        self.btnPlayBag.setEnabled(not bag_running)
        self.btnPauseBag.setEnabled(bag_running)
        self.btnStopBag.setEnabled(bag_running)

        if self.bag_paused:
            self.btnPauseBag.setText("Resume")
        else:
            self.btnPauseBag.setText("Pause")

        if bag_running and self.bag_duration > 0 and self.bag_start_time is not None:
            elapsed_ms = self.bag_start_time.msecsTo(QDateTime.currentDateTime())
            rate = self.spinBagRate.value()
            elapsed_sec = (elapsed_ms / 1000.0) * rate

            if self.chkBagLoop.isChecked():
                elapsed_sec = elapsed_sec % self.bag_duration

            progress = min(100, int((elapsed_sec / self.bag_duration) * 100))
            self.progressBag.setValue(progress)

            current_time = self._format_time(min(elapsed_sec, self.bag_duration))
            total_time = self._format_time(self.bag_duration)
            self.progressBag.setFormat(f"{progress}% - {current_time} / {total_time}")
        elif not bag_running:
            if self.bag_process is None:
                self.progressBag.setValue(0)
                self.progressBag.setFormat("0% - 00:00 / 00:00")

    def update_button_states(self):
        """Update all button states based on running processes"""
        if self.node is None:
            return

        # Style definitions
        style_ready = "background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;"
        style_running = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_loc = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_save = "background-color: #FF9800; color: white; font-weight: bold; padding: 10px;"
        style_gazebo = "background-color: #9C27B0; color: white; font-weight: bold; padding: 10px;"

        # Check running states
        gazebo_running = self.node.is_running('gazebo')
        livox_running = self.node.is_running('livox')
        bag_running = self.bag_process is not None and self.bag_process.poll() is None
        data_source_available = gazebo_running or livox_running or bag_running

        # Gazebo
        gazebo_configured = self.node.launch_files.get('gazebo') is not None
        self.btnStartGazebo.setEnabled(gazebo_configured and not gazebo_running)
        self.btnStopGazebo.setEnabled(gazebo_running)
        if gazebo_running:
            self.btnStartGazebo.setStyleSheet(style_running)
            self.lblGazeboStatus.setText("Status: Running")
        else:
            self.btnStartGazebo.setStyleSheet(style_gazebo)
            self.lblGazeboStatus.setText("Status: Stopped")

        slamtoolbox_mapping_running = self.node.is_running('slamtoolbox_mapping')
        slamtoolbox_loc_running = self.node.is_running('slamtoolbox_loc')
        carto_mapping_running = self.node.is_running('carto_mapping')
        carto_loc_running = self.node.is_running('carto_loc')

        # Livox
        self.btnStartLivox.setEnabled(not livox_running)
        self.btnStopLivox.setEnabled(livox_running)
        if livox_running:
            self.btnStartLivox.setStyleSheet(style_running)
            self.lblLivoxStatus.setText("Status: Running")
        else:
            self.btnStartLivox.setStyleSheet(style_ready)
            self.lblLivoxStatus.setText("Status: Stopped")

        # SLAM Toolbox Mapping (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        self.btnStartSlamToolboxMapping.setEnabled(
            not slamtoolbox_mapping_running and not slamtoolbox_loc_running)
        self.btnStopSlamToolboxMapping.setEnabled(slamtoolbox_mapping_running)
        self.btnSaveSlamToolboxMap.setEnabled(slamtoolbox_mapping_running)
        if slamtoolbox_mapping_running:
            self.btnStartSlamToolboxMapping.setStyleSheet(style_running)
        else:
            self.btnStartSlamToolboxMapping.setStyleSheet(style_ready)
        self.btnSaveSlamToolboxMap.setStyleSheet(style_save)

        # SLAM Toolbox Localization (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        self.btnStartSlamToolboxLoc.setEnabled(
            not slamtoolbox_loc_running and not slamtoolbox_mapping_running)
        self.btnStopSlamToolboxLoc.setEnabled(slamtoolbox_loc_running)
        if slamtoolbox_loc_running:
            self.btnStartSlamToolboxLoc.setStyleSheet(style_running)
        else:
            self.btnStartSlamToolboxLoc.setStyleSheet(style_loc)

        # Cartographer Mapping (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        carto_configured = self.node.launch_files['carto_mapping'] is not None
        self.btnStartCartoMapping.setEnabled(
            carto_configured and not carto_mapping_running and not carto_loc_running)
        self.btnStopCartoMapping.setEnabled(carto_mapping_running)
        self.btnSaveCartoMap.setEnabled(carto_mapping_running)
        if carto_mapping_running:
            self.btnStartCartoMapping.setStyleSheet(style_running)
        else:
            self.btnStartCartoMapping.setStyleSheet(style_ready)
        self.btnSaveCartoMap.setStyleSheet(style_save)

        # Cartographer Localization (data_source_available 조건 임시 제거 - Gazebo 테스트용)
        carto_loc_configured = self.node.launch_files['carto_loc'] is not None
        self.btnStartCartoLoc.setEnabled(
            carto_loc_configured and not carto_loc_running and not carto_mapping_running)
        self.btnStopCartoLoc.setEnabled(carto_loc_running)
        if carto_loc_running:
            self.btnStartCartoLoc.setStyleSheet(style_running)
        else:
            self.btnStartCartoLoc.setStyleSheet(style_loc)

        # Update Cartographer not configured label
        if not carto_configured:
            self.lblCartoNotConfigured.setVisible(True)
        else:
            self.lblCartoNotConfigured.setVisible(False)

        # K-SLAM
        kslam_running = self.node.is_running('kslam')
        kslam_configured = self.node.launch_files['kslam'] is not None
        self.btnStartKSlam.setEnabled(kslam_configured and not kslam_running)
        self.btnStopKSlam.setEnabled(kslam_running)
        self.btnSaveKSlamMap.setEnabled(kslam_running)
        if kslam_running:
            self.btnStartKSlam.setStyleSheet(style_running)
        else:
            self.btnStartKSlam.setStyleSheet(style_ready)
        self.btnSaveKSlamMap.setStyleSheet(style_save)

        # RTAB-Map 2D
        rtabmap2d_mapping_running = self.node.is_running('rtabmap2d_mapping')
        rtabmap2d_loc_running = self.node.is_running('rtabmap2d_loc')
        rtabmap2d_mapping_configured = self.node.launch_files['rtabmap2d_mapping'] is not None
        rtabmap2d_loc_configured = self.node.launch_files['rtabmap2d_loc'] is not None

        # RTAB-Map 2D Mapping
        self.btnStartRtabmap2DMapping.setEnabled(
            rtabmap2d_mapping_configured and not rtabmap2d_mapping_running and not rtabmap2d_loc_running)
        self.btnStopRtabmap2DMapping.setEnabled(rtabmap2d_mapping_running)
        self.btnSaveRtabmap2DMap.setEnabled(rtabmap2d_mapping_running)
        if rtabmap2d_mapping_running:
            self.btnStartRtabmap2DMapping.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DMapping.setStyleSheet(style_ready)
        self.btnSaveRtabmap2DMap.setStyleSheet(style_save)

        # RTAB-Map 2D Localization
        self.btnStartRtabmap2DLoc.setEnabled(
            rtabmap2d_loc_configured and not rtabmap2d_loc_running and not rtabmap2d_mapping_running)
        self.btnStopRtabmap2DLoc.setEnabled(rtabmap2d_loc_running)
        if rtabmap2d_loc_running:
            self.btnStartRtabmap2DLoc.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DLoc.setStyleSheet(style_loc)

        # RTAB-Map 2D Camera
        rtabmap2d_camera_mapping_running = self.node.is_running('rtabmap2d_camera_mapping')
        rtabmap2d_camera_loc_running = self.node.is_running('rtabmap2d_camera_loc')
        rtabmap2d_camera_mapping_configured = self.node.launch_files.get('rtabmap2d_camera_mapping') is not None
        rtabmap2d_camera_loc_configured = self.node.launch_files.get('rtabmap2d_camera_loc') is not None

        # RTAB-Map 2D Camera Mapping
        self.btnStartRtabmap2DCameraMapping.setEnabled(
            rtabmap2d_camera_mapping_configured and not rtabmap2d_camera_mapping_running and not rtabmap2d_camera_loc_running)
        self.btnStopRtabmap2DCameraMapping.setEnabled(rtabmap2d_camera_mapping_running)
        self.btnSaveRtabmap2DCameraMap.setEnabled(rtabmap2d_camera_mapping_running)
        if rtabmap2d_camera_mapping_running:
            self.btnStartRtabmap2DCameraMapping.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DCameraMapping.setStyleSheet(style_ready)
        self.btnSaveRtabmap2DCameraMap.setStyleSheet(style_save)

        # RTAB-Map 2D Camera Localization
        self.btnStartRtabmap2DCameraLoc.setEnabled(
            rtabmap2d_camera_loc_configured and not rtabmap2d_camera_loc_running and not rtabmap2d_camera_mapping_running)
        self.btnStopRtabmap2DCameraLoc.setEnabled(rtabmap2d_camera_loc_running)
        if rtabmap2d_camera_loc_running:
            self.btnStartRtabmap2DCameraLoc.setStyleSheet(style_running)
        else:
            self.btnStartRtabmap2DCameraLoc.setStyleSheet(style_loc)

        # ROS Bag
        self._update_bag_button_states()

    def closeEvent(self, event):
        """Handle window close event"""
        reply = QMessageBox.question(
            self,
            "Confirm Exit",
            "Stop all processes and exit?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.node:
                self.node.stop_all_launches()
            if self.bag_process is not None:
                self.on_stop_bag()
            event.accept()
        else:
            event.ignore()
