# SLAM Simulation ROS2 Workspace

ROS2 기반 SLAM 시뮬레이션 워크스페이스입니다. Gazebo Ignition을 사용한 로봇 시뮬레이션과 다양한 2D/3D SLAM 알고리즘을 GUI로 관리할 수 있습니다.

## 목차

- [프로젝트 구조](#프로젝트-구조)
- [지원 SLAM 알고리즘](#지원-slam-알고리즘)
- [의존성](#의존성)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [토픽 및 프레임](#토픽-및-프레임)

---

## 프로젝트 구조

```
slam_sim_ros2_ws/
├── src/
│   ├── Gazebo/                          # Gazebo 시뮬레이션 패키지
│   │   ├── launch/gazebo.launch.py      # Gazebo + RViz + Bridge 실행
│   │   ├── models/                      # 로봇 및 환경 모델
│   │   │   ├── pioneer2dx/              # Pioneer 2DX 로봇
│   │   │   ├── Depot/                   # 창고 환경
│   │   │   └── ...
│   │   └── worlds/my_world.sdf          # 시뮬레이션 월드
│   │
│   └── SLAM/
│       ├── SLAM_Manager/                # PyQt5 GUI 매니저
│       │   └── slam_manager/
│       │       ├── slam_manager_node.py # ROS2 노드
│       │       └── slam_manager_ui.py   # GUI 코드
│       │
│       ├── 2D_SLAM/
│       │   ├── SLAMBOX/                 # SLAM Toolbox 설정
│       │   ├── Cartographer/            # Google Cartographer 설정
│       │   ├── k_slam_ros2/             # K-SLAM 패키지
│       │   └── rtab_map/                # RTAB-Map 2D 설정
│       │
│       └── 3D_SLAM/
│           └── rtab_map_3d/             # RTAB-Map 3D 설정 (LiDAR + Camera)
│
└── docs/                                # 문서
```

---

## 지원 SLAM 알고리즘

### 2D SLAM

| 알고리즘 | 설명 | 센서 |
|----------|------|------|
| **SLAM Toolbox** | Graph-based 2D SLAM | 2D LiDAR |
| **Cartographer** | Google의 실시간 SLAM | 2D LiDAR, IMU (선택) |
| **K-SLAM** | 커스텀 SLAM | 2D LiDAR |
| **RTAB-Map 2D** | ICP 기반 2D SLAM | 2D/3D LiDAR |

### 3D SLAM

| 알고리즘 | 설명 | 센서 |
|----------|------|------|
| **RTAB-Map 3D** | Visual + LiDAR SLAM | 3D LiDAR + RGB Camera |

---

## 의존성

### 시스템 요구사항

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Gazebo Ignition Fortress

### ROS2 패키지 의존성

```bash
# 핵심 패키지
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-nav2-map-server

# Gazebo 연동
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image

# 시각화 및 도구
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-robot-steering \
    ros-humble-tf2-ros

# TF 및 메시지
sudo apt install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs
```

### Python 의존성

```bash
# PyQt5 (SLAM Manager GUI)
pip3 install PyQt5

# 또는 apt로 설치
sudo apt install python3-pyqt5
```

### 전체 의존성 설치 (한 번에)

```bash
# ROS2 패키지 전체 설치
sudo apt update && sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-nav2-map-server \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-rviz2 \
    ros-humble-rqt-robot-steering \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    python3-pyqt5
```

---

## 설치 방법

### 1. 워크스페이스 빌드

```bash
cd ~/slam_sim_ros2_ws

# 의존성 설치 (rosdep)
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install

# 환경 설정
source install/setup.bash
```

### 2. 환경 변수 설정 (선택)

`~/.bashrc`에 추가:

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# SLAM Simulation 워크스페이스
source ~/slam_sim_ros2_ws/install/setup.bash

# Gazebo 모델 경로
export IGN_GAZEBO_RESOURCE_PATH=~/slam_sim_ros2_ws/src/Gazebo/models:$IGN_GAZEBO_RESOURCE_PATH
```

---

## 사용 방법

### 1. Gazebo 시뮬레이션 실행

```bash
# 터미널 1: Gazebo + RViz + 로봇 제어
ros2 launch tm_gazebo gazebo.launch.py
```

시뮬레이션이 실행되면:
- Gazebo Ignition 창이 열립니다
- RViz2가 열립니다
- rqt_robot_steering으로 로봇을 제어할 수 있습니다

### 2. SLAM 실행

#### 방법 A: SLAM Manager GUI 사용 (권장)

```bash
# 터미널 2: SLAM Manager GUI
ros2 run slam_manager slam_manager
```

GUI에서 원하는 SLAM 알고리즘의 "Start Mapping" 버튼을 클릭합니다.

#### 방법 B: 개별 Launch 파일 사용

```bash
# SLAM Toolbox
ros2 launch slambox_config slam_toolbox_mapping.launch.py use_sim_time:=true rviz:=true

# Cartographer
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=true

# RTAB-Map 2D (LiDAR)
ros2 launch rtabmap_lidar rtabmap_lidar.launch.py use_sim_time:=true

# RTAB-Map 3D (LiDAR + Camera)
ros2 launch rtab_map_3d_config rtabmap_3d_mapping.launch.py use_sim_time:=true
```

### 3. 지도 저장

#### SLAM Toolbox

```bash
# 서비스 호출로 저장
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/path/to/map'}"
```

#### Cartographer

```bash
# 궤적 종료 후 저장
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/map.pbstream'}"
```

### 4. Localization 모드

이미 생성된 맵을 사용하여 위치 추정:

```bash
# SLAM Toolbox Localization
ros2 launch slambox_config slam_toolbox_localization.launch.py \
    use_sim_time:=true \
    map_file:=/path/to/saved_map

# RTAB-Map Localization
ros2 launch rtabmap_lidar rtabmap_lidar_localization.launch.py \
    use_sim_time:=true \
    database_path:=/path/to/rtabmap.db
```

---

## 토픽 및 프레임

### 주요 ROS2 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | sensor_msgs/LaserScan | 2D LiDAR 스캔 |
| `/scan/points` | sensor_msgs/PointCloud2 | 3D LiDAR 포인트클라우드 |
| `/odom` | nav_msgs/Odometry | 휠 오도메트리 |
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 |
| `/camera/color/image_raw` | sensor_msgs/Image | RGB 카메라 이미지 |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth 카메라 이미지 |
| `/map` | nav_msgs/OccupancyGrid | 생성된 2D 맵 |

### TF 프레임 구조

```
map
 └── odom
      └── base_link
           ├── lidar_link    (z: +0.09m)
           └── camera_link   (x: +0.25m, z: +0.10m)
```

---

## 문제 해결

### Gazebo가 실행되지 않을 때

```bash
# IGN_GAZEBO_RESOURCE_PATH 확인
echo $IGN_GAZEBO_RESOURCE_PATH

# 모델 경로 설정
export IGN_GAZEBO_RESOURCE_PATH=~/slam_sim_ros2_ws/src/Gazebo/models
```

### SLAM이 시작되지 않을 때

```bash
# 토픽 확인
ros2 topic list
ros2 topic echo /scan --once

# TF 확인
ros2 run tf2_tools view_frames
```

### 시뮬레이션 시간 동기화 문제

모든 노드에 `use_sim_time:=true` 파라미터를 전달해야 합니다:

```bash
ros2 launch ... use_sim_time:=true
```

---

## 라이선스

Apache-2.0

---

## 참고 자료

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer](https://google-cartographer-ros.readthedocs.io/)
- [RTAB-Map](http://introlab.github.io/rtabmap/)
- [Gazebo Ignition](https://gazebosim.org/)
