# 사용 가이드 (Usage Guide)

이 문서는 SLAM Simulation ROS2 워크스페이스의 설치 및 상세 사용법을 설명합니다.

---

## 목차

1. [설치](#설치)
2. [빠른 시작](#빠른-시작)
3. [Gazebo 시뮬레이션](#gazebo-시뮬레이션)
4. [SLAM Manager GUI](#slam-manager-gui)
5. [개별 SLAM 알고리즘 사용법](#개별-slam-알고리즘-사용법)
6. [ROS Bag 사용](#ros-bag-사용)
7. [디버깅](#디버깅)
8. [성능 최적화](#성능-최적화)
9. [FAQ](#자주-묻는-질문-faq)

---

## 설치

### 시스템 요구사항

| 항목 | 버전 |
|------|------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Gazebo | Ignition Fortress |
| Python | 3.10+ |

### 1단계: ROS2 Humble 설치

ROS2가 설치되어 있지 않다면:

```bash
# 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 저장소 추가
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install ros-humble-desktop
```

### 2단계: 의존성 패키지 설치

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# SLAM 패키지
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros

# Gazebo 연동 패키지
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image

# Navigation 패키지
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager

# 시각화 및 도구
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

# Python 패키지 (SLAM Manager GUI)
sudo apt install -y python3-pyqt5
```

또는 설치 스크립트 사용:

```bash
cd ~/slam_sim_ros2_ws
./scripts/install_dependencies.sh
```

### 3단계: 워크스페이스 빌드

```bash
cd ~/slam_sim_ros2_ws

# rosdep으로 누락된 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install

# 환경 설정
source install/setup.bash
```

### 4단계: 환경 변수 설정 (선택)

매번 source 명령어를 입력하지 않으려면 `~/.bashrc`에 추가:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/slam_sim_ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export IGN_GAZEBO_RESOURCE_PATH=~/slam_sim_ros2_ws/src/Gazebo/models:\$IGN_GAZEBO_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

### 설치 확인

```bash
# ROS2 확인
ros2 --version

# 패키지 확인
ros2 pkg list | grep slam

# Gazebo 확인
ign gazebo --version
```

---

## 빠른 시작

### 1단계: Gazebo 시뮬레이션 실행

```bash
# 터미널 1
source ~/slam_sim_ros2_ws/install/setup.bash
ros2 launch tm_gazebo gazebo.launch.py
```

### 2단계: SLAM Manager GUI 실행

```bash
# 터미널 2
source ~/slam_sim_ros2_ws/install/setup.bash
ros2 run slam_manager slam_manager
```

### 3단계: 로봇 제어 및 매핑

1. SLAM Manager GUI에서 "Use Sim Time" 체크
2. 원하는 SLAM 알고리즘의 "Start Mapping" 클릭
3. rqt_robot_steering으로 로봇을 이동시켜 맵 생성
4. "Save Map" 버튼으로 맵 저장

---

## Gazebo 시뮬레이션

### 기본 실행

```bash
ros2 launch tm_gazebo gazebo.launch.py
```

### Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `world` | `my_world.sdf` | 월드 파일 경로 |
| `odom_tf` | `true` | odom→base_link TF 발행 여부 |

### 예시

```bash
# 커스텀 월드 사용
ros2 launch tm_gazebo gazebo.launch.py world:=/path/to/custom.sdf

# odom TF 비활성화 (SLAM에서 제공하는 경우)
ros2 launch tm_gazebo gazebo.launch.py odom_tf:=false
```

### 제공되는 토픽

| 토픽 | 타입 | 방향 |
|------|------|------|
| `/cmd_vel` | Twist | ROS → Gazebo |
| `/odom` | Odometry | Gazebo → ROS |
| `/scan` | LaserScan | Gazebo → ROS |
| `/scan/points` | PointCloud2 | Gazebo → ROS |
| `/camera/color/image_raw` | Image | Gazebo → ROS |
| `/camera/depth/image_raw` | Image | Gazebo → ROS |
| `/clock` | Clock | Gazebo → ROS |

---

## SLAM Manager GUI

### 실행

```bash
ros2 run slam_manager slam_manager
```

### GUI 구성

```
┌─────────────────────────────────────────────────┐
│ SLAM Manager                                    │
├─────────────────────────────────────────────────┤
│ [Data Source] [SLAM Toolbox] [Cartographer] ... │
├─────────────────────────────────────────────────┤
│                                                 │
│  ☑ Use Sim Time    ☑ RViz                      │
│                                                 │
│  [Start Mapping]  [Stop]  [Save Map]           │
│                                                 │
│  Position: X: 0.000  Y: 0.000  Yaw: 0.0°       │
│                                                 │
├─────────────────────────────────────────────────┤
│ Log:                                            │
│ [12:00:00] SLAM Manager UI Ready               │
│ [12:00:05] Started: slam_toolbox_mapping       │
└─────────────────────────────────────────────────┘
```

### 탭 설명

| 탭 | 기능 |
|-----|------|
| **Data Source** | Livox 라이다 또는 ROS Bag 재생 |
| **SLAM Toolbox** | SLAM Toolbox 매핑/로컬라이제이션 |
| **Cartographer** | Google Cartographer 매핑/로컬라이제이션 |
| **K-SLAM** | K-SLAM 매핑 |
| **RTAB-Map 2D** | RTAB-Map LiDAR 기반 2D SLAM |
| **RTAB-Map 2D Camera** | RTAB-Map LiDAR+Camera SLAM |

### 공통 옵션

- **Use Sim Time**: 시뮬레이션 시간 사용 (Gazebo 사용 시 필수)
- **RViz**: 시각화 창 함께 실행

---

## 개별 SLAM 알고리즘 사용법

### SLAM Toolbox

#### 매핑 모드

```bash
ros2 launch slambox_config slam_toolbox_mapping.launch.py \
    use_sim_time:=true \
    rviz:=true
```

#### 로컬라이제이션 모드

```bash
ros2 launch slambox_config slam_toolbox_localization.launch.py \
    use_sim_time:=true \
    map_file:=/path/to/saved_map \
    rviz:=true
```

#### 맵 저장

```bash
# 서비스 호출
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '~/maps/my_map'}"
```

저장되는 파일:
- `my_map.posegraph` - 그래프 데이터
- `my_map.data` - 포즈 데이터

#### 파라미터 설정

파라미터 파일: `src/SLAM/2D_SLAM/SLAMBOX/config/mapper_params_online_async.yaml`

주요 파라미터:
```yaml
slam_toolbox:
  ros__parameters:
    # 맵 업데이트 설정
    resolution: 0.05          # 맵 해상도 (m/cell)
    max_laser_range: 20.0     # 최대 라이다 거리

    # 루프 클로저 설정
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse: 0.35
```

---

### Cartographer

#### 매핑 모드

```bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=true
```

#### 궤적 종료 및 맵 저장

```bash
# 1. 현재 궤적 종료
ros2 service call /finish_trajectory \
    cartographer_ros_msgs/srv/FinishTrajectory \
    "{trajectory_id: 0}"

# 2. 상태 저장 (.pbstream)
ros2 service call /write_state \
    cartographer_ros_msgs/srv/WriteState \
    "{filename: '~/maps/carto_map.pbstream'}"
```

#### 파라미터 설정

Lua 설정 파일: `src/SLAM/2D_SLAM/Cartographer/config/cartographer.lua`

주요 설정:
```lua
-- 트래킹 프레임
options = {
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  -- 센서 설정
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,

  -- 맵 해상도
  resolution = 0.05,
}
```

---

### RTAB-Map 2D (LiDAR)

#### 매핑 모드

```bash
ros2 launch rtabmap_lidar rtabmap_lidar.launch.py \
    use_sim_time:=true \
    rviz:=true
```

#### 로컬라이제이션 모드

```bash
ros2 launch rtabmap_lidar rtabmap_lidar_localization.launch.py \
    use_sim_time:=true \
    database_path:=/path/to/rtabmap.db \
    rviz:=true
```

#### 데이터베이스 위치

기본 저장 위치: `~/.ros/rtabmap.db`

---

### RTAB-Map 3D (LiDAR + Camera)

#### 매핑 모드

```bash
ros2 launch rtab_map_3d_config rtabmap_3d_mapping.launch.py \
    use_sim_time:=true \
    rviz:=true
```

#### 주요 파라미터

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `Icp/VoxelSize` | 0.1 | ICP 복셀 크기 |
| `Icp/MaxCorrespondenceDistance` | 0.5 | 최대 대응점 거리 |
| `Grid/CellSize` | 0.1 | 그리드 셀 크기 |
| `Grid/RangeMax` | 20.0 | 최대 센서 범위 |

---

## ROS Bag 사용

### Bag 녹화

```bash
# 모든 토픽 녹화
ros2 bag record -a

# 특정 토픽만 녹화
ros2 bag record /scan /odom /tf /tf_static

# 이름 지정하여 녹화
ros2 bag record -o my_dataset /scan /odom /camera/color/image_raw
```

### Bag 재생

#### 기본 재생

```bash
ros2 bag play /path/to/bag_folder
```

#### 옵션

```bash
# 클럭 토픽 발행 (use_sim_time 사용 시 필수)
ros2 bag play --clock /path/to/bag

# 반복 재생
ros2 bag play --loop /path/to/bag

# 속도 조절
ros2 bag play --rate 0.5 /path/to/bag   # 0.5배속
ros2 bag play --rate 2.0 /path/to/bag   # 2배속
```

### SLAM Manager에서 Bag 사용

1. "Data Source" 탭 선택
2. "Browse" 버튼으로 bag 파일 선택
3. 옵션 설정:
   - Loop: 반복 재생
   - Rate: 재생 속도
   - Clock: 클럭 토픽 발행
4. "Play" 버튼 클릭

---

## 디버깅

### 토픽 확인

```bash
# 토픽 목록
ros2 topic list

# 토픽 데이터 확인
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### TF 확인

```bash
# TF 트리 시각화
ros2 run tf2_tools view_frames

# 특정 프레임 간 변환 확인
ros2 run tf2_ros tf2_echo map base_link
```

### 노드 확인

```bash
# 실행 중인 노드
ros2 node list

# 노드 정보
ros2 node info /slam_toolbox
```

### 파라미터 확인

```bash
# 노드 파라미터 목록
ros2 param list /slam_toolbox

# 파라미터 값 확인
ros2 param get /slam_toolbox resolution
```

---

## 성능 최적화

### SLAM Toolbox

```yaml
# 빠른 매핑 (정확도↓, 속도↑)
slam_toolbox:
  ros__parameters:
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 3
```

### Cartographer

```lua
-- 빠른 매핑
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
```

### RTAB-Map

```yaml
# 메모리 절약
rtabmap:
  ros__parameters:
    Mem/IncrementalMemory: "true"
    Mem/STMSize: "30"
    RGBD/OptimizeMaxError: "3.0"
```

---

## 자주 묻는 질문 (FAQ)

### Q: 맵이 생성되지 않습니다

**A:** 다음을 확인하세요:
1. `/scan` 토픽이 발행되고 있는지
2. TF가 올바르게 설정되어 있는지 (`map` → `odom` → `base_link` → `lidar_link`)
3. `use_sim_time` 파라미터가 올바르게 설정되어 있는지

### Q: 로봇이 움직이지 않습니다

**A:** `/cmd_vel` 토픽 확인:
```bash
ros2 topic echo /cmd_vel
```
rqt_robot_steering이 실행 중인지 확인하세요.

### Q: 시뮬레이션과 실제 시간이 맞지 않습니다

**A:** 모든 노드에 `use_sim_time:=true`를 전달하고, `/clock` 토픽이 발행되는지 확인하세요.

### Q: SLAM Manager가 실행되지 않습니다

**A:** PyQt5 설치 확인:
```bash
python3 -c "from PyQt5 import QtWidgets; print('OK')"
```

---

## 참고 명령어 모음

```bash
# 환경 설정
source ~/slam_sim_ros2_ws/install/setup.bash

# Gazebo 실행
ros2 launch tm_gazebo gazebo.launch.py

# SLAM Manager
ros2 run slam_manager slam_manager

# SLAM Toolbox 매핑
ros2 launch slambox_config slam_toolbox_mapping.launch.py use_sim_time:=true

# Cartographer 매핑
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=true

# RTAB-Map 2D 매핑
ros2 launch rtabmap_lidar rtabmap_lidar.launch.py use_sim_time:=true

# RTAB-Map 3D 매핑
ros2 launch rtab_map_3d_config rtabmap_3d_mapping.launch.py use_sim_time:=true

# 토픽 확인
ros2 topic list
ros2 topic echo /scan --once

# TF 확인
ros2 run tf2_tools view_frames
```
