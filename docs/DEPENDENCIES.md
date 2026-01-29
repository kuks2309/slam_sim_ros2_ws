# 의존성 (Dependencies)

이 문서는 SLAM Simulation ROS2 워크스페이스의 모든 의존성을 정리합니다.

---

## 시스템 요구사항

| 항목 | 버전 | 비고 |
|------|------|------|
| Ubuntu | 22.04 LTS (Jammy) | 필수 |
| ROS2 | Humble Hawksbill | 필수 |
| Gazebo | Ignition Fortress | 필수 |
| Python | 3.10+ | Ubuntu 22.04 기본 |
| CMake | 3.16+ | 빌드 도구 |

---

## ROS2 패키지 의존성

### 핵심 SLAM 패키지

```bash
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-odom \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-viz \
    ros-humble-rtabmap-util \
    ros-humble-rtabmap-launch
```

| 패키지 | 용도 |
|--------|------|
| `slam-toolbox` | SLAM Toolbox 2D SLAM |
| `cartographer` | Google Cartographer 코어 |
| `cartographer-ros` | Cartographer ROS2 인터페이스 |
| `rtabmap` | RTAB-Map 코어 라이브러리 |
| `rtabmap-ros` | RTAB-Map ROS2 메타패키지 |
| `rtabmap-odom` | ICP/Visual Odometry |
| `rtabmap-slam` | SLAM 노드 |

### Navigation 패키지

```bash
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-amcl
```

| 패키지 | 용도 |
|--------|------|
| `nav2-map-server` | 맵 저장/로드 |
| `nav2-lifecycle-manager` | 노드 생명주기 관리 |
| `nav2-amcl` | AMCL Localization |

### Gazebo 연동 패키지

```bash
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image
```

| 패키지 | 용도 |
|--------|------|
| `ros-gz-sim` | Gazebo Ignition 시뮬레이터 |
| `ros-gz-bridge` | ROS2 ↔ Gazebo 메시지 브릿지 |
| `ros-gz-image` | 이미지 토픽 브릿지 |

### 시각화 및 도구

```bash
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph
```

| 패키지 | 용도 |
|--------|------|
| `rviz2` | 3D 시각화 도구 |
| `rqt-robot-steering` | 로봇 수동 제어 GUI |
| `rqt-tf-tree` | TF 트리 시각화 |
| `rqt-graph` | 노드/토픽 그래프 |

### TF 및 변환

```bash
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs
```

### 메시지 타입

```bash
sudo apt install -y \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-visualization-msgs
```

---

## Python 의존성

### PyQt5 (SLAM Manager GUI)

```bash
# pip 설치 (권장)
pip3 install PyQt5

# 또는 apt 설치
sudo apt install -y python3-pyqt5 python3-pyqt5.qtwebengine
```

### 기타 Python 패키지

```bash
pip3 install \
    numpy \
    opencv-python \
    pyyaml
```

---

## 패키지별 의존성 상세

### SLAM Manager (`slam_manager`)

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<exec_depend>python3-pyqt5</exec_depend>
```

### Gazebo 패키지 (`tm_gazebo`)

```xml
<!-- package.xml -->
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

### Cartographer 설정 (`cartographer_slam`)

```xml
<!-- package.xml -->
<depend>cartographer_ros</depend>
<depend>nav2_map_server</depend>
```

### SLAM Toolbox 설정 (`slambox_config`)

```xml
<!-- package.xml -->
<exec_depend>slam_toolbox</exec_depend>
<exec_depend>nav2_map_server</exec_depend>
```

### RTAB-Map 3D 설정 (`rtab_map_3d_config`)

```xml
<!-- package.xml -->
<exec_depend>rtabmap_launch</exec_depend>
<exec_depend>rtabmap_odom</exec_depend>
<exec_depend>rtabmap_slam</exec_depend>
<exec_depend>rviz2</exec_depend>
```

---

## 전체 의존성 설치 스크립트

아래 스크립트를 실행하여 모든 의존성을 한 번에 설치할 수 있습니다:

```bash
#!/bin/bash
# install_dependencies.sh

set -e

echo "=== ROS2 Humble 의존성 설치 ==="

# ROS2 저장소 업데이트
sudo apt update

# SLAM 패키지
echo "Installing SLAM packages..."
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros

# Navigation 패키지
echo "Installing Navigation packages..."
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager

# Gazebo 연동
echo "Installing Gazebo packages..."
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image

# 시각화 도구
echo "Installing visualization tools..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree

# TF 패키지
echo "Installing TF packages..."
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

# 메시지 패키지
echo "Installing message packages..."
sudo apt install -y \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs

# Python 패키지
echo "Installing Python packages..."
sudo apt install -y python3-pyqt5
pip3 install numpy opencv-python pyyaml

echo "=== 설치 완료 ==="
```

### 사용법

```bash
# 스크립트 저장 및 실행
chmod +x install_dependencies.sh
./install_dependencies.sh
```

---

## rosdep을 사용한 의존성 설치

워크스페이스 내 모든 패키지의 의존성을 자동으로 설치:

```bash
cd ~/slam_sim_ros2_ws

# rosdep 초기화 (최초 1회)
sudo rosdep init
rosdep update

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y
```

---

## 의존성 확인

### 설치된 패키지 확인

```bash
# SLAM Toolbox 확인
ros2 pkg list | grep slam

# Cartographer 확인
ros2 pkg list | grep cartographer

# RTAB-Map 확인
ros2 pkg list | grep rtabmap

# Gazebo 브릿지 확인
ros2 pkg list | grep gz
```

### 버전 확인

```bash
# ROS2 버전
printenv ROS_DISTRO

# 패키지 버전 확인 예시
apt show ros-humble-slam-toolbox
```

---

## 알려진 문제

### 1. Cartographer 빌드 오류

Cartographer를 소스에서 빌드할 경우 protobuf 버전 충돌이 발생할 수 있습니다:

```bash
# 해결: apt로 설치된 버전 사용
sudo apt install ros-humble-cartographer-ros
```

### 2. RTAB-Map CUDA 지원

GPU 가속을 사용하려면 CUDA 버전의 RTAB-Map을 소스에서 빌드해야 합니다:

```bash
# CUDA 지원 빌드 (선택사항)
cd ~/slam_sim_ros2_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone https://github.com/introlab/rtabmap_ros.git -b humble-devel
cd ..
colcon build --cmake-args -DWITH_CUDA=ON
```

### 3. PyQt5 import 오류

```bash
# 해결
pip3 install --upgrade PyQt5
# 또는
sudo apt install --reinstall python3-pyqt5
```

---

## 참고

- [ROS2 Humble 패키지 목록](https://index.ros.org/packages/page/1/time/#humble)
- [rosdep 문서](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
