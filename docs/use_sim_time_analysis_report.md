# use_sim_time 설정 통일성 분석 보고서

**작성일:** 2026-01-29
**대상:** slam_sim_ros2_ws 워크스페이스

---

## 1. 개요

ROS2에서 `use_sim_time` 파라미터는 시뮬레이션 환경(Gazebo)과 실제 로봇 환경을 구분하는 핵심 설정입니다.
- `true`: Gazebo `/clock` 토픽 사용 (시뮬레이션 시간)
- `false`: 시스템 시간 사용 (실제 환경)

모든 노드가 동일한 시간 소스를 사용해야 TF, 센서 데이터 동기화가 정상 동작합니다.

---

## 2. 분석 결과 요약

| 상태 | 패키지 수 | 설명 |
|------|-----------|------|
| :white_check_mark: 정상 | 4 | 모든 노드에 use_sim_time 설정됨 |
| :warning: 부분 누락 | 1 | 일부 노드에 설정 누락 |
| :x: 기본값 불일치 | 5 | 패키지간 기본값이 다름 |

---

## 3. 패키지별 상세 분석

### 3.1 Gazebo 패키지 :warning: **문제 발견**

**파일:** `src/Gazebo/launch/gazebo.launch.py`

| 노드/프로세스 | use_sim_time | 상태 |
|---------------|--------------|------|
| rviz2 | `True` (하드코딩) | :white_check_mark: |
| ros_gz_bridge | **미설정** | :x: |
| static_transform_publisher (lidar) | **미설정** | :x: |
| static_transform_publisher (camera) | **미설정** | :x: |
| rqt_robot_steering | **미설정** | :x: |

**파일:** `src/Gazebo/scripts/odom_to_tf.py`
- use_sim_time 파라미터 처리 없음 :x:

**영향:**
- TF 타임스탬프 불일치로 인한 "TF_OLD_DATA" 경고 발생 가능
- 센서 데이터와 TF 동기화 문제

---

### 3.2 SLAM 패키지 기본값 비교

| 패키지 | 기본값 | 상태 |
|--------|--------|------|
| **Cartographer** | `false` | 실제환경 기준 |
| **SLAMBOX (SLAM Toolbox)** | `false` | 실제환경 기준 |
| **SLAMBOX (Hector)** | `false` | 실제환경 기준 |
| **rtab_map (2D)** | `false` | 실제환경 기준 |
| **rtab_map_3d (3D)** | `true` | 시뮬레이션 기준 :warning: |
| **k_slam_ros2** | `false` | 실제환경 기준 |

**불일치:** rtab_map_3d만 기본값이 `true`로 설정됨

---

### 3.3 개별 SLAM Launch 파일 상세

#### Cartographer :white_check_mark:
```
cartographer_node        → use_sim_time: LaunchConfiguration
cartographer_occupancy_grid_node → use_sim_time: LaunchConfiguration
rviz2                    → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨

#### SLAM Toolbox (SLAMBOX) :white_check_mark:
```
slam_toolbox_mapping.launch.py:
  async_slam_toolbox_node → use_sim_time: LaunchConfiguration
  rviz2                   → use_sim_time: LaunchConfiguration

slam_toolbox_localization.launch.py:
  localization_slam_toolbox_node → use_sim_time: LaunchConfiguration
  rviz2                          → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨

#### Hector SLAM (SLAMBOX) :white_check_mark:
```
hector_mapping.launch.py:
  hector_mapping → use_sim_time: LaunchConfiguration
  rviz2          → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨

#### RTAB-Map 2D :white_check_mark:
```
rtabmap_lidar.launch.py:
  icp_odometry → use_sim_time: LaunchConfiguration
  rtabmap      → use_sim_time: LaunchConfiguration
  rviz2        → use_sim_time: LaunchConfiguration

rtabmap_lidar_camera.launch.py:
  rgbd_odometry → use_sim_time: LaunchConfiguration
  rtabmap       → use_sim_time: LaunchConfiguration
  rviz2         → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨

#### RTAB-Map 3D :white_check_mark:
```
rtabmap_3d_mapping.launch.py:
  rgbd_odometry → use_sim_time: LaunchConfiguration
  rtabmap       → use_sim_time: LaunchConfiguration
  rviz2         → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨 (단, 기본값이 'true')

#### K-SLAM :white_check_mark:
```
k_slam.launch.py:
  k_slam_node → use_sim_time: LaunchConfiguration
  rviz2       → use_sim_time: LaunchConfiguration
```
**평가:** 모든 노드에 일관되게 적용됨 (C++ 코드에서도 처리)

---

### 3.4 SLAM Manager :white_check_mark:

**파일:** `src/SLAM/SLAM_Manager/slam_manager/`

| 기능 | use_sim_time 처리 |
|------|-------------------|
| ROS Bag 재생 시작 | `set_use_sim_time(True)` 호출 |
| ROS Bag 정지 | `set_use_sim_time(False)` 호출 |
| SLAM 알고리즘 실행 | `use_sim_time:=true` 인자 조건부 추가 |

**평가:** ROS Bag 재생 여부에 따라 동적으로 관리 - 적절함

---

## 4. 발견된 문제점

### 4.1 심각 (Critical) :x:

| # | 문제 | 위치 | 영향 |
|---|------|------|------|
| 1 | Gazebo bridge 노드에 use_sim_time 미설정 | gazebo.launch.py:43-59 | 센서 데이터 타임스탬프 불일치 |
| 2 | Static TF publishers에 use_sim_time 미설정 | gazebo.launch.py:70-83 | TF 타임스탬프 불일치 |
| 3 | odom_to_tf.py에 use_sim_time 미설정 | scripts/odom_to_tf.py | TF 타임스탬프 불일치 |

### 4.2 경고 (Warning) :warning:

| # | 문제 | 위치 | 영향 |
|---|------|------|------|
| 1 | rtab_map_3d 기본값이 다른 패키지와 불일치 | rtabmap_3d_*.launch.py | 혼란 유발 |
| 2 | rqt_robot_steering에 use_sim_time 미설정 | gazebo.launch.py:96-101 | 경미 (UI 도구) |

### 4.3 참고 (Info)

| # | 내용 |
|---|------|
| 1 | 업스트림 rtabmap_ros 패키지는 혼합된 기본값 사용 (수정 불필요) |
| 2 | YAML 파라미터 파일들은 하드코딩된 `True` 값 사용 (데모용) |

---

## 5. 권장 수정사항

### 5.1 gazebo.launch.py 수정

```python
# 수정 전
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[...],
    output='screen'
)

# 수정 후
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[...],
    parameters=[{'use_sim_time': True}],
    output='screen'
)
```

```python
# Static TF 노드들에도 추가
static_tf_lidar = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'lidar_link'],
    parameters=[{'use_sim_time': True}],
    output='screen'
)
```

### 5.2 odom_to_tf.py 수정

```python
class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        # use_sim_time 파라미터 선언
        self.declare_parameter('use_sim_time', True)
        ...
```

### 5.3 기본값 통일 (선택사항)

모든 SLAM 패키지의 기본값을 동일하게 설정:
- 권장: `default_value='false'` (실제 로봇 환경 기준)
- 시뮬레이션 실행 시 항상 `use_sim_time:=true` 명시

---

## 6. 검증 방법

수정 후 다음 명령으로 확인:

```bash
# 1. Gazebo 실행
ros2 launch tm_gazebo gazebo.launch.py

# 2. 모든 노드의 use_sim_time 파라미터 확인
ros2 param list | xargs -I {} ros2 param get {} use_sim_time

# 3. TF 타임스탬프 확인
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 7. 결론

| 항목 | 현재 상태 |
|------|-----------|
| SLAM 패키지들 | :white_check_mark: 내부적으로 일관됨 |
| Gazebo 패키지 | :x: 수정 필요 |
| SLAM Manager | :white_check_mark: 적절히 관리됨 |
| 패키지간 기본값 | :warning: 통일 권장 |

**우선순위:**
1. **즉시 수정:** Gazebo launch 파일 및 odom_to_tf.py
2. **권장 수정:** 기본값 통일 (선택사항)
