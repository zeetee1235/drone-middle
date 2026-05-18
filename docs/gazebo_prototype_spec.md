# GPS 비의존 스프린트 드론 — Gazebo 프로토타입 개발 명세서

**버전**: v0.2  
**작성일**: 2026-05-16  
**대상**: 팀 내부 개발 참조용

---

## 목차

1. [개발 환경 스택](#1-개발-환경-스택)
2. [언어 및 구현 전략](#2-언어-및-구현-전략)
3. [Gazebo 시뮬레이션 월드 명세](#3-gazebo-시뮬레이션-월드-명세)
4. [소프트웨어 아키텍처 개요](#4-소프트웨어-아키텍처-개요)
5. [모듈별 구현 명세](#5-모듈별-구현-명세)
   - 5.1 Grid Detector Node
   - 5.2 ArUco Detector + Marker Confidence Filter Node
   - 5.3 Visual Odometry Node (Optical Flow + Grid Localizer)
   - 5.4 Sprint Planner Node
   - 5.5 Perception-Aware Flight Controller Node
   - 5.6 Mission Manager Node
   - 5.7 Vertiport Detector Node
   - 5.8 Python 개발 도구
6. [ROS2 토픽 / 서비스 인터페이스 명세](#6-ros2-토픽--서비스-인터페이스-명세)
7. [개발 단계 및 마일스톤](#7-개발-단계-및-마일스톤)
8. [테스트 계획](#8-테스트-계획)
9. [리스크 및 대응](#9-리스크-및-대응)

---

## 1. 개발 환경 스택

### 1.1 기본 구성

| 항목 | 선택 | 비고 |
|------|------|------|
| OS | **Ubuntu 24.04 LTS (Noble)** | — |
| ROS2 | **Jazzy Jalisco** | Ubuntu 24.04 공식 LTS |
| Gazebo | **Gazebo Harmonic** | Jazzy 공식 연동 버전 |
| PX4 | PX4-Autopilot main | Ubuntu 24.04 지원, 초기 트윅 예상 |
| 실시간 노드 언어 | **C++ 17** | 비전·제어·미션 전 루프 |
| 개발 도구 언어 | **Python 3.12** | 로그 분석·시각화·튜닝 스크립트 |
| 영상 처리 | OpenCV 4.x (C++ API) | — |
| 빌드 | colcon + CMake | — |
| 시뮬 브릿지 | micro-XRCE-DDS Agent | PX4 v1.14+ |

> **주의**: Ubuntu 24.04 + PX4 조합은 공식 권장 스택(22.04)이 아니므로, Phase 0에서 빌드 환경 구성에 별도 시간을 확보한다. PX4 ubuntu.sh 스크립트 수정, break-system-packages 옵션, NuttX 의존성 버전 조정 등의 트윅이 필요할 수 있다.

### 1.2 필수 패키지

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# micro-XRCE-DDS 에이전트
sudo snap install micro-xrce-dds-agent

# OpenCV + ROS2 브릿지
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-image-transport

# Gazebo Harmonic + ROS2 브릿지
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-image

# PX4 메시지 인터페이스
sudo apt install ros-jazzy-px4-msgs

# 빌드 도구
sudo apt install python3-colcon-common-extensions
sudo apt install ros-jazzy-ament-cmake

# Python 도구 (비실시간)
pip3 install matplotlib pandas numpy scipy --break-system-packages
```

### 1.3 작업 디렉토리 구조

```
sprint_drone_ws/
├── src/
│   ├── camera_node/              # [C++] 카메라 프레임 수신·퍼블리시
│   ├── grid_detector/            # [C++] 격자선·교차점 검출
│   ├── aruco_tracker/            # [C++] ArUco 검출 + Confidence Filter
│   ├── visual_odometry/          # [C++] Optical Flow + Grid Localizer
│   ├── sprint_planner/           # [C++] Minimum-Time 경로 계획
│   ├── mission_manager/          # [C++] 전체 미션 상태 머신
│   ├── px4_offboard_control/     # [C++] PX4 Offboard 제어 + Anti-Sway
│   ├── vertiport_detector/       # [C++] 버티포트 검출 + 비전 서보
│   ├── sprint_drone_msgs/        # 커스텀 메시지 정의 (공통)
│   └── gazebo_worlds/            # 시뮬 월드 SDF 파일
├── tools/
│   ├── log_analyzer.py           # [Python] 비행 로그 분석
│   ├── plot_mission.py           # [Python] 미션 경로 시각화
│   ├── parameter_sweep.py        # [Python] 파라미터 튜닝 스크립트
│   ├── spawn_markers.py          # [Python] 시뮬 마커 배치 스크립트
│   └── dataset_labeling.py       # [Python] 인식 데이터 레이블링
├── config/
│   ├── params.yaml               # 전역 파라미터
│   └── px4_params/               # PX4 파라미터 세트
└── launch/
    ├── sim_full.launch.py        # 전체 스택 실행
    ├── sim_perception.launch.py  # 인식 모듈만 실행
    └── sim_planner.launch.py     # 계획 모듈만 실행
```

---

## 2. 언어 및 구현 전략

### 2.1 핵심 원칙

본 시스템은 2m 저고도 스프린트, 하향 영상 인식, PX4 Offboard 제어가 동시에 실행되는 구조다. 카메라 프레임 수신부터 setpoint 송신까지의 실시간 루프에서 프레임 드롭이나 처리 지연이 발생하면 마커 인식 실패, 제동 지연, 경로 이탈로 직결된다.

따라서 실시간성이 요구되는 모든 ROS2 노드는 **C++** 로 구현하고, Python은 비실시간 개발 도구에 한정한다.

> **제출 문서 표현**:  
> 실시간성이 요구되는 비전 인식, 위치 추정, 미션 상태 전환, PX4 Offboard 제어 노드는 C++ 기반 ROS2 노드로 구현한다. Python은 초기 알고리즘 검증, 로그 분석, 시각화, 파라미터 튜닝 등 비실시간 개발 도구에 한정하여 사용한다.

### 2.2 C++ 실시간 노드 — 역할 및 이유

| 노드 | 역할 | C++ 이유 |
|------|------|----------|
| `camera_node` | 카메라 프레임 수신 및 퍼블리시 | 프레임 드롭 시 인식 지연 |
| `grid_detector_node` | 격자선·교차점 검출 | 위치 보정 주기 유지 |
| `aruco_detector_node` | ArUco 검출 + Confidence Filter | 마커 후보 즉시 감지 필요 |
| `visual_odometry_node` | Optical Flow + Grid Localizer 융합 | 위치 추정 연속성 |
| `sprint_planner_node` | Minimum-Time 경로 계획 | 교차점마다 재계획 |
| `mission_manager_node` | 미션 상태 머신 | 상태 전환 지연 최소화 |
| `px4_offboard_control_node` | setpoint 생성 + Anti-Sway + PX4 송신 | 20Hz Offboard 유지 필수 |
| `vertiport_detector_node` | 버티포트 검출 + 비전 서보 | 착륙 서보 루프 |

### 2.3 Python 비실시간 도구

| 스크립트 | 역할 |
|---------|------|
| `log_analyzer.py` | rosbag2 로그 파싱 및 자동 리포트 생성 |
| `plot_mission.py` | 격자 탐색 경로, 마커 위치, 속도 시계열 시각화 |
| `parameter_sweep.py` | 속도/가속도/Confidence 파라미터 배치 실험 |
| `spawn_markers.py` | Gazebo 내 ArUco 마커 동적 배치 |
| `dataset_labeling.py` | 인식 알고리즘 개선용 데이터 레이블링 |

### 2.4 C++ 노드 공통 구현 지침

```cpp
// 모든 실시간 노드 공통 패턴 예시
class GridDetectorNode : public rclcpp::Node {
public:
    GridDetectorNode() : Node("grid_detector") {
        // 실시간 센서 토픽: BEST_EFFORT + depth 1
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/down/image_raw", qos,
            std::bind(&GridDetectorNode::imageCallback, this, _1));

        pub_lines_ = create_publisher<sprint_drone_msgs::msg::GridLines>(
            "/grid/lines", qos);
        // 타이머 기반이 아닌 콜백 기반 처리 (카메라 FPS 추종)
    }
private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};
```

```cmake
# 각 C++ 패키지 CMakeLists.txt 공통 구조
cmake_minimum_required(VERSION 3.16)
project(grid_detector)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sprint_drone_msgs REQUIRED)

add_executable(grid_detector_node src/grid_detector_node.cpp)
ament_target_dependencies(grid_detector_node
    rclcpp sensor_msgs OpenCV cv_bridge sprint_drone_msgs)
install(TARGETS grid_detector_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

---

## 3. Gazebo 시뮬레이션 월드 명세

### 3.1 경기장 레이아웃

```
안전구역       : 32m × 23m
비행/미션 구역 : 24m × 15m
격자 간격      : 3m × 3m
구획선 폭      : 0.10m
격자 높이      : 지면 (z = 0)
비행 고도      : z = 2m (규정)
버티포트       : 지름 3m 원형, 비행구역 외부 좌하단
ArUco 마커     : 격자 꼭짓점 및 교차점 중 임의 4개 배치
조명           : 균일 조명 (실내 환경 가정)
```

### 3.2 격자선 텍스처 명세

| 항목 | 값 |
|------|-----|
| 격자 간격 | 3m |
| 선 색상 | 흰색 (#FFFFFF) 또는 밝은 회색 |
| 배경 색상 | 어두운 회색 (#303030) |
| 선 두께 | 10cm |
| 교차점 마커 | 교차점 중앙 10×10cm 흰색 사각형 |

### 3.3 ArUco 마커 배치

```python
# tools/spawn_markers.py
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.5

MARKER_POSITIONS = [          # 격자 교차점 기준 예시
    {"id": 3, "x": 10.0, "y": 10.0},
    {"id": 8, "x": 19.0, "y": 7.0},
    {"id": 1, "x": 25.0, "y": 16.0},
    {"id": 5, "x": 13.0, "y": 16.0},
]
# 실제 대회 위치 비공개 → 랜덤 배치로 교체하며 테스트
```

### 3.4 버티포트 모델

주황색 (#CC7A00 계열) 원형 착륙 패드, 지름 3m, 지면에 수평 배치.

### 3.5 하향 카메라 플러그인 명세

| 항목 | 값 |
|------|-----|
| 해상도 | 640 × 480 (초기), 1280 × 720 (여유 시) |
| FPS | 60 |
| 화각 (FOV) | 110도 (수평) |
| 장착 위치 | 기체 중심 하부, 정방향 고정 |
| ROS2 토픽 | `/drone/camera/down/image_raw` |

---

## 4. 소프트웨어 아키텍처 개요

### 4.1 전체 데이터 흐름

```
[Gazebo 카메라] /drone/camera/down/image_raw
  │
  ├──► [grid_detector_node]          C++  → /grid/lines
  │                                        /grid/intersections
  │                                        /grid/heading_angle
  ├──► [aruco_detector_node]         C++  → /aruco/detections
  │                                        /markers/candidates
  │                                        /markers/confirmed
  └──► [visual_odometry_node]        C++  (Optical Flow 계산)

[Gazebo IMU] /drone/imu/data
  └──► [px4_offboard_control_node]   C++  → /attitude/rpy
                                           /sway/metric

[visual_odometry_node]               C++  → /optical_flow/velocity
  ← /grid/intersections                    /localization/pose_grid
  ← /aruco/detections                      /localization/pose_home
  ← /attitude/rpy

[mission_manager_node]               C++  → /mission/state
  ← /localization/pose_grid                /mission/target_node
  ← /markers/confirmed
  ← /fmu/out/battery_status

[sprint_planner_node]                C++  → /planner/velocity_profile
  ← /mission/target_node                   /planner/waypoints
  ← /localization/pose_grid

[px4_offboard_control_node]          C++  → /fmu/in/trajectory_setpoint (20Hz)
  ← /planner/velocity_profile              /fmu/in/offboard_control_mode
  ← /sway/metric
  ← /vertiport/center_error

[vertiport_detector_node]            C++  → /vertiport/center_error
  ← /drone/camera/down/image_raw

[Python Tools — 비실시간]
  log_analyzer.py      ← rosbag2 로그
  plot_mission.py      ← /mission/state, /localization/*
  parameter_sweep.py   → config/params.yaml 배치 수정
  spawn_markers.py     → Gazebo 마커 배치
```

### 4.2 PX4 ↔ ROS2 Jazzy 통신 브릿지

```bash
MicroXRCEAgent udp4 -p 8888
```

| PX4 uORB | ROS2 토픽 | 방향 |
|----------|----------|------|
| `vehicle_attitude` | `/fmu/out/vehicle_attitude` | PX4 → ROS2 |
| `vehicle_local_position` | `/fmu/out/vehicle_local_position` | PX4 → ROS2 |
| `battery_status` | `/fmu/out/battery_status` | PX4 → ROS2 |
| `offboard_control_mode` | `/fmu/in/offboard_control_mode` | ROS2 → PX4 |
| `trajectory_setpoint` | `/fmu/in/trajectory_setpoint` | ROS2 → PX4 |
| `vehicle_command` | `/fmu/in/vehicle_command` | ROS2 → PX4 |

---

## 5. 모듈별 구현 명세

---

### 5.1 Grid Detector Node

**언어**: C++  
**역할**: 하향 카메라 영상에서 격자선과 교차점을 검출한다.

#### 입출력

| | 내용 |
|--|------|
| 입력 | `/drone/camera/down/image_raw` |
| 출력 | `/grid/lines`, `/grid/intersections`, `/grid/heading_angle` |

#### 알고리즘

```
1. cv_bridge → cv::Mat 변환
2. 그레이스케일 + 가우시안 블러 (5×5)
3. Canny 엣지 (threshold 50/150)
4. HoughLinesP (rho=1, theta=π/180, threshold=80, minLen=30, maxGap=10)
5. 수평/수직 방향 그룹핑 (각도 ±10도 기준)
6. 선분 병합 및 격자 간격 검증
7. 교차점 계산 (수평선 × 수직선)
8. 신뢰도 필터 후 퍼블리시
```

#### 파라미터 (params.yaml)

```yaml
grid_detector:
  hough_threshold: 80
  min_line_length: 30
  max_line_gap: 10
  angle_tolerance_deg: 10.0
  expected_grid_pixel_width: 110   # 2m 고도, 640px, 110도 FOV 기준
  min_intersections_valid: 2
  blur_kernel_size: 5
```

#### 커스텀 메시지

```
# GridLines.msg
std_msgs/Header header
geometry_msgs/Point[] line_start_pts
geometry_msgs/Point[] line_end_pts
float32[] angles_deg
float32 dominant_heading_deg
float32 confidence

# GridIntersections.msg
std_msgs/Header header
geometry_msgs/Point[] pixel_positions
geometry_msgs/Point[] world_positions
float32 confidence
```

---

### 5.2 ArUco Detector + Marker Confidence Filter Node

**언어**: C++  
**역할**: ArUco 마커를 검출하고, 누적 관측 기반 신뢰도를 계산해 확정 발견을 판단한다.

#### 입출력

| | 내용 |
|--|------|
| 입력 | `/drone/camera/down/image_raw`, `/sway/metric` |
| 출력 | `/aruco/detections`, `/markers/candidates`, `/markers/confirmed` |

#### ArUco 검출 (C++)

```cpp
auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
auto params = cv::aruco::DetectorParameters::create();
params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

std::vector<int> ids;
std::vector<std::vector<cv::Point2f>> corners;
cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
```

#### Marker Confidence Filter (C++)

상태 전환: `Not Detected → Candidate → Tracking → Confirmed → Visited`

```cpp
struct MarkerState {
    int id;
    float confidence = 0.0f;
    cv::Point2f last_pixel_pos;
    float last_size_px;
    std::string status;
};

void updateConfidence(MarkerState& state,
                      const DetectedMarker& det,
                      float sway_metric)
{
    float weight = (sway_metric > SWAY_THRESHOLD) ? 0.2f : 1.0f;
    float pos_delta = cv::norm(det.center - state.last_pixel_pos);
    float size_delta = std::abs(det.size_px - state.last_size_px)
                       / state.last_size_px;
    bool stable = (pos_delta < POS_STABLE_PX) && (size_delta < SIZE_STABLE_RATIO);

    if (stable)
        state.confidence = std::min(1.0f, state.confidence + weight * CONF_INCREMENT);
    else
        state.confidence *= CONF_DECAY;
}

constexpr float CONFIRM_THRESHOLD  = 0.75f;
constexpr float SWAY_THRESHOLD     = 0.3f;   // rad/s (실험으로 결정)
constexpr float CONF_INCREMENT     = 0.15f;
constexpr float CONF_DECAY         = 0.8f;
constexpr float POS_STABLE_PX      = 10.0f;
constexpr float SIZE_STABLE_RATIO  = 0.2f;
```

---

### 5.3 Visual Odometry Node (Optical Flow + Grid Localizer)

**언어**: C++  
**역할**: Optical Flow와 격자선 관측, ArUco 마커를 융합하여 격자 좌표계 및 Home 좌표계 위치를 추정한다.

#### 입출력

| | 내용 |
|--|------|
| 입력 | `/drone/camera/down/image_raw`, `/grid/intersections`, `/aruco/detections`, `/fmu/out/vehicle_local_position` |
| 출력 | `/optical_flow/velocity`, `/localization/pose_grid`, `/localization/pose_home`, `/localization/drift_metric` |

#### Optical Flow (C++)

```cpp
cv::TermCriteria criteria(
    cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);

std::vector<cv::Point2f> next_pts;
std::vector<uchar> status;
cv::calcOpticalFlowPyrLK(
    prev_gray, curr_gray, prev_pts, next_pts,
    status, cv::noArray(), cv::Size(21, 21), 3, criteria);

// 픽셀 → 실세계 속도 변환 (2m 고도 기준)
float fov_rad = FOV_DEG * M_PI / 180.0f;
float pixel_per_meter = img_w / (2.0f * altitude * std::tan(fov_rad / 2.0f));
float vx = (mean_dx / pixel_per_meter) * fps;
float vy = (mean_dy / pixel_per_meter) * fps;
```

#### 격자 교차점 스냅 보정 (C++)

```cpp
std::pair<Eigen::Vector2f, bool>
snapToGrid(const Eigen::Vector2f& pos,
           float spacing = 3.0f, float radius = 0.4f)
{
    float gx = std::round(pos.x() / spacing) * spacing;
    float gy = std::round(pos.y() / spacing) * spacing;
    float dist = (pos - Eigen::Vector2f(gx, gy)).norm();
    return (dist < radius)
        ? std::make_pair(Eigen::Vector2f(gx, gy), true)
        : std::make_pair(pos, false);
}
```

---

### 5.4 Sprint Planner Node

**언어**: C++  
**역할**: 격자 그래프 위에서 물리 기반 예상 시간이 최소인 경로를 계획한다.

#### 입출력

| | 내용 |
|--|------|
| 입력 | `/localization/pose_grid`, `/mission/target_node`, `/markers/confirmed` |
| 출력 | `/planner/velocity_profile`, `/planner/waypoints` |

#### 물리 파라미터 (params.yaml)

```yaml
sprint_planner:
  v_max_mps: 3.0
  a_max_mps2: 2.0
  a_brake_mps2: 2.5
  v_turn_mps: 0.5
  t_turn_90_sec: 1.5
  t_turn_180_sec: 2.5
  t_stabilize_sec: 0.5
  t_marker_check_sec: 2.0
  t_hover_sec: 3.0
  grid_spacing_m: 3.0
```

#### 물리 기반 시간 비용 (C++)

```cpp
float timeCostStraight(float L, float v0, float vf,
                        const DroneParams& p)
{
    float da = (p.v_max*p.v_max - v0*v0) / (2.0f*p.a_max);
    float db = (p.v_max*p.v_max - vf*vf) / (2.0f*p.a_brake);

    if (da + db <= L) {
        // 사다리꼴 프로파일
        return (p.v_max - v0) / p.a_max
             + (L - da - db) / p.v_max
             + (p.v_max - vf) / p.a_brake;
    } else {
        // 삼각형 프로파일 (v_max 미도달)
        float vp = std::sqrt(
            (2.0f*p.a_max*p.a_brake*L
             + p.a_max*vf*vf + p.a_brake*v0*v0)
            / (p.a_max + p.a_brake));
        return (vp - v0)/p.a_max + (vp - vf)/p.a_brake;
    }
}

float totalCost(const Route& r, float coverage_gain,
                float marker_likelihood, const DroneParams& p)
{
    float T = 0.0f;
    for (const auto& e : r.edges)
        T += timeCostStraight(e.length, e.v0, e.vf, p);
    T += r.n_turns_90 * p.t_turn_90 + r.n_turns_180 * p.t_turn_180;
    T += r.candidate_count * p.t_marker_check;
    T += r.confirmed_count * p.t_hover;

    return 1.0f * T
         + 0.5f * r.n_turns_total
         - 2.0f * coverage_gain
         - 1.5f * marker_likelihood;
}
```

---

### 5.5 Perception-Aware Flight Controller Node

**언어**: C++  
**역할**: 속도 프로파일을 받아 PX4 Offboard 속도 setpoint를 20Hz로 생성한다. 비전 인식 상태에 따라 동적으로 속도·가속도 제한을 조정하고, Anti-Sway 제어를 수행한다.

#### 입출력

| | 내용 |
|--|------|
| 입력 | `/planner/velocity_profile`, `/sway/metric`, `/markers/candidates`, `/vertiport/center_error` |
| 출력 | `/fmu/in/trajectory_setpoint` (20Hz), `/fmu/in/offboard_control_mode`, `/controller/mode` |

#### 제어 모드

```cpp
enum class ControlMode {
    TAKEOFF,
    SPRINT,           // 고속 직선 비행
    APPROACH,         // 교차점/마커 후보 접근 감속
    ANTI_SWAY,        // 흔들림 억제 안정화
    HOVER_CONFIRM,    // 마커 3초 호버링
    RETURN,           // 귀환
    VISION_SERVO,     // 버티포트 비전 서보
    LANDING,
};
```

#### 모드별 속도·가속도 제한 (params.yaml)

```yaml
flight_controller:
  sprint:
    v_max_xy: 3.0
    a_max_xy: 2.0
    jerk_max: 5.0
    tilt_max_deg: 25.0
  approach:
    v_max_xy: 1.0
    a_max_xy: 1.0
    jerk_max: 2.0
    tilt_max_deg: 15.0
  anti_sway:
    v_max_xy: 0.5
    a_max_xy: 0.5
    jerk_max: 1.0
    tilt_max_deg: 10.0
  hover_confirm:
    v_max_xy: 0.1
    a_max_xy: 0.2
    jerk_max: 0.5
    tilt_max_deg: 5.0
  vision_servo:
    v_max_xy: 0.3
    a_max_xy: 0.3
    jerk_max: 1.0
    v_max_z: 0.2
```

#### Offboard 제어 루프 (C++, 20Hz 타이머)

```cpp
void controlLoop() {
    // PX4 Offboard 유지: 반드시 20Hz 이상 퍼블리시
    publishOffboardControlMode();

    const auto& lim = limits_[mode_];
    auto v = computeTargetVelocity();
    v.x = std::clamp(v.x, -lim.v_max_xy, lim.v_max_xy);
    v.y = std::clamp(v.y, -lim.v_max_xy, lim.v_max_xy);

    // Anti-Sway 자동 개입
    if (sway_metric_ > SWAY_THRESHOLD && mode_ == ControlMode::SPRINT)
        transitionTo(ControlMode::ANTI_SWAY);

    publishTrajectorySetpoint(v);
}
```

---

### 5.6 Mission Manager Node

**언어**: C++  
**역할**: 전체 미션 상태 머신을 관리한다.

#### 상태 전환도

```
INIT → TAKEOFF → HOME_INIT → GRID_SEARCH
                                  │
                    (마커 후보 검출) │
                                  ▼
                          MARKER_APPROACH → ANTI_SWAY → HOVER_CONFIRM
                                                              │
                                            confirmed ────────┤
                                                              ▼
                                                        MARKER_SAVE → GRID_SEARCH
                                                                          │ (4개 완료)
                                                                          ▼
                                                                  RESCUE_ROUTE_PLAN
                                                                          │
                                                                  RESCUE_VISIT (ID 역순)
                                                                          │
                                                                  RETURN_HOME
                                                                          │
                                                                  VERTIPORT_ACQUIRE
                                                                          │
                                                                  VISION_SERVO_LAND
                                                                          │
                                                                       LANDED

※ 어느 상태에서든 배터리 부족 → EMERGENCY_RETURN → LANDING
```

#### 관리 데이터 구조 (C++)

```cpp
struct MarkerInfo {
    int id;
    Eigen::Vector2f grid_pos;
    Eigen::Vector2f home_pos;
    float confidence;
    double timestamp;
};

struct MissionState {
    std::string state;
    Eigen::Vector2f pose_grid;
    Eigen::Vector2f pose_home;
    std::set<std::pair<int,int>> visited_edges;
    std::vector<MarkerInfo> confirmed_markers;
    std::vector<int> rescue_order;    // ID 역순
    std::vector<int> rescue_visited;
    float battery_percent;
    float drift_metric;
};
```

#### 타이머 및 조건 (params.yaml)

```yaml
mission_manager:
  hover_confirm_duration_sec: 3.0
  battery_return_threshold_pct: 20
  max_drift_before_stabilize_m: 0.5
  home_approach_radius_m: 5.0
  vertiport_detect_radius_m: 3.0
```

---

### 5.7 Vertiport Detector Node

**언어**: C++  
**역할**: 귀환 후 하향 카메라로 버티포트를 재검출하고, 픽셀 오차 기반 비전 서보 착륙 명령을 생성한다.

#### 검출 알고리즘 (C++)

```cpp
std::optional<DetectionResult> detectVertiport(const cv::Mat& bgr) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv,
        cv::Scalar(10, 100, 100),   // 주황색 하한 (H, S, V)
        cv::Scalar(25, 255, 255),   // 주황색 상한
        mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT,
        1.2, 100, 50, 30, 100, 200);

    if (!circles.empty()) {
        auto& c = circles[0];
        return DetectionResult{cv::Point2f(c[0], c[1]), c[2], 0.8f};
    }
    return std::nullopt;
}
```

#### 비전 서보 (C++)

```cpp
constexpr float K_P_XY = 0.003f;   // 픽셀 오차 → m/s 게인

std::pair<float, float>
visionServoStep(const DetectionResult& det, int w, int h) {
    float ex = det.center.x - w / 2.0f;
    float ey = det.center.y - h / 2.0f;
    return {std::clamp(K_P_XY * ex, -0.3f, 0.3f),
            std::clamp(K_P_XY * ey, -0.3f, 0.3f)};
}

bool canDescend(const DetectionResult& det,
                float sway_metric, int w, int h) {
    return std::abs(det.center.x - w/2.0f) < 20.0f
        && std::abs(det.center.y - h/2.0f) < 20.0f
        && sway_metric < 0.1f
        && det.confidence > 0.7f;
}
```

---

### 5.8 Python 개발 도구

비실시간 작업에만 사용한다.

```python
# tools/log_analyzer.py — rosbag2 파싱 후 자동 리포트
# 분석: 총 임무 시간 / 마커 발견 오차 / 격자 이탈 / Sprint 속도 / 착륙 오차
import rclpy
from rosbag2_py import SequentialReader
import pandas as pd
import matplotlib.pyplot as plt
```

```python
# tools/plot_mission.py — 격자 위 비행 궤적 + 마커 위치 오버레이 시각화
```

```python
# tools/parameter_sweep.py — params.yaml 배치 수정 후 시뮬 반복 실행
# 결과 성능 지표 CSV 수집 및 최적 조합 탐색
```

---

## 6. ROS2 토픽 / 서비스 인터페이스 명세

### 6.1 전체 토픽 목록

| 토픽명 | 메시지 타입 | 발행 노드 | 주기 |
|--------|------------|----------|------|
| `/drone/camera/down/image_raw` | sensor_msgs/Image | Gazebo | 60Hz |
| `/drone/imu/data` | sensor_msgs/Imu | Gazebo | 200Hz |
| `/grid/lines` | GridLines | grid_detector (C++) | 30Hz |
| `/grid/intersections` | GridIntersections | grid_detector (C++) | 30Hz |
| `/grid/heading_angle` | std_msgs/Float32 | grid_detector (C++) | 30Hz |
| `/aruco/detections` | MarkerDetectionList | aruco_tracker (C++) | 30Hz |
| `/markers/candidates` | MarkerList | aruco_tracker (C++) | 10Hz |
| `/markers/confirmed` | MarkerList | aruco_tracker (C++) | 10Hz |
| `/optical_flow/velocity` | geometry_msgs/Vector3 | visual_odometry (C++) | 30Hz |
| `/localization/pose_grid` | geometry_msgs/PoseStamped | visual_odometry (C++) | 20Hz |
| `/localization/pose_home` | geometry_msgs/PoseStamped | visual_odometry (C++) | 20Hz |
| `/localization/drift_metric` | std_msgs/Float32 | visual_odometry (C++) | 10Hz |
| `/attitude/rpy` | geometry_msgs/Vector3 | px4_offboard_control (C++) | 50Hz |
| `/sway/metric` | std_msgs/Float32 | px4_offboard_control (C++) | 50Hz |
| `/controller/mode` | std_msgs/String | px4_offboard_control (C++) | 10Hz |
| `/mission/state` | std_msgs/String | mission_manager (C++) | 10Hz |
| `/mission/target_node` | geometry_msgs/Point | mission_manager (C++) | event |
| `/planner/velocity_profile` | VelocityProfile | sprint_planner (C++) | event |
| `/planner/waypoints` | geometry_msgs/PoseArray | sprint_planner (C++) | event |
| `/vertiport/center_error` | geometry_msgs/Vector3 | vertiport_detector (C++) | 30Hz |
| `/fmu/in/trajectory_setpoint` | px4_msgs/TrajectorySetpoint | px4_offboard_control (C++) | 20Hz |
| `/fmu/in/offboard_control_mode` | px4_msgs/OffboardControlMode | px4_offboard_control (C++) | 20Hz |
| `/fmu/in/vehicle_command` | px4_msgs/VehicleCommand | mission_manager (C++) | event |

### 6.2 서비스

| 서비스명 | 타입 | 역할 |
|---------|------|------|
| `/mission/start` | std_srvs/Trigger | 미션 시작 |
| `/mission/abort` | std_srvs/Trigger | 긴급 중단 |
| `/localizer/reset_home` | std_srvs/Trigger | Home Pose 재설정 |
| `/grid/reset_map` | std_srvs/Trigger | 격자 탐색 상태 초기화 |

---

## 7. 개발 단계 및 마일스톤

### Phase 0: 환경 구성

| # | 작업 | 완료 기준 | 비고 |
|---|------|-----------|------|
| 0-1 | Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic 설치 | `gz sim` 정상 실행 | — |
| 0-2 | PX4 SITL Ubuntu 24.04 빌드 | `make px4_sitl gz_x500` 성공 | 트윅 예상 |
| 0-3 | micro-XRCE-DDS ↔ ROS2 Jazzy 연동 | PX4 토픽 수신 확인 | — |
| 0-4 | 격자 Gazebo 월드 제작 (SDF) | 32×23m 안전구역 + 24×15m 비행구역 확인 | — |
| 0-5 | 하향 카메라 플러그인 추가 | 이미지 토픽 수신 확인 | — |
| 0-6 | ArUco 마커 배치 스크립트 (Python) | Gazebo 내 마커 시각적 확인 | tools/ |
| 0-7 | 기본 C++ Offboard 제어 테스트 | 2m 호버링 + 수평 이동 | — |

### Phase 1: 인식 모듈

| # | 작업 | 완료 기준 |
|---|------|-----------|
| 1-1 | grid_detector_node (C++) | 2m 고도 격자선/교차점 검출 |
| 1-2 | aruco_detector_node (C++) | 2m 고도 마커 ID 정확 인식 |
| 1-3 | Optical Flow (C++) | 정지 비행 중 velocity ≈ 0 |
| 1-4 | Marker Confidence Filter (C++) | 3초 안정 후 confirmed 전환 |
| 1-5 | 인식 통합 테스트 | 1m/s 이동 중 인식 성공률 ≥ 80% |

### Phase 2: 위치추정 + 제어

| # | 작업 | 완료 기준 |
|---|------|-----------|
| 2-1 | Grid Localizer (C++) | 교차점 스냅 drift < 0.3m |
| 2-2 | Home Pose 초기화 (C++) | 이륙 후 Home 저장 확인 |
| 2-3 | px4_offboard_control_node (C++) | Sprint/Anti-Sway 모드 전환 |
| 2-4 | PX4 파라미터 튜닝 | 2m 고도 ±0.2m 안정 유지 |
| 2-5 | 격자선 추종 비행 | 단일 Edge 직선 비행 성공 |

### Phase 3: 계획 + 미션

| # | 작업 | 완료 기준 |
|---|------|-----------|
| 3-1 | sprint_planner_node (C++) | 물리 비용 기반 경로 계산 확인 |
| 3-2 | mission_manager_node (C++) | INIT→GRID_SEARCH→HOVER_CONFIRM 전환 |
| 3-3 | 전체 탐색 루프 | 4개 마커 자동 탐색 완료 |
| 3-4 | 구조 경로 생성 및 방문 | 확정된 ID 순서 방문 + 각 3초 호버링 |
| 3-5 | 실패 복구 로직 | 재탐색, 격자 이탈 복귀 동작 확인 |

### Phase 4: 귀환 + 착륙

| # | 작업 | 완료 기준 |
|---|------|-----------|
| 4-1 | 귀환 경로 계획 | Home 좌표로 Sprint 귀환 |
| 4-2 | vertiport_detector_node (C++) | 2m 고도 버티포트 원형 검출 |
| 4-3 | 비전 서보 착륙 (C++) | 중심 오차 < 0.3m 착륙 |
| 4-4 | 전체 미션 통합 시뮬 | 처음~끝 1회 완주 |

### Phase 5: 최적화 + Python 도구

| # | 작업 | 완료 기준 |
|---|------|-----------|
| 5-1 | Sprint 파라미터 튜닝 | 임무 시간 단축 |
| 5-2 | log_analyzer.py 구현 | 자동 리포트 생성 |
| 5-3 | plot_mission.py 구현 | 비행 경로 시각화 |
| 5-4 | parameter_sweep.py 구현 | 배치 실험 자동화 |
| 5-5 | 랜덤 마커 배치 테스트 | 5종 이상 배치 통과 |
| 5-6 | 반복 성공률 측정 | 10회 중 8회 이상 완주 |

---

## 8. 테스트 계획

### 8.1 단위 테스트 (C++ gtest)

```bash
colcon test --packages-select grid_detector
colcon test --packages-select aruco_tracker
colcon test --packages-select sprint_planner
```

### 8.2 통합 테스트 시나리오

| 시나리오 | 설명 | 성공 기준 |
|---------|------|-----------|
| T01: 이착륙 | 이륙 → 2m 호버링 → 착륙 | 고도 오차 ±0.3m |
| T02: 격자 직진 | 단일 Edge 3m 직진 | 횡방향 오차 ±0.3m |
| T03: 격자 교차 | 교차점 90도 회전 | 다음 Edge 진입 확인 |
| T04: 마커 발견 | 1개 마커 + 3초 호버링 | Confirmed + ID 저장 |
| T05: 전체 탐색 | 4개 마커 탐색 | 탐색 완료 확인 |
| T06: 구조 방문 | 확정된 ID 순서로 4개 재방문 | 순서 정확 + 각 3초 호버링 |
| T07: 귀환 착륙 | 버티포트 착륙 | 중심 오차 < 0.5m |
| T08: 흔들림 복구 | 강제 외란 → Anti-Sway | 3초 내 안정화 |
| T09: 전체 미션 | T01~T07 연속 완주 | 완주 성공 |
| T10: 랜덤 배치 | 마커 위치 5종 무작위 | 5종 모두 완주 |

### 8.3 성능 지표

| 지표 | 초기 목표 | 최종 목표 |
|------|-----------|-----------|
| 전체 임무 시간 | 120초 이내 | 60초 이내 |
| 마커 인식 성공률 | ≥ 80% | ≥ 95% |
| 격자선 이탈 횟수 | ≤ 5회/미션 | ≤ 1회/미션 |
| 착륙 중심 오차 | ≤ 0.5m | ≤ 0.3m |
| 위치 drift | ≤ 0.5m (30m 주행) | ≤ 0.3m |
| 반복 완주 성공률 | ≥ 60% (10회) | ≥ 90% (10회) |

---

## 9. 리스크 및 대응

| 리스크 | 발생 단계 | 대응 |
|--------|----------|------|
| Ubuntu 24.04 + PX4 빌드 트윅 | Phase 0 | ubuntu.sh 수정, NuttX 의존성 버전 조정 |
| Gazebo Harmonic + ros-jazzy 플러그인 호환 이슈 | Phase 0 | ros-jazzy-ros-gz 버전 확인, 소스 빌드 검토 |
| PX4 ↔ ROS2 Jazzy DDS 메시지 버전 불일치 | Phase 0~2 | PX4 v1.16+ Message Translation Node 활용 |
| C++ ArUco 고속 비행 중 미검출 | Phase 1 | 속도 제한 강화, FPS 향상, ROI 처리 |
| visual_odometry drift 누적 | Phase 2 | 교차점 스냅 보정 주기 단축 |
| Sprint Planner 계산 시간 초과 | Phase 3 | 탐색 깊이 제한, 결과 캐싱 |
| 버티포트 색상 인식 실패 | Phase 4 | AprilTag 별도 착륙 마커 추가 검토 |
| Gazebo Harmonic 렌더링 부하 | 전체 | 해상도 낮춤, GPU 가속 활성화 |

---

## 부록: 빠른 시작 명령어

```bash
# 1. PX4 SITL + Gazebo Harmonic 실행
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# 2. DDS 에이전트 실행 (별도 터미널)
MicroXRCEAgent udp4 -p 8888

# 3. 전체 ROS2 Jazzy 스택 실행 (별도 터미널)
source /opt/ros/jazzy/setup.bash
source ~/sprint_drone_ws/install/setup.bash
ros2 launch sprint_drone sim_full.launch.py

# 4. 미션 시작
ros2 service call /mission/start std_srvs/srv/Trigger

# 5. 로그 분석 (Python)
python3 ~/sprint_drone_ws/tools/log_analyzer.py --bag latest

# 6. 경로 시각화 (Python)
python3 ~/sprint_drone_ws/tools/plot_mission.py --bag latest
```

---

*본 명세서는 프로토타입 개발 진행에 따라 지속 갱신한다.*
