# Development Status

## Current Prototype State

Implemented C++ core packages:

- `sprint_planner`
  - Minimum-time straight segment timing
  - Route cost model with turn, stop, marker, hover, and stabilization penalties
  - Unit test verifies a longer straight sprint can beat a shorter zigzag route

- `grid_detector`
  - Grid line orientation classification
  - Collinear line merging
  - Horizontal/vertical intersection extraction
  - Confidence calculation from valid intersections

- `aruco_tracker`
  - Marker Confidence Filter
  - Sway-aware confidence growth
  - Missing-observation decay
  - Confirmed/candidate/visited state handling

- `visual_odometry`
  - 2m altitude pixel-to-meter conversion
  - Optical-flow velocity conversion
  - Pose integration in yaw-aware local frame
  - Grid snap correction
  - Home/Grid SE2 transform helpers

- `mission_manager`
  - Mission phase state machine
  - Mission start/abort handling
  - Marker candidate, anti-sway, 3-second hover, and marker-save gates
  - 4-marker completion trigger
  - Rescue route generation in descending marker ID order
  - Rescue-visit hover timing and return-home transition
  - Emergency return on low battery

- `px4_offboard_control`
  - Control mode mapping from mission state
  - Per-mode XY/Z velocity limits
  - Acceleration-based velocity rate limiting
  - Marker-candidate approach override
  - Sway-triggered anti-sway override
  - Vision-servo pixel-error velocity clamp
  - 20Hz PX4 Offboard node scaffold gated on `px4_msgs`

ROS2/OpenCV node scaffolds added:

- `grid_detector_node`
- `aruco_tracker_node`
- `visual_odometry_node`
- `sprint_planner_node`
- `mission_manager_node`
- `px4_offboard_control_node`

Bringup package:

- `sprint_drone`
  - `sim_perception.launch.py`
  - `sim_planner.launch.py`
  - `sim_full.launch.py`
  - `sim_px4_full.launch.py`
  - `config/ros_params.yaml`
  - `config/bridge_px4.yaml`
  - `worlds/sprint_grid_preview.sdf`
  - `worlds/sprint_grid_world.sdf`

PX4 SITL integration:

- Local `MicroXRCEAgent` build under `external/install/micro-xrce-dds-agent`
- PX4-Autopilot clone under `external/PX4-Autopilot`
- `tools/make_px4_world.py`
  - generates `sprint_grid_world_px4.sdf`
  - strips the standalone fixed camera rig
  - keeps the competition grid, vertiport, and ArUco marker layout
- `scripts/run_micro_xrce_agent.sh`
- `scripts/run_px4_sitl.sh`
- PX4 `gz_x500_mono_cam_down` starts in `sprint_grid_world_px4`

Environment preview:

- `docs/environment_preview.html`
  - Top-down visual preview of the 32 x 23m safe area
  - 24 x 15m mission area
  - 3m grid
  - 2m flight altitude note
  - 3m diameter vertiport
  - sample marker positions and sprint/search path

## Local Verification

Pure C++ core verification:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

ROS2 package verification:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths src
```

Latest focused checks:

```text
px4_offboard_control_core_test passed
sim_px4_full.launch.py parses
PX4 SITL px4_sitl_default builds
MicroXRCEAgent starts on UDP 8888
PX4 Gazebo world starts with x500_mono_cam_down_0
PX4 SITL publishes vehicle_magnetometer and vehicle_air_data from Gazebo sensors
EKF2 reaches yaw, magnetometer, and barometer alignment in SITL
/fmu/out/vehicle_attitude publishes around 50 Hz
/drone/camera/down/image_raw publishes around 25-30 Hz
/sway/metric publishes around 20 Hz
/mission/start transitions the integrated stack into PX4 Offboard
PX4 reports armed=true and flag_control_offboard_enabled=true in full SITL smoke
```

## Suggested Next Step

Close the loop with an actual moving offboard mission in PX4 SITL:

- start PX4 + Agent + `sim_px4_full.launch.py`
- call `/mission/start`
- watch `/fmu/out/vehicle_control_mode` stay armed/offboard through the first climb
- watch `/fmu/out/vehicle_local_position` altitude converge to 2 m
- record a short rosbag and inspect the route with `tools/plot_mission.py`
