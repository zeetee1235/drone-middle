# Development Environment Setup

This workspace targets Ubuntu 24.04 with ROS2 Jazzy and Gazebo Harmonic.

## Install

Run from the repository root:

```bash
sudo bash scripts/install_dev_env_ubuntu24.sh
```

The installer adds:

- ROS2 Jazzy apt repository
- Gazebo Harmonic apt repository
- `ros-jazzy-desktop`
- `gz-harmonic`
- `ros-jazzy-ros-gz-*`
- `ros-jazzy-cv-bridge`
- `ros-jazzy-image-transport`
- OpenCV development headers
- colcon and ROS development tools
- optional `ros-jazzy-px4-msgs` if available from apt

## Check

```bash
scripts/check_dev_env.sh
```

## Source ROS

```bash
source /opt/ros/jazzy/setup.bash
```

## View Current Gazebo World

```bash
gz sim src/sprint_drone/worlds/sprint_grid_preview.sdf
```

## Build ROS2 Workspace

After ROS2 is installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Prototype

```bash
ros2 launch sprint_drone sim_full.launch.py
```

## PX4 SITL Mode

Prepare PX4 and the Micro XRCE-DDS Agent once:

```bash
scripts/setup_px4_sitl.sh
```

This clones PX4 into `external/PX4-Autopilot`, builds a local
`MicroXRCEAgent`, installs PX4 Python requirements for the current user, and
generates the PX4-only world at:

```text
external/PX4-Autopilot/Tools/simulation/gz/worlds/sprint_grid_world_px4.sdf
```

Build the PX4 SITL binary:

```bash
make -C external/PX4-Autopilot px4_sitl_default
```

Run the moving-drone stack in three terminals:

```bash
scripts/run_micro_xrce_agent.sh
```

```bash
HEADLESS=1 scripts/run_px4_sitl.sh
```

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sprint_drone sim_px4_full.launch.py
```

Useful overrides:

```bash
PX4_MODEL_POSE=2,19,0.72,0,0,0 HEADLESS=1 scripts/run_px4_sitl.sh
PX4_WORLD_NAME=sprint_grid_world_px4 scripts/run_px4_sitl.sh
PX4_RELAXED_PREFLIGHT=0 HEADLESS=1 scripts/run_px4_sitl.sh
PORT=8888 scripts/run_micro_xrce_agent.sh
```

`PX4_RELAXED_PREFLIGHT=1` is the default for this prototype SITL flow. It only
affects the local simulation run and sets PX4 parameter overrides for lab-only
power/GCS checks so offboard arming can be tested without a ground-control
station. Barometer, compass, and navsat simulation are provided by the Gazebo
world plugins. `scripts/run_px4_sitl.sh` also forces `SYS_HAS_BARO=1` and
`SYS_HAS_MAG=1` so stale PX4 parameter files cannot disable the simulated
barometer/compass path. Set `PX4_RELAXED_PREFLIGHT=0` when you want strict PX4
preflight behavior.

Quick checks:

```bash
ros2 topic hz /fmu/out/vehicle_attitude
ros2 topic hz /drone/camera/down/image_raw
ros2 topic hz /sway/metric
```
