# Figure Validation Summary

## Passed checks
- fig_01_system_block.png: 2354x1209
- fig_02_mission_flow.png: 2175x703
- fig_03_optical_flow_snap.png: 2175x1090
- fig_04_software_architecture.png: 2260x1245
- fig_05_gazebo_search_path.png: 1366x837
- fig_06_gazebo_mission_path.png: 1376x864
- fig_11_marker_sweep_results.png: 2779x1027
- fig_12_marker_layout_cases.png: 2601x903
- fig_13_phase1_speed_tradeoff.png: 2759x1027
- fig_08_gazebo_telemetry.png: 2397x1593
- fig_09_strategy_comparison.png: 2294x1009
- fig_10_safety_state_machine.png: 2175x1075
- Gazebo world marker positions match seed 310
- sweep CSV contains 600 random-layout cases
- sweep seed 310 marker grid coordinates match Gazebo world
- sweep class counts: clustered=327, mixed=216, spread=57
- Phase 1 speed sweep CSV contains 1800 cases
- Phase 1 speed sweep covers 5/7/10 m/s
- Telemetry summary contains Duration:
- Telemetry summary contains LANDED transition:
- Telemetry summary contains Pose samples:
- Telemetry summary contains Planner velocity profiles:
- Telemetry summary contains Downward camera frames:
- Fig. 8 pose source: 1437 samples match rosbag metadata
- Fig. 8 planner target source: 576 samples match rosbag metadata
- Fig. 8 setpoint speed source: 0 samples match rosbag metadata
- Fig. 8 altitude source: 1437 samples match rosbag metadata
- Fig. 8 camera source: 0 samples match rosbag metadata
- Fig. 8 grid source: 0 samples match rosbag metadata
- Fig. 8 marker source: 0 samples match rosbag metadata
- Fig. 8 duration matches metadata: 57.500s
- Fig. 8 pose-derived speed data: n=1436, mean=3.760m/s, p95=10.049m/s, max=10.775m/s
- Fig. 8 planner target speed data: n=259, min=3.000m/s, p95=10.000m/s, max=10.000m/s
- Fig. 8 altitude data: n=1437, min=0.100m, max=2.000m, final=0.100m
- Fig. 8 run summary max pose sample step: 0.400m
- Fig. 8 transitions: mission=[(0.09, 'TAKEOFF'), (1.39, 'HOME_INIT'), (1.49, 'GRID_SEARCH'), (3.19, 'MARKER_APPROACH'), (3.29, 'ANTI_SWAY'), (3.39, 'HOVER_CONFIRM'), (6.39, 'MARKER_SAVE'), (6.49, 'GRID_SEARCH'), (9.79, 'MARKER_APPROACH'), (9.89, 'ANTI_SWAY'), (9.99, 'HOVER_CONFIRM'), (12.99, 'MARKER_SAVE'), (13.09, 'GRID_SEARCH'), (21.09, 'MARKER_APPROACH'), (21.19, 'ANTI_SWAY'), (21.29, 'HOVER_CONFIRM'), (24.39, 'MARKER_SAVE'), (24.49, 'GRID_SEARCH'), (25.89, 'MARKER_APPROACH'), (25.99, 'ANTI_SWAY'), (26.09, 'HOVER_CONFIRM'), (29.09, 'MARKER_SAVE'), (29.19, 'RESCUE_ROUTE_PLAN'), (29.29, 'RESCUE_VISIT'), (47.19, 'RETURN_HOME'), (48.79, 'VERTIPORT_ACQUIRE'), (48.89, 'VISION_SERVO_LAND'), (52.69, 'LANDED')], controller=[]

## Warnings
- none
