# Prototype Tools

Python scripts in this directory are for non-realtime development work only:

- log analysis
- mission plotting
- parameter sweeps
- Gazebo marker spawning
- dataset labeling

Realtime perception, planning, mission, and control loops stay in C++.

Recommended local tool environment:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install rosbags matplotlib numpy opencv-python
```

Common commands:

```bash
python3 tools/gen_aruco_markers.py
python3 tools/spawn_markers.py --seed 42 --write
python3 tools/log_analyzer.py bags/<run>
python3 tools/plot_mission.py bags/<run>
```
