#!/usr/bin/env bash
set -u

echo "== OS =="
cat /etc/os-release | grep -E 'PRETTY_NAME|VERSION_CODENAME' || true

echo
echo "== Commands =="
for cmd in ros2 colcon gz pkg-config cmake; do
  if command -v "$cmd" >/dev/null 2>&1; then
    printf "%-12s %s\n" "$cmd" "$(command -v "$cmd")"
  else
    printf "%-12s MISSING\n" "$cmd"
  fi
done

echo
echo "== ROS =="
if command -v ros2 >/dev/null 2>&1; then
  ros2 --help >/dev/null && echo "ros2 ok" || echo "ros2 command exists but failed"
fi

echo
echo "== Gazebo =="
if command -v gz >/dev/null 2>&1; then
  gz sim --versions || true
fi

echo
echo "== OpenCV pkg-config =="
pkg-config --modversion opencv4 2>/dev/null || echo "opencv4 pkg-config missing"

echo
echo "== Apt package probes =="
for pkg in ros-jazzy-desktop gz-harmonic ros-jazzy-cv-bridge ros-jazzy-px4-msgs; do
  dpkg -s "$pkg" >/dev/null 2>&1 && echo "$pkg installed" || echo "$pkg missing"
done
