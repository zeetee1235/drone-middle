#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Please run with sudo:"
  echo "  sudo bash scripts/install_dev_env_ubuntu24.sh"
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

echo "[1/8] Installing base apt tools"
apt update
apt install -y \
  ca-certificates \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

echo "[2/8] Enabling Ubuntu universe repository"
add-apt-repository -y universe
apt update

echo "[3/8] Adding ROS2 Jazzy apt repository"
install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  > /etc/apt/sources.list.d/ros2.list

echo "[4/8] Adding Gazebo Harmonic apt repository"
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

echo "[5/8] Updating package index"
apt update

echo "[6/8] Installing ROS2 Jazzy, Gazebo Harmonic, and bridge packages"
apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  gz-harmonic \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-image \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-opencv \
  libopencv-dev \
  pkg-config \
  python3-pip

echo "Installing optional px4_msgs package if available"
if apt-cache show ros-jazzy-px4-msgs >/dev/null 2>&1; then
  apt install -y ros-jazzy-px4-msgs
else
  echo "ros-jazzy-px4-msgs not found in apt; install px4_msgs from source later if needed."
fi

echo "[7/8] Initializing rosdep if needed"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init || true
fi
sudo -u "${SUDO_USER:-$(logname)}" rosdep update || true

echo "[8/8] Installing optional Python analysis tools"
sudo -u "${SUDO_USER:-$(logname)}" python3 -m pip install --user \
  --break-system-packages \
  matplotlib pandas numpy scipy || true

echo
echo "Done."
echo "Open a new terminal or run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo
echo "Quick checks:"
echo "  ros2 --version"
echo "  gz sim --versions"
echo "  colcon --help"
