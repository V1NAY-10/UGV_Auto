#!/usr/bin/env bash
# One-shot installer for the UGV_Auto project (Ubuntu 22.04 + ROS 2 Humble).
#
#   bash install.sh pi   # on the Raspberry Pi 4B: robot runtime, RPLIDAR A1, rf2o, GPIO UART
#   bash install.sh pc   # on the laptop/PC: Gazebo simulation, RViz, SLAM, Nav2
#
# Workspace defaults to ~/ros2_ws (override with UGV_WS=/path bash install.sh pi).
# Safe to re-run: every step skips what is already done.
set -eo pipefail

MODE="${1:-}"
if [[ "$MODE" != "pi" && "$MODE" != "pc" ]]; then
  echo "usage: $0 pi|pc"
  exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${UGV_WS:-$HOME/ros2_ws}"

step() { echo -e "\n\033[1;34m==> $*\033[0m"; }

. /etc/os-release
if [[ "$VERSION_CODENAME" != "jammy" ]]; then
  echo "ROS 2 Humble needs Ubuntu 22.04 (jammy). Found: $PRETTY_NAME"
  echo "On the Raspberry Pi, flash 'Ubuntu Server 22.04 LTS (64-bit)' with Raspberry Pi Imager."
  exit 1
fi

# ---------------------------------------------------------------- ROS 2 apt repo
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  step "Adding the ROS 2 apt repository"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl locales
  sudo locale-gen en_US en_US.UTF-8
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

# ---------------------------------------------------------------- apt packages
step "Installing ROS 2 Humble packages ($MODE)"
PKGS=(
  ros-humble-xacro
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-nav2-simple-commander
  ros-humble-teleop-twist-keyboard
  python3-serial
  python3-colcon-common-extensions
  python3-rosdep
  git
  build-essential
)
if [[ "$MODE" == "pi" ]]; then
  PKGS+=(ros-humble-ros-base)
else
  PKGS+=(ros-humble-desktop ros-humble-gazebo-ros-pkgs)
fi
sudo apt-get update
sudo apt-get install -y "${PKGS[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# ---------------------------------------------------------------- workspace
step "Setting up workspace $WS"
mkdir -p "$WS/src"
if [[ "$REPO_DIR" != "$WS/src/"* && ! -e "$WS/src/UGV_Auto" ]]; then
  ln -s "$REPO_DIR" "$WS/src/UGV_Auto"
fi

clone() {  # clone <url> <branch> <dir>
  if [[ ! -d "$WS/src/$3" ]]; then
    git clone --depth 1 -b "$2" "$1" "$WS/src/$3"
  fi
}

if [[ "$MODE" == "pi" ]]; then
  # RPLIDAR A1 driver (provides rplidar_node)
  clone https://github.com/Slamtec/rplidar_ros.git ros2 rplidar_ros
  # Laser odometry - replaces wheel encoders
  clone https://github.com/MAPIRlab/rf2o_laser_odometry.git ros2 rf2o_laser_odometry
fi

step "Installing remaining dependencies with rosdep"
source /opt/ros/humble/setup.bash
ROSDEP_ARGS=(--from-paths "$WS/src" --ignore-src -y --rosdistro humble)
if [[ "$MODE" == "pi" ]]; then
  ROSDEP_ARGS+=(--skip-keys "gazebo_ros gazebo_plugins rviz2")   # not needed on the robot
fi
rosdep install "${ROSDEP_ARGS[@]}"

step "Building"
cd "$WS"
if [[ "$MODE" == "pi" ]]; then
  # Limit parallel jobs so the Pi does not run out of RAM
  MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 2
else
  colcon build --symlink-install
fi

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "source $WS/install/setup.bash" ~/.bashrc || echo "source $WS/install/setup.bash" >> ~/.bashrc

# ---------------------------------------------------------------- Pi hardware
if [[ "$MODE" == "pi" ]]; then
  step "Serial permissions and LiDAR udev rule"
  sudo usermod -aG dialout "$USER"
  sudo cp "$REPO_DIR/udev/99-ugv.rules" /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger

  step "Enabling the GPIO UART for the ESP32 (/dev/ttyAMA0 on GPIO14/15)"
  CFG=/boot/firmware/config.txt
  CMDLINE=/boot/firmware/cmdline.txt
  if ! grep -qxF "dtoverlay=disable-bt" "$CFG"; then
    # Moves the full PL011 UART from Bluetooth to GPIO14/15 (the ESP32 has its own Bluetooth)
    printf '\n[all]\nenable_uart=1\ndtoverlay=disable-bt\n' | sudo tee -a "$CFG" > /dev/null
  fi
  # Stop the Linux serial console from talking on the same pins
  sudo sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' "$CMDLINE"
  sudo systemctl disable --now serial-getty@ttyAMA0.service serial-getty@ttyS0.service 2>/dev/null || true
  sudo systemctl disable --now hciuart.service 2>/dev/null || true

  step "Done. REBOOT the Pi now (sudo reboot) so the UART and group changes apply."
else
  step "Done. Open a new terminal (or: source ~/.bashrc), then: ros2 launch ugv_auto sim.launch.py"
fi
