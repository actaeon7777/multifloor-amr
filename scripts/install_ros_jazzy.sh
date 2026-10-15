#!/usr/bin/env bash
# Official ROS apt-source bootstrap for Ubuntu 24.04; no full system upgrade.
set -euo pipefail
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 ]] || {
  echo 'Ubuntu 24.04 is required.' >&2; exit 1;
}
[[ $EUID -eq 0 ]] || { echo "Run: sudo bash $0" >&2; exit 1; }
apt-get update
apt-get install -y curl ca-certificates software-properties-common python3
add-apt-repository -y universe
installer_tmp=$(mktemp -d)
trap 'rm -rf -- "$installer_tmp"' EXIT
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  curl --fail --location --proto '=https' --tlsv1.2 \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    -o "$installer_tmp/release.json"
  ros_source_version=$(/usr/bin/python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$installer_tmp/release.json")
  [[ $ros_source_version =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]
  curl --fail --location --proto '=https' --tlsv1.2 \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$ros_source_version/ros2-apt-source_${ros_source_version}.noble_all.deb" \
    -o "$installer_tmp/ros2-apt-source.deb"
  dpkg -i "$installer_tmp/ros2-apt-source.deb"
fi
apt-get update
apt-get install -y ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization \
  ros-jazzy-test-msgs ros-jazzy-slam-toolbox ros-jazzy-joy ros-jazzy-xacro ros-jazzy-rosbag2 \
  ros-jazzy-pcl-conversions ros-jazzy-tf2-eigen libpcl-dev libopencv-dev libeigen3-dev liboctomap-dev \
  python3-can python3-yaml python3-pytest python3-numpy python3-scipy python3-pil
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi
echo 'Installed. Next, as your normal user: ./scripts/build_ros.sh'
