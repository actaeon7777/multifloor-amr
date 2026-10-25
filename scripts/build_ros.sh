#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -f /opt/ros/jazzy/setup.bash ]] || {
  echo "Install ROS first: sudo bash $workspace_dir/scripts/install_ros_jazzy.sh" >&2; exit 1;
}
# Exclude Conda executables from the ROS Python/CMake build.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths src --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
/usr/bin/python3 -m pytest -q tests sim/scripts/test_drive_controller.py
python3 scripts/test_ros_integration.py
