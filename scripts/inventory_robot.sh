#!/usr/bin/env bash
# Run locally on the Jetson. No CAN transmission or firmware/ODrive writes.
set -euo pipefail
output_dir=${1:?Usage: inventory_robot.sh OUTPUT_DIRECTORY}
mkdir -- "$output_dir"
uname -a > "$output_dir/uname.txt"
cat /etc/os-release > "$output_dir/os-release.txt"
dpkg-query -W 'ros-jazzy-*' > "$output_dir/ros-packages.txt" 2>/dev/null || true
ip -details link show can0 > "$output_dir/can0.txt" 2>&1 || true
if command -v ros2 >/dev/null; then
  timeout 10s ros2 node list > "$output_dir/nodes.txt" || true
  timeout 10s ros2 topic list -t > "$output_dir/topics.txt" || true
  for node in control_mux navigation_manager controller_server planner_server ekf_filter_node amcl can_telemetry cmd_vel_to_can; do
    timeout 10s ros2 param dump "/$node" > "$output_dir/$node.yaml" 2> "$output_dir/$node.errors.txt" || true
  done
fi
cat > "$output_dir/hardware_identity.json" <<'EOF'
{
  "live_stm32_firmware_hash": null,
  "live_stm32_firmware_build_id": null,
  "odrive_firmware_version": null,
  "odrive_velocity_ramp_rate_turns_s2": null,
  "odrive_velocity_limit_turns_s": null,
  "note": "Fill from verified device readback. Archived STM32 C source does not establish flashed firmware identity."
}
EOF
