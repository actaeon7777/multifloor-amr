# Upstream source tracking

Verified on 2026-09-29 against the official default branches:

- ROS 2 driver: https://github.com/Slamtec/sllidar_ros2/commit/34300099fadfc772965962dec837bf436706188f
- Embedded SDK: https://github.com/Slamtec/rplidar_sdk/commit/99478e5fb90de3b4a6db0080acacd373f8b36869
- SDK version: 2.1.0; the upstream fix does not change the version number.

The SDK update corrects the UltraDense sample delay calculation in
`sdk/src/dataunpacker/unpacker/handler_capsules.cpp`. The complete `sdk/`
directory matches the SDK commit above.

The SDK update retained the existing local launch and filter configuration.
The S2 launch file's default serial device was subsequently corrected on
2026-09-29 to the connected CP2102N adapter (serial
`1805f83a4477ed119669f0fafdf7b791`), using its stable `/dev/serial/by-id/` path.
The old launch file is backed up beside the SDK backup.
The `config/` installation rule and `config/scan_filter_chain.yaml` are retained.
Neither change alters scan orientation or TF.

The previous SDK source and pre-update hashes are stored in
`../../diagnostics/lidar_2026-09-29/` relative to this package.
