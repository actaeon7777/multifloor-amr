#!/usr/bin/env bash
# Run inside a newly extracted source release on the Jetson, without live vehicle IO.
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ $(uname -m) == aarch64 ]] || { echo 'Jetson release validation requires aarch64' >&2; exit 1; }
"$workspace_dir/scripts/build_ros.sh"
# build_ros.sh runs unit tests and isolated ROS integration with CAN dry-run fixtures.
date --iso-8601=seconds > "$workspace_dir/RELEASE_VALIDATED"
echo "Validated source build: $workspace_dir"
