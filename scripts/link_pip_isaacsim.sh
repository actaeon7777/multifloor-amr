#!/usr/bin/env bash
# Expose a pip-installed Isaac Sim 6.1.0 venv at runtime/isaac-sim-6.1.0 using the
# standalone layout the wrappers expect (python.sh, isaac-sim.sh, kit headers).
# Usage: scripts/link_pip_isaacsim.sh [venv]   (default: ~/isaacsim-env)
set -euo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
venv=$(cd -- "${1:-$HOME/isaacsim-env}" && pwd)
python="$venv/bin/python"
[[ -x $python ]] || { echo "Python not found: $python" >&2; exit 1; }
package=$("$python" -c 'import importlib.util as u; print(u.find_spec("isaacsim").submodule_search_locations[0])')
version=$(<"$package/VERSION")
[[ $version == 6.1.0* ]] || { echo "Expected Isaac Sim 6.1.0, found $version" >&2; exit 1; }
include=$("$python" -c 'import sysconfig; print(sysconfig.get_paths()["include"])')
[[ -f $include/Python.h ]] || { echo "Python.h missing ($include); install python3-dev" >&2; exit 1; }

target="$workspace_dir/runtime/isaac-sim-6.1.0"
if [[ -e $target && ! -f $target/.pip_shim ]]; then
    echo "Refusing to replace non-shim runtime: $target" >&2; exit 1
fi
rm -rf "$target"
mkdir -p "$target/kit/python/include"
ln -s "$package/kit/dev" "$target/kit/dev"
ln -s "$include" "$target/kit/python/include/python3.12"
# Standalone Isaac preloads its bundled Jazzy libraries for the ROS 2 bridge;
# a pip install needs them on LD_LIBRARY_PATH before the process starts.
ros_lib="$package/exts/isaacsim.ros2.core/jazzy/lib"
[[ -f $ros_lib/libament_index_cpp.so ]] || { echo "Bundled Jazzy libs missing: $ros_lib" >&2; exit 1; }
cat >"$target/python.sh" <<EOF
#!/usr/bin/env bash
export LD_LIBRARY_PATH="$ros_lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
exec "$python" "\$@"
EOF
cat >"$target/isaac-sim.sh" <<EOF
#!/usr/bin/env bash
export LD_LIBRARY_PATH="$ros_lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
exec "$venv/bin/isaacsim" isaacsim.exp.full "\$@"
EOF
chmod +x "$target/python.sh" "$target/isaac-sim.sh"
printf '%s\n' "$venv" >"$target/.pip_shim"
echo "Linked Isaac Sim $version from $venv -> $target"
