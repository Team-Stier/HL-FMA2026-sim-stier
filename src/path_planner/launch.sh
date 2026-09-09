#!/usr/bin/env bash
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/install/setup.bash"
exec ros2 launch path_planner path_planner.launch.py map_path:="${HDMAP_PATH:-$ROOT/map/hdmap.bin}"
