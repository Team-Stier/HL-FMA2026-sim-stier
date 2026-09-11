#!/usr/bin/env bash
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/install/setup.bash"
exec ros2 run speed_annotator speed_annotator_node --ros-args -p map_path:="${HDMAP_PATH:-$ROOT/map/hdmap.bin}"
