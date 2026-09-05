#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec ros2 launch visualization visualization.launch.py map_path:="${HDMAP_PATH:-$ROOT/map/hdmap.bin}"
