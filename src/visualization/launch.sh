#!/usr/bin/env bash
set -e
exec ros2 launch visualization visualization.launch.py map_path:="${HDMAP_PATH:-}"
