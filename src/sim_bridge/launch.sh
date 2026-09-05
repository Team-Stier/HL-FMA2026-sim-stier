#!/usr/bin/env bash
set -eo pipefail
workspace_root=$(dirname -- "$(dirname -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")")")
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
exec ros2 run sim_bridge sim_bridge_node --ros-args \
    -p host:="${VTD_HOST:-127.0.0.1}" \
    -p port:="${VTD_PORT:-9910}" \
    -p runtime_config:="$workspace_root/config/runtime.yaml" \
    -p vehicle_config:="$workspace_root/config/vehicle.yaml" \
    "$@"
