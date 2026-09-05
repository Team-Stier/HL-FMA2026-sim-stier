#!/usr/bin/env bash
# Source in a child Bash so the caller's shell remains untouched.
if [[ ${BASH_SOURCE[0]} != "$0" ]]; then
    if bash "${BASH_SOURCE[0]}" "$@"; then return 0; else return $?; fi
fi
set -eo pipefail
cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"

if [[ ${1:-} == --help ]]; then
    echo 'Usage: source /path/to/run.sh [--check]'
    echo '--check: list available and missing launch scripts without changing anything.'
    exit 0
fi
if (( $# > 1 )) || [[ $# == 1 && $1 != --check ]]; then
    echo 'Usage: run.sh [--check|--help]' >&2
    exit 2
fi
source /etc/os-release
if [[ $ID != ubuntu || $VERSION_ID != 24.04 ]]; then
    echo 'run.sh requires Ubuntu 24.04 / ROS 2 Jazzy.' >&2
    exit 1
fi
if /usr/bin/python3 scripts/bringup.py check; then
    :
else
    status=$?
    [[ $status == 3 ]] && exit 0  # No launch scripts: no installation/build/cleanup needed.
    exit "$status"
fi
[[ ${1:-} != --check ]] || exit 0

# Serialize bringup so two invocations cannot kill/start each other's nodes.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/hl-fma-bringup-$UID.lock"
flock -n 9 || { echo 'Another bringup is already running.' >&2; exit 1; }
/usr/bin/python3 scripts/bringup.py stop-existing

as_root() {
    if (( EUID == 0 )); then "$@"; else
        command -v sudo >/dev/null || { echo 'Install sudo or run dependency installation as root.' >&2; return 1; }
        if [[ -t 0 ]]; then
            echo 'Missing public dependencies require sudo privileges.' >&2
            sudo -v || return 1
        else
            sudo -n true || { echo 'No non-interactive sudo permission; run in a terminal to install dependencies.' >&2; return 1; }
        fi
        sudo -n "$@"
    fi
}
installed() { [[ $(dpkg-query -W -f='${Status}' "$1" 2>/dev/null) == 'install ok installed' ]]; }

if ! installed ros2-apt-source; then
    as_root apt-get update
    as_root apt-get install -y ca-certificates curl software-properties-common
    as_root add-apt-repository -y universe
    apt_deb=$(mktemp --suffix=.deb)
    trap 'rm -f -- "$apt_deb"' EXIT
    version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
    curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/ros2-apt-source_${version}.noble_all.deb" -o "$apt_deb"
    as_root dpkg -i "$apt_deb"
    rm -f -- "$apt_deb"
    trap - EXIT
fi
missing=()
for package in ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-lanelet2-core \
    python3-colcon-common-extensions python3-rosdep python3-yaml \
    build-essential cmake libeigen3-dev libboost-all-dev; do
    installed "$package" || missing+=("$package")
done
if (( ${#missing[@]} )); then
    as_root apt-get update
    as_root apt-get install -y "${missing[@]}"
fi
source /opt/ros/jazzy/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    as_root rosdep init
fi
rosdep update --rosdistro jazzy
if ! rosdep check --from-paths src --ignore-src --rosdistro jazzy; then
    # Authenticate before rosdep invokes sudo itself.
    as_root true
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
fi
colcon build --cmake-clean-cache
source install/setup.bash
exec /usr/bin/python3 scripts/bringup.py run
