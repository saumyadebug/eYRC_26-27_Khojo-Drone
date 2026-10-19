#!/usr/bin/env bash
#
# One-shot setup for the eYRC Khojo-Drone Swift Pico workspace.
#
#   bash setup/install_workspace.sh              # default: ~/pico_ws
#   bash setup/install_workspace.sh /path/to/ws  # custom workspace
#
# What it does:
#   1. checks the prerequisites (ROS 2 Humble, colcon, OpenCV, NumPy)
#   2. clones this repository into <workspace>/src
#   3. initialises the git submodules (PID tuner, MAVLink, rotors, WhyCode)
#   4. installs the ROS packages the build needs
#   5. builds with colcon
#   6. writes an env.sh so the workspace can be sourced easily
#
set -e

WS="${1:-$HOME/pico_ws}"
REPO_URL="https://github.com/saumyadebug/eYRC_26-27_Khojo-Drone.git"
REPO_DIR="eYRC_26-27_Khojo-Drone"
ROS_DISTRO="${ROS_DISTRO:-humble}"

say() { printf '\n\033[1;34m==> %s\033[0m\n' "$1"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$1"; }
die() { printf '\033[1;31m[error] %s\033[0m\n' "$1" >&2; exit 1; }

# ---------------------------------------------------------------------------
say "Checking prerequisites"
# ---------------------------------------------------------------------------
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    die "ROS 2 ${ROS_DISTRO} not found. Install it first:
  sudo apt install ros-${ROS_DISTRO}-desktop"
fi
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

command -v colcon >/dev/null || die "colcon not found:
  sudo apt install python3-colcon-common-extensions"

python3 - <<'PY' || die "OpenCV and NumPy are required"
import cv2, numpy
print("  OpenCV", cv2.__version__, "| NumPy", numpy.__version__)
PY

echo "  ROS ${ROS_DISTRO} | $(colcon version-check 2>/dev/null | head -1 || echo colcon ok)"

# ---------------------------------------------------------------------------
say "Fetching the repository into ${WS}/src"
# ---------------------------------------------------------------------------
mkdir -p "${WS}/src"
if [ -d "${WS}/src/${REPO_DIR}/.git" ]; then
    echo "  already present, pulling latest"
    git -C "${WS}/src/${REPO_DIR}" pull --ff-only
else
    git clone "${REPO_URL}" "${WS}/src/${REPO_DIR}"
fi

# ---------------------------------------------------------------------------
say "Initialising git submodules"
# ---------------------------------------------------------------------------
git -C "${WS}/src/${REPO_DIR}" submodule update --init --recursive
git -C "${WS}/src/${REPO_DIR}" submodule status

# ---------------------------------------------------------------------------
say "Installing the ROS packages the build needs"
# ---------------------------------------------------------------------------
# actuator_msgs  -> required by rotors_control
# image_view     -> used by the simulation launch file (camera preview)
# Both are standard ROS 2 packages.  If they are already installed this is a
# no-op.  Without sudo the script falls back to extracting the .deb into a
# local overlay prefix, which is picked up by the generated env.sh.
APT_PKGS=(ros-${ROS_DISTRO}-actuator-msgs ros-${ROS_DISTRO}-image-view
          ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-image-pipeline)

if command -v apt-get >/dev/null && sudo -n true 2>/dev/null; then
    sudo apt-get update -qq || true
    sudo apt-get install -y "${APT_PKGS[@]}"
    OVERLAY=""
else
    warn "no passwordless sudo - installing into a local overlay prefix instead"
    OVERLAY="${HOME}/.local/opt_overlay"
    mkdir -p "${OVERLAY}"
    TMP=$(mktemp -d)
    ( cd "${TMP}" && apt-get download "${APT_PKGS[@]}" >/dev/null 2>&1 ) \
        || die "could not download the ROS packages; please install them with:
  sudo apt install ${APT_PKGS[*]}"
    for d in "${TMP}"/*.deb; do dpkg-deb -x "$d" "${TMP}/x"; done
    cp -r "${TMP}/x/opt/ros/${ROS_DISTRO}/." "${OVERLAY}/"
    rm -rf "${TMP}"
    echo "  overlay at ${OVERLAY}"
fi

# ---------------------------------------------------------------------------
say "Building the workspace"
# ---------------------------------------------------------------------------
cd "${WS}"
colcon build --symlink-install

# ---------------------------------------------------------------------------
say "Writing ${WS}/env.sh"
# ---------------------------------------------------------------------------
cat > "${WS}/env.sh" <<EOF
# Source this to use the workspace:
#     source ~/pico_ws/env.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
export AMENT_PREFIX_PATH="${OVERLAY:-\$HOME/.local/opt_overlay}:\$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="${OVERLAY:-\$HOME/.local/opt_overlay}:\$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="${OVERLAY:-\$HOME/.local/opt_overlay}/lib:\$LD_LIBRARY_PATH"
if [ -f "$HOME/pico_ws/install/setup.bash" ]; then
    source "$HOME/pico_ws/install/setup.bash"
fi
EOF

cat <<EOF

$(printf '\033[1;32m')Setup complete.$(printf '\033[0m')

  Workspace : ${WS}
  Source it : source ${WS}/env.sh

  Simulation (terminal 1):
    ros2 launch swift_pico swift_pico_simulation.launch.py
  Task 1C  (terminal 2):
    ros2 run swift_pico task_1c_controller
  Errors   (terminal 3):
    ros2 topic echo /pos_error

EOF
