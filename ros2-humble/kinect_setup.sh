#!/usr/bin/env bash
set -euo pipefail

K4A_VERSION="${K4A_VERSION:-1.4.1}"
ARCH="$(dpkg --print-architecture)"

ROS2_WS="${ROS2_WS:-/root/local_git/dev/ros2_ws}"
REPO_URL="${REPO_URL:-https://github.com/microsoft/Azure_Kinect_ROS_Driver.git}"
REPO_BRANCH="${REPO_BRANCH:-humble}"

CLONE_DRIVER="${CLONE_DRIVER:-1}"   # 1=clone/update, 0=skip
BUILD_DRIVER="${BUILD_DRIVER:-1}"   # 1=rosdep+colcon, 0=skip
CLEAN_WS="${CLEAN_WS:-1}"           # 1=rm -rf build install log before build, 0=skip

# Known-working debconf preseeding for libk4a EULA
K4A_EULA_HASH="${K4A_EULA_HASH:-0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76}"

log()  { echo -e "\033[1;32m[azure-kinect]\033[0m $*"; }
warn() { echo -e "\033[1;33m[azure-kinect]\033[0m $*" >&2; }
die()  { echo -e "\033[1;31m[azure-kinect]\033[0m $*" >&2; exit 1; }

need_root() {
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    die "Run as root (or via sudo)."
  fi
}

# With set -u, sourcing ROS scripts can fail on unset vars
safe_source() {
  local f="$1"
  [[ -f "$f" ]] || die "Cannot source missing file: $f"
  set +u
  # shellcheck disable=SC1090
  source "$f"
  set -u
}

export DEBIAN_FRONTEND=noninteractive

install_apt() {
  apt-get update
  apt-get install -y --no-install-recommends "$@"
}

ensure_universe() {
  if command -v add-apt-repository >/dev/null 2>&1; then
    add-apt-repository -y universe >/dev/null 2>&1 || true
  fi
}

ensure_libsoundio1() {
  if dpkg -s libsoundio1 >/dev/null 2>&1; then
    return 0
  fi

  log "Installing libsoundio1..."
  if apt-get update && apt-get install -y --no-install-recommends libsoundio1; then
    return 0
  fi

  warn "libsoundio1 not available from current apt sources; using Ubuntu archive .deb fallback..."
  local tmpd; tmpd="$(mktemp -d -p /var/tmp k4a.XXXXXX)"
  chmod 755 "$tmpd"

  wget -qO "$tmpd/libsoundio1.deb" \
    "http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_${ARCH}.deb"
  chmod 644 "$tmpd/libsoundio1.deb"

  dpkg -i "$tmpd/libsoundio1.deb" || true
  apt-get -f install -y
  rm -rf "$tmpd"
}

preseed_k4a_eula() {
  log "Pre-seeding Azure Kinect EULA acceptance (debconf)..."
  echo "libk4a1.4 libk4a1.4/accepted-eula-hash string ${K4A_EULA_HASH}" | debconf-set-selections
  echo "libk4a1.4 libk4a1.4/accept-eula boolean true" | debconf-set-selections
}

install_k4a_debs() {
  [[ "$ARCH" == "amd64" ]] || die "Azure Kinect .deb packages are amd64-only. Detected: $ARCH"

  preseed_k4a_eula

  log "Downloading Azure Kinect SDK ${K4A_VERSION} .deb packages..."
  local tmpd; tmpd="$(mktemp -d -p /var/tmp k4a.XXXXXX)"
  chmod 755 "$tmpd"

  local base="https://packages.microsoft.com/ubuntu/18.04/prod/pool/main"
  wget -qO "$tmpd/libk4a1.4.deb"     "$base/libk/libk4a1.4/libk4a1.4_${K4A_VERSION}_amd64.deb"
  wget -qO "$tmpd/libk4a1.4-dev.deb" "$base/libk/libk4a1.4-dev/libk4a1.4-dev_${K4A_VERSION}_amd64.deb"
  wget -qO "$tmpd/k4a-tools.deb"     "$base/k/k4a-tools/k4a-tools_${K4A_VERSION}_amd64.deb"
  chmod 644 "$tmpd/"*.deb

  log "Installing Azure Kinect .deb packages..."
  apt-get update
  apt-get install -y --no-install-recommends "$tmpd/libk4a1.4.deb"
  apt-get install -y --no-install-recommends "$tmpd/libk4a1.4-dev.deb" "$tmpd/k4a-tools.deb"
  ldconfig

  rm -rf "$tmpd"
}

fix_pkgconfig_name() {
  local pcdir="/usr/lib/x86_64-linux-gnu/pkgconfig"
  if [[ -d "$pcdir" ]]; then
    if [[ ! -e "$pcdir/k4a.pc" && -e "$pcdir/libk4a1.4.pc" ]]; then
      log "Creating pkg-config alias: k4a.pc -> libk4a1.4.pc"
      ln -sf "$pcdir/libk4a1.4.pc" "$pcdir/k4a.pc"
    fi
  fi

  log "Setting K4A_SDK_DIR=/usr (system-wide)"
  cat >/etc/profile.d/k4a.sh <<'EOF'
export K4A_SDK_DIR=/usr
EOF
}

install_udev_rules() {
  log "Installing udev rules: /etc/udev/rules.d/99-k4a.rules"
  groupadd -f plugdev || true

  cat >/etc/udev/rules.d/99-k4a.rules <<'EOF'
BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"
LABEL="k4a_logic_rules_end"
EOF

  udevadm control --reload-rules >/dev/null 2>&1 || true
  udevadm trigger >/dev/null 2>&1 || true
}

clone_driver_repo() {
  [[ "$CLONE_DRIVER" == "1" ]] || { warn "CLONE_DRIVER=0 set; skipping git clone."; return 0; }

  local src="$ROS2_WS/src"
  local repo_dir="$src/Azure_Kinect_ROS_Driver"

  log "Ensuring workspace exists: $ROS2_WS"
  mkdir -p "$src"

  if [[ -d "$repo_dir/.git" ]]; then
    log "Repo already exists; updating: $repo_dir"
    git -C "$repo_dir" fetch --all --prune
    git -C "$repo_dir" checkout "$REPO_BRANCH"
    git -C "$repo_dir" pull --ff-only || true
  else
    log "Cloning Azure_Kinect_ROS_Driver ($REPO_BRANCH) into: $repo_dir"
    git clone --branch "$REPO_BRANCH" --single-branch --depth 1 "$REPO_URL" "$repo_dir"
  fi
}

clean_workspace() {
  [[ "$CLEAN_WS" == "1" ]] || { warn "CLEAN_WS=0 set; skipping workspace clean."; return 0; }

  log "Cleaning workspace (rm -rf build install log) in: $ROS2_WS"
  rm -rf "$ROS2_WS/build" "$ROS2_WS/install" "$ROS2_WS/log"
}

build_ros_driver() {
  [[ "$BUILD_DRIVER" == "1" ]] || { warn "BUILD_DRIVER=0 set; skipping rosdep/colcon."; return 0; }

  [[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble not found at /opt/ros/humble"

  clean_workspace

  log "Sourcing /opt/ros/humble/setup.bash"
  safe_source /opt/ros/humble/setup.bash

  log "Running rosdep (skip nonstandard key: K4A) + colcon build"
  rosdep update || true
  ( cd "$ROS2_WS" && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys K4A )

  # IMPORTANT: use the build command that you confirmed produces a correct libexec layout.
  # (Your failing case used --symlink-install + --packages-select; your working fix used plain colcon build.)
  ( cd "$ROS2_WS" && colcon build )

  # Sanity check: ROS 2 expects libexec dir to exist for launch-able nodes
  if [[ ! -d "$ROS2_WS/install/azure_kinect_ros_driver/lib/azure_kinect_ros_driver" ]]; then
    warn "libexec directory is still missing after build. You can usually fix by:"
    warn "  rm -rf $ROS2_WS/build $ROS2_WS/install $ROS2_WS/log"
    warn "  source /opt/ros/humble/setup.bash"
    warn "  (cd $ROS2_WS && colcon build)"
    die "Build did not create expected libexec directory."
  fi

  log "Build finished. To use in your current shell run:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source $ROS2_WS/install/setup.bash"
}

main() {
  need_root
  log "Ubuntu: $(. /etc/os-release && echo "${PRETTY_NAME}")"
  log "Workspace: $ROS2_WS"
  log "Repo: $REPO_URL (branch: $REPO_BRANCH)"

  ensure_universe

  log "Installing base dependencies..."
  install_apt \
    ca-certificates curl wget gnupg2 lsb-release software-properties-common \
    apt-transport-https \
    git \
    debconf-utils \
    usbutils udev \
    pkg-config \
    libusb-1.0-0 libusb-1.0-0-dev \
    libgl1 libgl1-mesa-glx libgl1-mesa-dev

  ensure_libsoundio1
  install_k4a_debs
  fix_pkgconfig_name
  install_udev_rules

  clone_driver_repo
  build_ros_driver

  log "Done."
}

main "$@"
