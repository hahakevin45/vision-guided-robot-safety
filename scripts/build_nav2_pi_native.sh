#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${VGR_NAV2_WS:-$HOME/nav2_jazzy_ws}"
SOURCE_DIR="$WORKSPACE/src"
MANIFEST="$REPO_ROOT/config/nav2_pi_native_sources.tsv"
NAV2_PATCH="$REPO_ROOT/patches/navigation2-gcc14-denoise.patch"
NAV2_PATCHED_PATH="nav2_costmap_2d/include/nav2_costmap_2d/denoise/image_processing.hpp"
REQUIRED_APT=(
  colcon python3-rosdep2 build-essential cmake git pkg-config
  libsqlite3-dev libzmq3-dev libtinyxml2-dev uuid-dev
  libyaml-cpp-dev libgraphicsmagick++1-dev
  ros-jazzy-angles ros-jazzy-diagnostic-updater
  ros-jazzy-laser-geometry ros-jazzy-map-msgs
)
TARGETS=(
  nav2_map_server nav2_planner nav2_navfn_planner nav2_controller
  nav2_regulated_pure_pursuit_controller nav2_behaviors
  nav2_bt_navigator nav2_lifecycle_manager vgr_nav2_bringup
)

for package in "${REQUIRED_APT[@]}"; do
  dpkg-query -W -f='${Status}' "$package" 2>/dev/null |
    grep -q 'install ok installed' || {
      echo "missing apt package: $package" >&2
      exit 3
    }
done

set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$SOURCE_DIR"

while IFS=$'\t' read -r name url commit; do
  [[ -z "$name" || "$name" == \#* ]] && continue
  destination="$SOURCE_DIR/$name"
  if [[ ! -d "$destination/.git" ]]; then
    git init "$destination"
    git -C "$destination" remote add origin "$url"
  fi
  [[ "$(git -C "$destination" remote get-url origin)" == "$url" ]] || {
    echo "remote mismatch: $name" >&2
    exit 4
  }
  status="$(git -C "$destination" status --porcelain)"
  if [[ -n "$status" ]]; then
    if [[ "$name" != "navigation2" || \
          "$status" != " M $NAV2_PATCHED_PATH" ]] || \
       ! git -C "$destination" apply --reverse --check "$NAV2_PATCH" >/dev/null 2>&1; then
      echo "dirty source checkout: $name" >&2
      exit 4
    fi
  fi
  git -C "$destination" fetch --depth 1 origin "$commit"
  git -C "$destination" checkout --detach FETCH_HEAD
  [[ "$(git -C "$destination" rev-parse HEAD)" == "$commit" ]] || {
    echo "source revision mismatch: $name" >&2
    exit 4
  }
  if [[ "$name" == "navigation2" ]]; then
    if git -C "$destination" apply --reverse --check "$NAV2_PATCH" >/dev/null 2>&1; then
      :
    elif git -C "$destination" apply --check "$NAV2_PATCH" >/dev/null 2>&1; then
      git -C "$destination" apply "$NAV2_PATCH"
    else
      echo "Nav2 GCC 14 patch does not apply cleanly" >&2
      exit 4
    fi
  fi
done < "$MANIFEST"

ln -sfn "$REPO_ROOT/ros2_ws/src/vgr_nav2_bringup" \
  "$SOURCE_DIR/vgr_nav2_bringup"

cd "$WORKSPACE"
colcon build \
  --executor sequential \
  --symlink-install \
  --packages-up-to "${TARGETS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

set +u
source "$WORKSPACE/install/setup.bash"
set -u
for package in "${TARGETS[@]}"; do
  prefix="$(ros2 pkg prefix "$package")"
  [[ "$prefix" == "$WORKSPACE/install/"* ]] || {
    echo "package did not resolve to native overlay: $package=$prefix" >&2
    exit 5
  }
done

while IFS=$'\t' read -r name _url commit; do
  [[ -z "$name" || "$name" == \#* ]] && continue
  resolved="$(git -C "$SOURCE_DIR/$name" rev-parse HEAD)"
  [[ "$resolved" == "$commit" ]] || exit 5
  printf '%s\t%s\n' "$name" "$resolved"
done < "$MANIFEST" | tee "$WORKSPACE/resolved_sources.tsv"
