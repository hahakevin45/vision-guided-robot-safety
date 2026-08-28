#!/bin/bash
# 設定 strict mode
set -euo pipefail

# 偵測 Gazebo Fortress 命令行工具：優先使用 ign gazebo，若無則使用 gz sim
GZ_CMD=""
if command -v ign &> /dev/null; then
  # 檢查 ign gazebo 是否可用
  if ign gazebo --version &> /dev/null || ign gazebo -h &> /dev/null; then
    GZ_CMD="ign gazebo"
  fi
fi

if [ -z "$GZ_CMD" ] && command -v gz &> /dev/null; then
  # 檢查 gz sim 是否可用
  if gz sim --version &> /dev/null || gz sim -h &> /dev/null; then
    GZ_CMD="gz sim"
  fi
fi

# 如果兩者都不存在，輸出安裝提示並以狀態碼 2 結束
if [ -z "$GZ_CMD" ]; then
  echo "找不到 Gazebo Fortress 命令行工具。"
  echo "請安裝套件：apt install ros-humble-ros-gz"
  exit 2
fi

# 顯示偵測到的 Gazebo 版本
echo "偵測到的 Gazebo 版本資訊："
$GZ_CMD --version || true

# 設定模擬世界檔案，預設為系統內建的 empty.sdf，若找不到則生成一個臨時的最小 SDF 檔案
WORLD_FILE="${1:-}"
CLEANUP_TEMP=false
TEMP_WORLD=""

cleanup() {
  if [ "$CLEANUP_TEMP" = true ] && [ -n "$TEMP_WORLD" ] && [ -f "$TEMP_WORLD" ]; then
    rm -f "$TEMP_WORLD"
  fi
}
trap cleanup EXIT

if [ -z "$WORLD_FILE" ]; then
  # 嘗試尋找系統中可能存在的 empty.sdf 檔案
  # Gazebo Fortress 預設在 /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
  # 或者在 /opt/ros/humble/share/ 之下
  if [ -f "/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf" ]; then
    WORLD_FILE="/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf"
  elif [ -f "/opt/ros/humble/share/ros_gz_sim/worlds/empty.sdf" ]; then
    WORLD_FILE="/opt/ros/humble/share/ros_gz_sim/worlds/empty.sdf"
  else
    # 找不到預設檔案時，使用 mktemp 生成一個最小的 inline 虛擬世界檔案
    TEMP_WORLD=$(mktemp --suffix=.sdf)
    CLEANUP_TEMP=true
    cat << 'EOF' > "$TEMP_WORLD"
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="libignition-gazebo-physics-system.so"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="libignition-gazebo-user-commands-system.so"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
      filename="libignition-gazebo-scene-broadcaster-system.so"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
  </world>
</sdf>
EOF
    WORLD_FILE="$TEMP_WORLD"
  fi
fi

# 執行無 GUI 的伺服器端模擬，設定 1000 次疊代，並使用 timeout(1) 限制執行時間在 60 秒內
# 參數說明：
# -s: 僅伺服器模式 (server-only/headless)
# -r: 啟動後立即運行 (run on start)
# --iterations 1000: 運行剛好 1000 次疊代後結束
if timeout 60 $GZ_CMD -s -r --iterations 1000 "$WORLD_FILE"; then
  echo "SMOKE_OK"
else
  echo "SMOKE_FAIL"
  exit 1
fi
