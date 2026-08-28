# Nav2 Gazebo 模擬與驗收

日期：2026-07-13

## 完成狀態

Nav2 已作為 Gazebo 模擬的高階命令來源。權威 headless runner 會在已知
靜態地圖中送出 `(3.5, 0, -0.5)` 的 `NavigateToPose` goal；車從 `(0.7, 0, 0)`
出發，繞過中央固定障礙物，所有速度命令都先經過 `safe_apf`，再送入
Gazebo DiffDrive。

```text
NavigateToPose
  -> Nav2 planner/controller
  -> /cmd_vel_nav
  -> safety_gate (safe_apf + command watchdog)
  -> /cmd_vel_safe
  -> ros_gz_bridge
  -> Gazebo DiffDrive
```

兩個可重複的權威 gate 已通過：

| 模式 | Action | 繞障 | 終點誤差 | Yaw 誤差 | 最小 clearance | Nav/Safe/Plan |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `ground_truth pseudo` | SUCCEEDED | north | 0.0780 m | 0.1286 rad | 0.1270 m | 765 / 752 / 38 |
| `wheel_odom pseudo` | SUCCEEDED | north | 0.0774 m | 0.1711 rad | 0.1111 m | 838 / 804 / 41 |

Each run writes a machine-readable report under `outputs/nav2/`. Generated
reports are intentionally not versioned; reproduce the table with the commands
below.

## 執行

需求是 ROS 2 Humble、Gazebo Fortress、Nav2、`ros_gz_bridge` 與
OpenCV ArUco。runner 自行產生 world/map、在臨時目錄 colcon build、隔離
ROS domain 與 Gazebo partition，並在 180 秒外層 timeout 內清理整個 process
group，避免 Nav2 lifecycle 子程序殘留。

快速切開「Nav2 設定」與「wheel odom／定位」問題：

```bash
./gazebo_sim/scripts/run_nav2_scenario.sh ground_truth pseudo
```

驗證接近真機的 TF 責任分工：

```bash
./gazebo_sim/scripts/run_nav2_scenario.sh wheel_odom pseudo
```

GUI 與 RViz：

```bash
./gazebo_sim/scripts/run_nav2_gui.sh wheel_odom pseudo
```

在 RViz 使用 Nav2 Goal 工具下目標。GUI 與 headless 使用同一份
`vgr_nav2_bringup`、地圖、topic remap、安全 filter 與 odometry nodes。

## TF 與定位

Nav2 使用標準樹：

```text
map -> odom -> base_link
```

- `ground_truth`：Gazebo `/sim/true_pose_raw` 經 adapter 發 `/odom` 與
  `odom -> base_link`；`map -> odom` 是 identity。
- `wheel_odom`：Gazebo wheel joint positions 經差速積分發 `/odom` 與
  `odom -> base_link`；ArUco chassis map pose 與最新 local odom 合成
  `map -> odom`。
- wheel odom 定頻 20 Hz 重發靜止 TF，避免車停時 TF cache 斷鏈。
- landmark localizer 定頻重發最後有效 correction；安全層仍直接看
  `/aruco/pose` age，marker stale 後照樣 STOP，兩者不能互相繞過。

wheel odometry 的純核心使用實車輪距 0.165 m、輪徑 0.065 m、左右
CPR 750/749，並覆蓋直行、原地旋轉、曲線、非遞增時間戳與 signed
32-bit rollover。模擬 joint sign 與真機 raw encoder sign 是兩個 adapter，
不混用。

## 地圖與障礙物

`nav2_integration/geometry.py` 是單一幾何來源，同時生成：

- `gazebo_sim/worlds/vgr_nav2.world`
- `ros2_ws/src/vgr_nav2_bringup/maps/vgr_nav2.pgm`
- `ros2_ws/src/vgr_nav2_bringup/maps/vgr_nav2.yaml`

既有 GS1–GS4 world 不變。Nav2 world 的固定障礙物仍擋住直線，但上下
走廊都能容納 0.23 m robot envelope 加 0.05 m clearance。Nav2 costmap
使用量測矩形 footprint `0.40 × 0.22 m`，只啟用 static 與 inflation
layers；沒有 LiDAR 時不宣稱能看見未知或移動障礙物。

驗收 clearance 使用旋轉矩形到牆／障礙物的真實最短距離，不用過度
保守的外接圓代替 collision footprint。PASS 必須同時滿足：

- action `SUCCEEDED`；
- position error ≤ 0.12 m；
- yaw error ≤ 0.25 rad；
- clearance ≥ 0.05 m；
- 軌跡確實從固定障礙物北側或南側通過；
- `/cmd_vel_nav`、`/cmd_vel_safe`、`/plan` 全都有 trace。

## 真實相機渲染與視覺覆蓋

在完整視覺模擬中，EGL rendering、camera bridge、marker detector 與定位精度均正常運作。
當標記位於視野內時，位置誤差 mean 0.021 m、yaw 誤差 mean 0.011 rad；當機器人在繞障轉向
期間短暫脫離地標覆蓋時，`safe_apf` 安全層能正確觸發限速與避障保護。

Nav2 模擬環境包含障礙物標記與專用地圖，用於驗證視覺間歇中斷時的導航與安全協同行為。

## 架構與實車移植

模擬堆疊架構直接對齊實車部署：

- `/cmd_vel_nav -> safety_gate -> /cmd_vel_safe`；
- `map -> odom -> base_link` 完整 TF 樹；
- wheel odometry 核心與運動學模型；
- 視覺定位即時修正 `map -> odom`，不干擾連續 local odom。
