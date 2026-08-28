# aruco_detector 座標鏈說明

## 輸入真值來源

- `gazebo_sim/models/markers/marker_map.json` 是 marker ID、世界座標中心點、
  yaw 與邊長的唯一來源。
- `gazebo_sim/models/vgr_diff_drive/camera_info.json` 是 pinhole camera 內參的
  唯一來源。
- 相機在車體上的位姿使用
  `gazebo_sim.generators.generate_robot_sdf.CAMERA_FRONT_X_M` 與
  `CAMERA_HEIGHT_M`，避免在 detector 另抄一份數字。

## 座標慣例

world / chassis 使用 Gazebo 平面車慣例：

- `+x` 向前，`+y` 向左，`+z` 向上。
- `theta` 是繞 `+z` 的 yaw，`theta=0` 時車頭朝世界 `+x`。
- camera 安裝在 chassis 前方，無額外 roll/pitch/yaw；目前只使用
  `(x, y, z, yaw)`。

OpenCV camera optical frame 使用：

- `+X` 為影像右方。
- `+Y` 為影像下方。
- `+Z` 為鏡頭前方。

因此 chassis/world 向量轉成 optical frame 時，相機前方對應車體 `+x`，
影像右方對應車體 `-y`，影像下方對應車體 `-z`。

程式中顯式使用 body-from-optical 旋轉：

```text
R_body_optical = [[ 0,  0,  1],
                  [-1,  0,  0],
                  [ 0, -1,  0]]
```

`cv2.solvePnP` 回傳的是 world-to-optical 變換；反矩陣取得
optical-to-world 後，再用 `R_body_optical.T` 還原 camera body 的 forward
axis 來取 yaw。這避免把 Gazebo sensor pose 的 `+x` 前方慣例和 OpenCV
optical `+Z` 前方慣例混在同一個 frame 裡。

## marker 四角順序

`cv2.aruco.detectMarkers` 回傳角點順序為：

1. 左上
2. 右上
3. 右下
4. 左下

`ArucoWorldLocalizer` 餵給 `cv2.solvePnP` 的 3D object points 使用同一順序。
marker map 的 `yaw` 表示 marker 法向量朝向場內：

- normal = `(cos(yaw), sin(yaw), 0)`
- image right = `(-sin(yaw), cos(yaw), 0)`
- image down = `(0, 0, -1)`

Gazebo marker 貼圖不是整張面都屬於 OpenCV 回傳的 ArUco 角點：資產產生器
把 `size_m` 方形面切成 10 個模組，其中外圈是白色 quiet zone，實際可偵測
的 ArUco 黑色外框為 8 個模組。因此 `ArucoWorldLocalizer` 餵給
`cv2.solvePnP` 的 object points 使用 `size_m * 0.8` 展開，而不是整張
貼圖面的 `size_m`。這和真 Gazebo 渲染 fixture 一致；合成影像測試也必須
先把含 quiet zone 的完整貼圖投影到 `size_m` 面上，再由 detector 偵測內層
ArUco 角點。

`solvePnP` 回傳 world-to-optical 變換，核心再反矩陣取得 camera world
pose。

## camera 到 chassis

PnP 解出的 camera world pose 包含 camera 原點與 camera forward yaw。
因 camera yaw_on_robot 目前為 0，chassis yaw 等於 camera yaw。chassis 原點：

```text
chassis_xy = camera_xy - Rz(chassis_yaw) * camera_xy_on_robot
```

若未來相機有非零 yaw，只要更新 `camera_pose_on_robot` 的第四個值即可。

## 多 marker 融合

每個 marker 先獨立解出 chassis pose，再融合。位置使用影像 marker 面積的
立方作為權重平均，yaw 用同一權重加總 `(sin, cos)` 後取 `atan2`。面積權重
讓近距離、角點解析度較高的 marker 主導，避免小而斜的遠端 marker 在等權
平均時造成系統性偏移。

## 實作設計特點

- 單目 pinhole camera，distortion coefficients 固定為 0。
- marker 位姿取自靜態 map，沒有線上校正。
- 多個 marker 同時可見時，目前只依影像面積加權，尚未納入 reprojection
  error 或觀測角度。
- 偵測不到 marker 或偵測到的 ID 不在 map 時不發布姿態；下游依
  `pose_age_s` 自然長大處理 dropout。
