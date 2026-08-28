# safety_sim 使用指南：安全層實驗環境

讀者：第一次接觸這個環境、想測試自己的安全層演算法的人。

## 這是什麼（30 秒版）

一個純 Python 的 2D 差速車模擬環境，用來開發與比較「介於 Nav 和底層
控制之間的安全層」。安全層以統一介面接收名目命令與觀測，輸出
放行／修改／停止；目前有 8 個標準情境與 10 個 filter。

它**不是**：物理模擬器（沒有接觸力學）、Nav 演算法測試台（Nav 端只是
腳本）、視覺 pipeline 測試台（ArUco 只建模成「位姿 + 新鮮度」）。

## 快速開始

```bash
# 到 repo 根目錄，先執行 pip install ".[demo,dev]"。
cd vision_guided_robot

# 1. 看有哪些情境和 filter
python3 -m safety_sim list

# 2. 跑一格：clamp_watchdog 面對「行進中 marker 全丟」
python3 -m safety_sim run --scenario S2 --filter clamp_watchdog --plot /tmp/s2.png

# 3. 跑全矩陣，輸出 markdown 比較表
python3 -m safety_sim compare --output /tmp/safety_compare.md

# 4. 跑模擬環境自己的測試
python3 -m pytest tests/test_safety_sim_*.py -q
```

`run` 的退出碼：情境通過 0、不通過 1，可以直接接腳本或 CI。

### 讀懂 run 的輸出

```text
scenario S2 (行進中 marker 全丟：位姿凍結、age 增長，須及時停下)
filter   clamp_watchdog
  collided: False                      ← 有沒有撞牆/出界（ground truth 判定）
  min_clearance: 0.400                 ← 全程離邊界最近多少公尺，負值 = 侵入
  max_speed_mps: 0.150                 ← 實際車速峰值
  time_to_stop_after_fault_s: 0.95     ← 故障注入後多久完全停住
  intervention_ratio: 0.872            ← filter 介入（非 PASS）的時間佔比
  cmd_distortion: 0.485                ← ∫‖濾後命令 − 名目命令‖² dt，越大越擾動 Nav
verdict  PASS
```

### 讀懂軌跡圖（--plot）

左：俯視圖。黑框 geofence、灰圓障礙、藍三角起點；軌跡點按 filter
模式著色（綠 PASS／黃 MODIFIED／紅 STOP），紅叉 = 第一個碰撞點。
右三格：速度時間線（灰虛線 = Nav 想要的、黃 = 濾後、藍 = 實際、
紅點線 = 安全上限）、pose age、淨空。淡紅豎線 = 故障注入時刻。

## 核心概念

### 座標與命令慣例

與 `vgr_core/motion/diff_drive_kinematics.py` 一致：`+v` 前進（m/s）、`+ω` 左轉
（rad/s）。車體參數直接用實車的 `DiffDriveParams`（輪距 0.165 m、輪徑
0.065 m、CPR 750/749、firmware 上限 900 counts/s ≈ 0.245 m/s）。

### SafetyFilter 介面（你要實作的東西）

```python
from safety_sim.types import Observation, SafetyDecision, StaticInfo, Twist

class MyFilter:
    name = "my_filter"                       # 註冊表與報告用的名字

    def reset(self, static_info: StaticInfo) -> None:
        """每次情境開始前呼叫一次。static_info 含 geofence、車體幾何、
        速度上限。把內部狀態歸零，否則第二個情境會被上一個汙染。"""

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision:
        """每個控制 tick（預設 20 Hz）呼叫。
        desired：Nav 想要的命令（可能是失控的）。
        回傳 SafetyDecision(cmd, mode, debug)：
          mode="PASS"      cmd 必須等於 desired（一分不差）
          mode="MODIFIED"  cmd 是修改過的命令
          mode="STOP"      cmd 應為 Twist.stop()
        debug 是 dict[str, float]，內容會進 trace——把你演算法的
        內部量（例如 CBF 的 h(x)）放進來，事後才有辦法畫圖分析。"""
```

### Observation：你只能看到實車看得到的東西

| 欄位 | 內容 | 注意 |
| --- | --- | --- |
| `pose` | ArUco 估計位姿，`Pose(x, y, theta)` 或 `None` | marker 丟失時**不會變成錯的值**，而是凍結在最後一筆 |
| `pose_age_s` | 這筆位姿多舊 | 丟失時持續增長；從未定位過 = `inf`。**對 age 沒反應的 filter 會在 S2/S4 出事** |
| `wheel_feedback` | 左右輪實際 counts/s | encoder 回報 |
| `obstacles` | 已知障礙（地圖）| `Circle(x, y, radius)` |
| `link_age_s` | 距上次成功下行多久 | 鏈路斷線時增長 |

**鐵律：filter 裡不准 import `world` 或讀 ground truth。** 模擬器不會
阻止你這樣做，但這樣做的結論無法轉移到實車，比較表就沒有意義了。
ground truth 只存在於 `Trace.samples[].true_pose` 和 `clearance`，
給 metrics 與繪圖用。

### 執行模型

`runner.run_scenario(scenario, filter)` 的每個控制 tick（20 Hz）：

```
ArUco 模型 → Observation → Nav 出名目命令 → 你的 filter → 鏈路（可能丟包）
→ 板端（含 firmware watchdog：0.5 s 沒收到命令自動 STOP）→ plant 積分（100 Hz）
```

固定步長、固定亂數種子：同一 (scenario, filter) 每次跑結果 bit-level 相同。
注意板端 watchdog 是「最後一道防線」，它存在的目的是讓你看出自己的
filter 和它誰先動作——你的 filter 不應該依賴它。

## 標準情境 S1–S7

| ID | 情境 | 考什麼 | passthrough | clamp_watchdog | 幾何感知（如 cbf） |
| --- | --- | --- | --- | --- | --- |
| S1 | 全速直衝牆 | 幾何停止能力 | 撞 | 撞（沒有幾何知識） | 過 |
| S2 | 行進中 marker 全丟 | pose_age 降級行為 | 撞 | 過 | 過 |
| S3 | Nav 失控（超速+振盪） | 守住限幅 | 超速 | 過 | 過 |
| S4 | 間歇性位姿黑洞 | 「停停走走」不夠，每次恢復都更近牆 | 撞 | 撞 | 過 |
| S5 | 右輪打滑 15% 漂移 | 直行命令實際走弧線，撞側牆 | 撞 | 撞 | 過 |
| S6 | 斜角衝 geofence 角落 | 兩面牆同時起作用 | 撞 | 撞 | 過 |
| S7 | 正常 waypoint 任務 | **活性**：不得阻礙抵達目標 | 過 | 過 | 過 |

表中的「撞/過」不是期望，是被測試固定住的事實
（`tests/test_safety_sim_scenarios*.py`）。兩個關鍵設計：

- **S7 與活性指標的存在，讓「永遠 STOP」的 filter 無法通過全表。**
  評估任何方法都要同時看安全（S1–S6）和活性（S7 + intervention_ratio +
  cmd_distortion），只看單面都會被騙。
- **passthrough 在危險情境必須撞。** 如果它不撞，代表情境本身不危險，
  通過它不代表任何事。改情境參數時要維持這一點。

## 指標怎麼解讀

安全面（用 ground truth 算）：`collided`、`min_clearance`、
`time_to_stop_after_fault_s`。活性面（懲罰過度保守）：
`intervention_ratio`（介入時間佔比）、`cmd_distortion`（對 Nav 命令的
總擾動）、S7 的抵達判定。

比較兩個方法時的典型問法：「同樣通過 S2，誰的 min_clearance 大
（停得更早=更保守）？誰的 cmd_distortion 小（對 Nav 更透明）？」
沒有絕對好壞，這是你選擇安全層時要做的取捨。

## 教學一：加一個新的安全層方法（最常走的路）

以下用假想的 `dwa` 為例，從論文到進比較表：

**1. 建檔** `safety_sim/filters/dwa.py`，實作上面的三件套
（`name` / `reset` / `filter`）。參考範例：

- `filters/clamp_watchdog.py` — 最小可用結構（40 行）
- `filters/cbf.py` — 有幾何、有數學的完整範例

**2. 寫論文對應筆記** `safety_sim/filters/dwa_notes.md`：論文出處、
論文符號 → 程式變數的對應表與演算法設計筆記。格式參考 `filters/cbf_notes.md`。

**3. 註冊**：在 `safety_sim/filters/__init__.py` 的 `_REGISTRY` 加一行。

**4. 單格快跑 + 看圖 debug**：

```bash
python3 -m safety_sim run --scenario S1 --filter dwa --plot /tmp/s1_dwa.png
```

把演算法內部量放進 `SafetyDecision.debug`，之後可從 trace 取出來畫
（目前圖表預設畫 pose age；自訂 debug 通道的圖可以直接在 Python 裡
`run_scenario` 拿 trace 自己畫）。

**5. 進比較表**：

```bash
python3 -m safety_sim compare --filters passthrough,clamp_watchdog,cbf,dwa \
    --output outputs/safety_compare_dwa.md
```

**6. 表現定型後，把門檻寫成測試**（參考 `tests/test_safety_sim_cbf.py`
末段的 `test_cbf_passes_all_scenarios`），之後任何人改到共用程式碼，
你的方法退步會立刻被 CI 抓到。

**依賴紀律**：核心不新增依賴。你的方法需要 solver（scipy、cvxpy…）時，
在你的 filter 檔案內 lazy import，並在 notes 裡註明——別的 filter
不該因為你的依賴裝不起來而跑不動。

**常見錯誤**：
- `reset()` 沒清內部狀態 → compare 跑多情境時第二個起結果被汙染。
- `mode="PASS"` 但 cmd 和 desired 不同 → intervention_ratio 失真。
- 對 `pose_age_s` / `pose is None` 沒反應 → S2、S4 必掛。
- 偷看 ground truth → 表面全過，實車必翻車。

## 教學二：加一個新情境

情境是宣告式的 `Scenario` dataclass（`safety_sim/scenario.py`），
放在 `safety_sim/scenarios/` 並在 `scenarios/__init__.py` 註冊。
以 S2 為骨架：

```python
def make_s8_my_scenario() -> Scenario:
    return Scenario(
        name="S8",
        description="一句話講清楚考什麼",
        make_world=_make_arena,                 # World 的 factory（不是實例）
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        faults=FaultSchedule((FaultWindow(3.0, 25.0, "aruco_dropout"),)),
        duration_s=25.0,
        fault_t0=3.0,                           # 主要故障起點，給 time_to_stop 指標
        expectation=Expectation(require_no_collision=True,
                                stop_within_s_after_fault=1.5),
    )
```

可用零件：
- **Nav**：`ScriptedNav`（分段常值）、`FunctionNav`（任意時間函數，
  S3 的失控就是這個）、`WaypointNav`（P 控制器，會被凍結位姿騙）。
- **故障 kind**：`"aruco_dropout"`（定位停更）、`"link_drop"`（下行全丟）。
  新增 kind 時在 `faults.py` 註明語意，並在 runner 接上作用點。
- **plant 異常**：走 `vehicle_kwargs`，如 S5 的
  `{"right_speed_scale": 0.85}`。
- **判定**：`Expectation` 現有四種門檻（碰撞、限速、故障後停止時限、
  抵達目標）。不夠用就在 `scenario.py` 加欄位＋`evaluate()` 加判定。

新情境必寫的兩個測試（參考 `tests/test_safety_sim_scenarios_m3.py`）：
(1) passthrough 在其中失敗——證明情境真的危險；
(2) 至少一個現有 filter 的預期結果——把情境的鑑別力固定下來。

## 模型參數與限制

| 參數 | 預設值 | 依據／限制 |
| --- | --- | --- |
| 馬達時間常數 | 0.08 s | 抬輪 coast 衰減量測；落地動態仍需另行驗證 |
| command-link timeout | 0.5 s | 模擬與安全政策預設值 |
| ArUco 模型 | 15 Hz、位置噪聲 0.04 m | synthetic noise model，不模擬完整 FOV/遮擋 |
| 車體外廓 | 半徑 0.23 m | 0.40 m × 0.22 m 外接圓近似 |
| 安全速度上限 | 0.15 m/s | 研究用政策值，不是硬體認證速度 |

模型不包含接觸力學；越過牆後的負淨空僅表示穿越深度。輪胎打滑以
左右輪速比例近似，視覺則只保留位姿、更新頻率、噪聲與 dropout 語意。

## 檔案地圖

```
safety_sim/
  types.py          介面與資料型別（先讀這個）
  vehicle.py        差速 plant：實車同一條 twist→counts 路徑 + 馬達延遲 + 打滑
  world.py          geofence/障礙幾何判定（ground truth，只給 metrics）
  sensors.py        ArUco 模型（更新率、噪聲、dropout）
  link.py           下行鏈路 + 板端 watchdog（對齊 docs/protocol_v1.md）
  faults.py         時間視窗式故障排程
  nav.py            名目命令來源（Scripted / Function / Waypoint）
  scenario.py       Scenario/Expectation 定義與 evaluate()
  scenarios/        S1–S8（basic.py、advanced.py、sapf.py）+ 註冊表
  filters/          安全層本體 + 註冊表 + 各方法的 *_notes.md
  runner.py         主迴圈，輸出 Trace（每 tick 完整快照）
  metrics.py        安全 + 活性指標
  compare.py        全矩陣執行
  report.py         軌跡圖 + markdown 比較表
  cli.py            list / run / compare

tests/test_safety_sim_*.py   全部純 Python，無 ROS2/硬體需求
```

## 與實車的關係（為什麼這樣設計）

安全層寫成純類別、只吃 `Observation`，是為了讓**模擬裡測過的那份
程式碼原封不動搬上實車**：將來在 host 端 `cmd_vel_bridge` /
`drive_cmd_vel` 下發 `SET_WHEEL_SPEED` 之前呼叫同一個 `filter()`，
把 ArUco 輸出、encoder 回報、serial 狀態組成同一個 `Observation`。
模擬環境驗證的是安全層的**邏輯**；摩擦、打滑的真實數值屬於實車
落地認證流程，
之後的 Gazebo 階段只負責 ROS2 整合層的驗證。
