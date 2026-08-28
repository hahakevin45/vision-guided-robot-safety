# cbf filter：論文對應與簡化

## 出處

- A. D. Ames, X. Xu, J. W. Grizzle, P. Tabuada,
  "Control Barrier Function Based Quadratic Programs for Safety Critical
  Systems," IEEE TAC, 2017.（CBF-QP 框架本體）
- Look-ahead 點處理 diff-drive 非完整性的技巧常見於 CBF 應用文獻，
  等價於對 unicycle 做 near-identity diffeomorphism / feedback point offset。

## 符號對應

| 論文 | 程式 |
| --- | --- |
| 狀態 x | `obs.pose`（估計位姿，不是 ground truth） |
| barrier h(x) ≥ 0 | 每條 geofence 邊：`h = n̂·(p − q) − margin`；每個圓障礙：`h = ‖p − c‖ − r − margin` |
| 控制 u | `(v, ω)` |
| CBF 條件 ḣ ≥ −α(h) | `a_v·v + a_ω·ω ≥ −α·h`，α 取線性 class-K |
| L_g h | `(a_v, a_ω)`，經 look-ahead 點 `p = pose + l·e_θ` 得到（`_build_constraints` 的 `add()`） |
| QP：min ‖u − u_des‖² s.t. CBF | 逐約束半平面投影迭代（`_project`），可行集為空 → STOP |

## 工程實現與計算優化

1. **QP → 迭代投影**：標準做法是解 QP；這裡用「挑最違反的約束、
   投影到其半平面」迭代 30 次。多約束同時起作用（角落）時不保證
   收斂到 QP 最優解，只保證輸出可行或 STOP。之後可換 OSQP/cvxpy
   對照，依賴 lazy import 進本檔即可。
2. **一階運動學模型**：CBF 條件用 unicycle 運動學推導，馬達
   一階延遲（τ≈0.08s）以 `buffer_m` 安全裕度吸收。
3. **位姿噪聲/延遲未進理論**：h 用估計位姿直接算，屬 measurement-robust
   CBF 未處理的部分；同樣靠 buffer 吸收，靠 S2/S4 情境實測把關。
4. **速度盒夾制在投影之後**：夾回上限可能重新違反 CBF 條件，
   此時直接 STOP（保守但安全）。

## 關鍵參數設定

- `alpha = 1.0`：逼近邊界時 v 上限 ≈ α·h，決定煞車曲線陡峭度。
- `lookahead_m = 0.10`：太小 → ω 對 h 的影響消失；太大 → 過度保守。
- `buffer_m = 0.08`：吸收馬達延遲 + 位姿誤差的總裕度。
