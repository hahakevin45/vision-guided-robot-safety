"""大場地幾何 + 真實定位誤差模型的濾波器對抗掃描實驗（wrapper，不改核心）。

這個套件不修改 safety_sim 的模擬器核心或任何 filter。它重用核心零件
（World / DiffDriveVehicle / CommandLink / nav / filters / metrics），
只在外面套上：

1. 梯形（多邊形）大場地 —— 由 safety_sim.world.World 原生支援。
2. 定位誤差模型 FieldLocalizer —— 在原生 ArucoLocalizer 的高斯噪聲之外，
   額外注入「系統性偏差」與「盲段 pose_drift 不確定度成長」（核心不支援，
   屬本 wrapper 新增）。

詳見 docs/safety_sim_guide.md 與 outputs/sim_field_comparison/summary.md。
"""
