# oscrs — OSCRS (Online Slosh-Constrained Reference Selector) 模块化实现
#
# 层级:
#   G (generators/)  — 候选生成
#   F (feasibility)   — 几何/碰撞/执行可行性门
#   R (slosh_rollout) — 晃动动力学 rollout 与指标
#   S (selector)      — hard gate + score + selection + fallback
#   diagnostics       — candidate_report / safety_alarm / metrics 格式化
