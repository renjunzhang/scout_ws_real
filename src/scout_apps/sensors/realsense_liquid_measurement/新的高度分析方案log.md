# 新的高度分析方案 Log

## 当前核心目标

**当前主目标：验证 `/slosh/height` 能否估计“当前时刻的主液面高度”。**

- 当前主观测口径：
  - `height_center_rel_mm_bias_corrected_v2`
- `peak` 与 `/slosh/height_pred_max` 不作为本阶段主结论口径
- 后续所有日志结论，默认优先参考：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/`

## 当前效果与判断

- 当前最新主基线：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/`
- 当前主结论：
  - `/slosh/height` 已经可以作为“当前时刻主液面高度”的工程估计量使用
  - 但它还不能当绝对真值标准
- 当前最能代表效果的指标：
  - `Q5_static`: `raw_MAE = 0.007837 mm`
  - `Q5_test1`: `raw_MAE = 0.093253 mm`
  - `Q5_test2`: `raw_MAE = 0.092495 mm`
  - `Q5_test3`: `raw_MAE = 0.063274 mm`
  - `Q0_test1`: `raw_MAE = 0.189001 mm`
- 当前限制：
  - `Q5_test1` 仍有少量模型侧局部低估段
  - 视觉 `center` 也仍有少量近静态异常帧
  - `Q0` 与 `Q5` 运动 bag 的路径和激励不严格可比，不能直接据此下“Q=5 必然更优”的强结论

## 说明

- 这份日志只记录“新高度分析方案”的修改、决策、验证和结论。
- 不记录旧方案的迭代细节。
- 每次新增内容时，优先回答：
  - 做了什么
  - 为什么做
  - 结果如何
  - 下一步是什么

## 2026-03-25

### 初始化

- 新建 [新的高度分析方案.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/新的高度分析方案.md)
- 确定新方案与当前方案彻底分离
- 确定新方案主线：
  - 多点标尺
  - `y_px -> h_mm` 高度映射
  - 不再依赖单一 `mm_per_pixel`
- 约定新方案数据目录：
  - `/data/a/realsense_validation_v2/`

### 当前结论

- 当前方案可继续作为证据链使用，但毫米换算方法不够好。
- 新方案优先解决尺度映射问题，而不是继续在当前 `29 mm` 单点比例上打补丁。

### 下一步

- 先设计 `annotate_height_ruler_v2.py` 的输入和输出格式
- 再实现 `extract_liquid_height_v2_from_bag.py` 的分段高度映射

### 文档重构

- 将 [新的高度分析方案.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/新的高度分析方案.md) 重写为自包含版本
- 新增：
  - 新对话入口摘要
  - 任务定义
  - 当前相关文件
  - 当前数据集与目录约定
  - 接口设计
  - 脚本职责规划
  - 验收标准
  - 下一步唯一推荐动作

### 当前结论补充

- 这份新方案文档现在应被视为新方案的主入口文档。
- 新开对话时，应优先阅读这份文档，再决定是否打开旧方案 README 或旧改进文档。

### v1 接口冻结

- 冻结 `0 mm` 定义：
  - `静止液面 = 0 mm`
- 冻结高度映射坐标系：
  - 使用旋正后 ROI 坐标
- 冻结 `x_px` 语义：
  - 只做记录和一致性检查，不参与 `F(y)` 计算
- 冻结标尺点点击口径：
  - 点击刻线中心
- 冻结 `annotate_height_ruler_v2.py` 交互方式：
  - 通过 `--ruler-heights-mm 0,5,10,...` 给定高度序列
  - 用户按顺序点击，不逐点手输高度

### 这样做的原因

- 避免在实现 `v2` 标尺脚本前接口继续摇摆
- 避免后面在 `0 mm`、坐标系和点法上返工

### 实现 `annotate_height_ruler_v2.py`

- 新增 [annotate_height_ruler_v2.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_height_ruler_v2.py)
- 复用旧标注脚本的主流程：
  - 先标 `ROI`
  - 再标 `left/right wall line`
  - 再标 `still_level_line`
  - 再标 `tube_axis_line`
- 新增 `v2` 标尺点流程：
  - 通过 `--ruler-heights-mm` 预先给定高度序列
  - 用户按顺序点击对应刻线中心
  - 不在点击过程中逐点输入 `h_mm`
- 新脚本输出的 YAML 已按 `v2` 口径写入：
  - `geometry_roi`
  - `height_mapping.reference_points`
  - `reference_points_image.ruler_points`

### 这次实现的关键点

- `height_mapping.reference_points` 使用旋正后 ROI 坐标
- `geometry_roi` 仍保存原始 ROI 相对坐标，不和旋正坐标混用
- `x_px` 不参与映射，只用于记录和一致性检查
- 新增标尺点 `x` 向离散警告：
  - 如果旋正后各点 `x_px` 偏差过大，保存前给出警告
- 新增单调性硬检查：
  - 如果 `h_mm` 递增但旋正后 `y_px` 不递减，则拒绝保存

### 当前结果

- `annotate_height_ruler_v2.py` 已能作为 `P2` 的独立入口脚本使用
- 已完成最小本地验证：
  - `python3 -m py_compile`
  - `python3 annotate_height_ruler_v2.py --help`
- 还没有做真实图片上的人工交互回归

### 下一步

- 用一张静止参考图实际跑一次 `annotate_height_ruler_v2.py`
- 检查导出的 `frame_000000_multiscale_raw.yaml` 是否满足文档字段约定
- 再开始实现 `extract_liquid_height_v2_from_bag.py` 中的 `piecewise_linear` 映射

### 实现 `extract_liquid_height_v2_from_bag.py`

- 新增 [extract_liquid_height_v2_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_v2_from_bag.py)
- 实现策略不是复制旧提取脚本，而是薄封装复用旧检测链：
  - 继续使用旧的 ROI 旋正、阈值、edge fallback、temporal gate
  - 新脚本只负责：
    - 读取 `v2` multiscale YAML
    - 解析 `height_mapping.reference_points`
    - 计算 `piecewise_linear` 的 `F(y_rect) -> h_mm`
    - 输出 `v2` CSV / debug video / plot

### 这次实现冻结的口径

- `height_peak_rel_mm_v2`、`height_left_rel_mm_v2`、`height_right_rel_mm_v2`
  - 全部由 `height_mapping.reference_points` 计算
  - 不再依赖旧的 `mm_per_pixel`
- `height_*_rel_px_*_v2`
  - 使用 `h_mm = 0` 对应的 `zero_y_rect`
  - 不再偷偷回退到旧 `still_level_px` 作为 `v2` 零点
- `y_peak_raw_rect_v2`
  - 直接使用旋正后 ROI 的峰值候选行
- `y_peak_raw_v2`
  - 通过逆变换回原始 ROI 坐标，仅用于调试和溯源

### 当前结果

- 新脚本已通过最小本地验证：
  - `python3 -m py_compile`
  - `python3 extract_liquid_height_v2_from_bag.py --help`
- 已完成静止包烟雾测试：
  - bag: `/data/a/slosh_bags/TestOnRealScale/slosh_QQ5_static_20260324_190041.bag`
  - calibration: `/data/a/realsense_validation_v2/calibration/TestOnRealScale/frame_000000_multiscale_raw.yaml`
  - `--max-frames 8`
  - `--max-frames 80`
- 烟雾测试结果：
  - `v2` 脚本能稳定生成 `liquid_height_v2.csv`
  - 当前静止包上 `valid_v2` 很低
  - 这不是 `v2` 新映射单独导致的问题，因为旧脚本在同一静止包上的 `valid` 也同样很低

### 当前判断

- `P3` 的“v2 提取链 + piecewise_linear 映射”已经打通
- `P4` 的下一步瓶颈不是映射接口，而是静止包上的检测通过率/门控策略

### 下一步

- 先用完整静止包输出检查：
  - `height_peak_rel_mm_v2` 的分布
  - `valid_v2 / accept_for_peak_report_v2` 的比例
- 然后单独处理检测链问题：
  - 判断是否要放宽 `valid/reportable` 门槛
  - 或把 `v2` 的主报告口径从旧 gate 中解耦

### 新增 `liquid_measurement_v2.yaml`

- 新增 [liquid_measurement_v2.yaml](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/liquid_measurement_v2.yaml)
- 原因不是继续“拍脑袋放阈值”，而是静止包 CSV 已经表明：
  - 旧 `v1` gate 在 `v2` 静止包上根本过不去
  - `central_valid_columns` 是 `valid_v2` 的主瓶颈
  - `peak_local_rms_px` 的硬阈值也明显过严

### 这次配置迭代

- 第 1 轮：
  - 缩小 `central_band_ratio`
  - 放宽 `report_min_central_coverage`
  - 放宽 `report_max_peak_local_rms_px`
  - 放宽 `report_min_confidence`
- 第 2 轮：
  - 进一步把 `central_band_ratio` 收窄到更接近真实有效中心带
  - 把 `report_min_confidence` 拉回更保守的值
  - 保留较宽的 `peak_local_rms_px` 上限，避免它继续把高置信候选一刀切掉

### 静止包复跑结果

- 输出目录：
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/`
- 第 2 轮配置下：
  - `processed frames = 476`
  - `valid_v2 = 1`
  - `accept_for_peak_report_v2 = 1`
  - `max height_peak_rel_mm_v2 = 1.744`

### 当前判断更新

- 好消息：
  - `reportable` 终于从 `0` 变成了 `1`
  - 说明 `v2` gate 的方向是对的，不是完全死路
- 坏消息：
  - `1 / 476` 仍然太低，静止包还远没达到可用
- 进一步拆 CSV 后的结论：
  - 绝大多数“接近可用”的帧卡在 `central_valid_columns`
  - 很多高置信候选的 `central_valid_columns` 只有 `18~19`
  - 而当前 `valid_v2` 过线所需刚好在 `20` 左右

### 下一步更新

- 下一步不该再碰高度映射
- 下一步应该二选一：
  - 继续收窄 `central_band_ratio`
  - 或把 `valid_v2` 从当前旧式 central-band gate 中解耦，改成更贴近 `v2` 主报告目标的有效性定义

## 2026-03-25: `valid_v2` 从旧 gate 解耦，但保持 detector 内部时序行为不变

### 背景

- 静止包 CSV 已经说明，真正卡住 `v2` 的不是 `piecewise_linear` 映射，而是旧 detector 的 `valid`：
  - 只要去掉 `central_valid_columns` 那条旧硬门槛，静止包里原本就有一批候选能通过 `report` 条件
- 我先试过把 `previous_state` 也改成跟新 `valid_v2` 走，但前 `80` 帧烟雾测试立刻退化成：
  - `valid_v2 = 1`
  - `accept_for_peak_report_v2 = 1`
- 这说明“连 detector 内部状态一起解耦”不是最短路径，会把时序行为变化混进当前问题

### 本次代码调整

- 修改 [extract_liquid_height_v2_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_v2_from_bag.py)
- 新增 `evaluate_v2_gates(...)`
  - `valid_v2` 改为基于：
    - `valid_columns`
    - `fit_points`
    - `fit_rms_px`
    - `fit_slope`
    - `peak/left/right y` 在 ROI 内
    - `temporal_gate_passed`
  - 不再直接等于旧 detector 的 `valid`
- `accept_for_peak_report_v2`
  - 仍沿用已有 `report_*` 阈值
  - 但不再依赖旧 detector 的 `valid`
- 保留旧 gate 结果作为诊断列：
  - `legacy_valid_gate_v2`
  - `legacy_accept_for_peak_report_gate_v2`
- detector 的 `previous_state`
  - 仍然只跟旧 detector 的 `valid` 更新
  - 避免这轮把“门槛定义”问题和“时序状态行为”问题混在一起

### 验证

- 本地检查：
  - `python3 -m py_compile`
  - `python3 extract_liquid_height_v2_from_bag.py --help`
- 静止包烟雾测试（前 `80` 帧）：
  - 新 `v2` gate：`valid_v2 = 23`，`accept_for_peak_report_v2 = 23`
  - 旧 gate：`legacy_valid_gate_v2 = 1`，`legacy_accept_for_peak_report_gate_v2 = 1`
- 静止包全量复跑：
  - bag: `/data/a/slosh_bags/TestOnRealScale/slosh_QQ5_static_20260324_190041.bag`
  - output: `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/`
  - `processed frames = 476`
  - `valid_v2 = 23 (4.8%)`
  - `legacy_valid_gate_v2 = 1 (0.2%)`
  - `accept_for_peak_report_v2 = 23`
  - `legacy_accept_for_peak_report_gate_v2 = 1`

### 结果判断

- 这轮改动已经把根因钉住了：
  - `v2` 之前几乎“没帧可用”，主要是旧 central-band gate 在卡
  - 不是多尺度映射接口本身坏了
- 但静止包的 `height_peak_rel_mm_v2` 仍不接近 `0`：
  - `reportable` 帧范围约 `1.625 ~ 1.744 mm`
- 所以下一步重点应该从“门槛是否过得去”切到：
  - `peak` 定义是否和 `0 mm = 静止液面` 口径一致
  - 是否需要 `peak/center` 双口径输出
  - 或是否需要单独的 `v2 auto-zero`

## 2026-03-25: 增加 `center` 输出，验证静止包偏差主要来自 `peak` 口径

### 本次代码调整

- 修改 [extract_liquid_height_v2_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_v2_from_bag.py)
- 在 `v2` CSV 中新增：
  - `y_center_raw_rect_v2`
  - `height_center_rel_px_signed_v2`
  - `height_center_rel_px_v2`
  - `height_center_rel_mm_signed_v2`
  - `height_center_rel_mm_v2`
- debug overlay 也同步增加 `center_rel_mm`
- 这一步没有修改 detector 本体，只是把旧 detector 已经算好的 `center_y` 一并走 `v2` 映射链输出

### 验证

- 本地检查：
  - `python3 -m py_compile`
- 静止包小样本烟雾测试：
  - 前 `40` 帧已确认新 CSV 列存在
  - `height_center_rel_mm_v2` 已正常写出
- 静止包全量复跑：
  - output: `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/`
  - `reportable_count = 23`
  - `peak`:
    - `min = 1.624797 mm`
    - `median = 1.624797 mm`
    - `mean = 1.671396 mm`
    - `max = 1.743883 mm`
  - `center`:
    - `min = 0.076668 mm`
    - `median = 0.076668 mm`
    - `mean = 0.076668 mm`
    - `max = 0.076668 mm`

### 结果判断

- 这个结果已经把问题进一步收窄了：
  - 静止包里 `center` 基本贴近 `0 mm`
  - 当前明显偏高的是 `peak`
- 所以当前主矛盾不是整体零位定义错了，也不是 `piecewise_linear` 映射整体漂了
- 更合理的结论是：
  - `peak` 口径描述的是“左右两侧较高 meniscus 候选中的最高点”
  - 这一天然会高于“静止液面中心线”
- 因而下一步不应优先做 `v2 auto-zero`
- 下一步更合理的是先冻结双口径：
  - `height_peak_rel_mm_v2`
  - `height_center_rel_mm_v2`
- 然后再决定主报告口径到底以哪个为准

## 2026-03-25: 冻结双口径文档口径，并实现 `compare_realsense_vs_mpc_slosh_v2.py`

### 文档更新

- 更新 [新的高度分析方案.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/新的高度分析方案.md)
- 明确冻结双口径：
  - `height_center_rel_mm_v2`
  - `height_peak_rel_mm_v2`
- 同时把这些内容写进方案正文：
  - 静止包优先用 `center` 判断零位
  - `center` 默认对齐 `/slosh/height`
  - `peak` 默认对齐 `/slosh/height_pred_max`
  - 静止包不再要求 `peak` 必须接近 `0`

### 新增脚本

- 新增 [compare_realsense_vs_mpc_slosh_v2.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/compare_realsense_vs_mpc_slosh_v2.py)
- 功能：
  - 读取 `liquid_height_v2.csv`
  - 同时加载：
    - `height_center_rel_mm_v2`
    - `height_peak_rel_mm_v2`
  - 对齐：
    - `center` vs `/slosh/height`
    - `peak` vs `/slosh/height_pred_max`
  - 额外输出交叉诊断误差：
    - `center` vs `pred_max`
    - `peak` vs `height`
  - 输出：
    - `mpc_realsense_comparison_v2.png`
    - `mpc_realsense_aligned_v2.csv`

### 验证

- 本地检查：
  - `python3 -m py_compile`
  - `python3 compare_realsense_vs_mpc_slosh_v2.py --help`
- 静止包验证：
  - bag: `/data/a/slosh_bags/TestOnRealScale/slosh_QQ5_static_20260324_190041.bag`
  - liquid csv: `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/liquid_height_v2.csv`
- 已生成正式输出：
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/mpc_realsense_comparison_v2.png`
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/mpc_realsense_aligned_v2.csv`

### 当前结果

- 默认启用初始零对齐后：
  - `center` offset 约 `0.077 mm`
  - `peak` offset 约 `1.625 mm`
- 零对齐后的静止包结果：
  - `center vs /slosh/height`
    - `MAE = 0.000`
    - `RMSE = 0.000`
  - `peak vs /slosh/height_pred_max`
    - `MAE = 0.047`
    - `RMSE = 0.074`

### 结果判断

- 到这里，双口径主线已经真正落地：
  - 提取链能产出双口径
  - 对比链能分别消费双口径
- 当前最合理的下一步不再是讨论学习模型
- 当前最合理的下一步是进入运动包：
  - `Q0_test1`
  - `Q5_test1`

## 2026-03-25: 运行 `Q0_test1 / Q5_test1`，完成双口径运动包提取与对比

### 提取结果

- 使用同一份 `multiscale` 标定：
  - `/data/a/realsense_validation_v2/calibration/TestOnRealScale/frame_000000_multiscale_raw.yaml`
- `Q0_test1`
  - bag: `/data/a/slosh_bags/TestOnRealScale/slosh_QQ0_test1_20260324_185558.bag`
  - output: `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q0_test1/`
  - `processed frames = 1324`
  - `valid_v2 = 244 (18.4%)`
  - `legacy_valid_gate_v2 = 201 (15.2%)`
  - `accept_for_peak_report_v2 = 121`
  - `legacy_accept_for_peak_report_gate_v2 = 95`
- `Q5_test1`
  - bag: `/data/a/slosh_bags/TestOnRealScale/slosh_QQ5_test1_20260324_185840.bag`
  - output: `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_test1/`
  - `processed frames = 1374`
  - `valid_v2 = 73 (5.3%)`
  - `legacy_valid_gate_v2 = 20 (1.5%)`
  - `accept_for_peak_report_v2 = 20`
  - `legacy_accept_for_peak_report_gate_v2 = 2`

### 原始双口径量级

- `Q0_test1`
  - `peak`:
    - `min = 1.029362 mm`
    - `median = 1.743883 mm`
    - `max = 1.862970 mm`
  - `center`:
    - `min = 0.000000 mm`
    - `median = 0.148211 mm`
    - `max = 0.319758 mm`
- `Q5_test1`
  - `peak`:
    - `min = 1.505710 mm`
    - `median = 1.743883 mm`
    - `max = 1.743883 mm`
  - `center`:
    - `min = 0.076668 mm`
    - `median = 0.076668 mm`
    - `max = 0.528443 mm`

### 对比结果

- 已生成正式输出：
  - `Q0_test1`
    - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q0_test1/mpc_realsense_comparison_v2.png`
    - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q0_test1/mpc_realsense_aligned_v2.csv`
  - `Q5_test1`
    - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_test1/mpc_realsense_comparison_v2.png`
    - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_test1/mpc_realsense_aligned_v2.csv`
- 默认启用初始零对齐后：
  - `Q0_test1`
    - `center vs /slosh/height`
      - `MAE = 0.045`
      - `RMSE = 0.072`
    - `peak vs /slosh/height_pred_max`
      - `MAE = 0.167`
      - `RMSE = 0.556`
  - `Q5_test1`
    - `center vs /slosh/height`
      - `MAE = 0.008`
      - `RMSE = 0.023`
      - `Corr = 0.997`
    - `peak vs /slosh/height_pred_max`
      - `MAE = 0.125`
      - `RMSE = 0.558`

### 当前判断

- 到这一步，方案主线已经足够清楚：
  - `center` 口径在静止包和 `Q5_test1` 上都明显更像 `/slosh/height`
  - `peak` 口径目前还不像 `/slosh/height_pred_max` 的稳定代理
- 因此当前不该先切去监督学习
- 当前更合理的顺序是：
  1. 保持双口径冻结不动
  2. 继续把 `center` 作为当前主液面口径
  3. 单独重审 `peak` 的物理定义和候选逻辑

## 2026-03-25: 明确 `center` 是当前主液面口径，并记录 `peak` 的现定义

### 文档与对比口径更新

- 更新 [新的高度分析方案.md](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/新的高度分析方案.md)
- 明确冻结：
  - 当前主液面口径 = `height_center_rel_mm_v2`
  - `height_peak_rel_mm_v2` 作为次级诊断量保留
- 更新 [compare_realsense_vs_mpc_slosh_v2.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/compare_realsense_vs_mpc_slosh_v2.py)
  - 图标题、图例、终端输出都明确：
    - `center` 是 main liquid level
    - `peak` 是 diagnostic

### `peak` 现定义的代码事实

- 现有 detector 中，`center_y` 来自中心带拟合线：
  - `center_y = center_fit_y`
- 但 `peak_y` 不是“全宽最高点”，也不是“中心附近最高点”
- 当前 `peak_y` 的实际定义链是：
  1. 先按 `side_band_bounds(...)` 划左右两条 side-band
  2. 每条 band 内用 `robust_band_peak(...)` 找各自的顶部候选
     - 具体做法不是单列极值
     - 而是对 band 内较高一批 `y` 候选取中位数
  3. 左右各得到一个候选：
     - `left_y`
     - `right_y`
  4. 再比较：
     - `left_rel_px_signed`
     - `right_rel_px_signed`
  5. 取抬升更高的一侧作为：
     - `peak_y`
     - `peak_source`

### 当前判断更新

- 这说明当前 `peak` 更准确的语义不是“主液面最高点”
- 更接近：
  - “左右 side-band 中较高的 meniscus 候选”
  - 或“侧边峰值包络代理”
- 这也解释了为什么：
  - `peak` 在静止包上稳定正偏
  - `peak` 和 `/slosh/height_pred_max` 的一致性仍然一般

### 下一步边界

- 下一步如果重审 `peak`，目标不应是再改 `center`
- 应该只回答这一个问题：
  - 当前 `peak` 是继续保留为 side-band envelope，还是改成新的峰值定义

## 2026-03-25: 正式产物改为 `center` 主线展示，并刷新 `/data/...` 输出

### 代码调整

- 更新 [extract_liquid_height_v2_from_bag.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_v2_from_bag.py)
  - `liquid_height_peak_curve_v2.png` 继续沿用旧文件名
  - 但图内容已改为：
    - `center` 主液面曲线
    - `peak` 诊断曲线
  - 终端摘要也新增：
    - `max height_center_rel_mm_v2`

### 验证

- 本地检查：
  - `python3 -m py_compile`
- 本地烟雾测试：
  - `Q5_static --max-frames 40`
  - 已确认新摘要图生成成功
- 已刷新正式提取产物：
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/liquid_height_peak_curve_v2.png`
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q0_test1/liquid_height_peak_curve_v2.png`
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_test1/liquid_height_peak_curve_v2.png`
- 已刷新正式对比产物：
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_static/mpc_realsense_comparison_v2.png`
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q0_test1/mpc_realsense_comparison_v2.png`
  - `/data/a/realsense_validation_v2/verify/TestOnRealScale/Q5_test1/mpc_realsense_comparison_v2.png`

### 结果判断

- 到这里，正式产物的口径已经完全一致：
  - `center` 是当前主液面
  - `peak` 是诊断量
- 下一步不需要再改展示层
- 下一步只需要决定：
  - `peak` 保留现 side-band envelope 语义
  - 还是进入新的峰值定义设计

## 2026-03-25: 新增 `v2` 逐帧交互调试脚本，并构建 `Q5_test1` 正式缓存

### 新增脚本

- 新增 [debug_liquid_vs_mpc_frame_by_frame_v2.py](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/debug_liquid_vs_mpc_frame_by_frame_v2.py)
- 设计目标只针对 `v2`：
  - 突出显示 RealSense 视觉判断的最高点
  - 同步显示 `/slosh/height` 与 `/slosh/height_pred_max`
  - 提供人工输入的人眼晃动高度标签
  - 将人工标签保存为后续监督学习和误差分析可直接使用的 CSV

### 当前交互重点

- ROI 调试图中显式标出：
  - `RS visual peak`
  - `main center`
- 右侧信息面板重点显示：
  - `main center`
  - `RS visual peak`
  - `/slosh/height`
  - `human label`
- 底部有人工输入框：
  - `i` 进入编辑
  - 输入数值（单位 `mm`）
  - `Enter` 保存
  - `x` 清空当前标签
- 输出文件：
  - `debug_session.json`
  - `debug_session.csv`
  - `human_labels.csv`

### 验证

- 本地检查：
  - `python3 -m py_compile`
  - `python3 debug_liquid_vs_mpc_frame_by_frame_v2.py --help`
- 本地烟雾测试：
  - `/tmp/realsense_v2_frame_debug_smoke/`
  - `--max-frames 8 --skip-viewer`
- 已构建正式缓存：
  - `/data/a/realsense_validation_v2/debug/TestOnRealScale/Q5_test1/debug_session.json`
  - `/data/a/realsense_validation_v2/debug/TestOnRealScale/Q5_test1/debug_session.csv`
  - `/data/a/realsense_validation_v2/debug/TestOnRealScale/Q5_test1/human_labels.csv`

### 当前判断

- 到这里已经可以开始 `v2` 的逐帧人工审阅
- 当前更合理的逐帧调试顺序是：
  1. 先看 `Q5_test1`
  2. 重点看：
     - `main center` vs `/slosh/height`
     - `RS visual peak` 是否只是稳定边缘偏置
  3. 一边录入 `human_height_mm`，为后续监督学习保留标注数据
## 2026-03-25 debug-v2 键盘兼容修复

- 现象：`debug_liquid_vs_mpc_frame_by_frame_v2.py` 在部分 OpenCV GTK/Wayland 环境下，按 `i` 无法进入 `human_height_mm` 输入模式。
- 根因：脚本直接用 `cv2.waitKeyEx()` 的原始返回值和 `ord('i')` 比较；该环境下字母键可能携带高位修饰码，导致匹配失败。
- 处理：
  - 新增 `normalized_ascii_key()` 和 `key_matches_char()`，统一按低 8 位字符键处理字母/数字/回车/退格。
  - 输入模式与导航模式都改为使用归一化后的字符键判断。
- 影响：
  - `i/x/a/d/w/s/z/e/g/h/q` 以及数字、`.`、`-` 输入在当前环境下更稳定。
  - 不改变已有缓存格式，也不影响 `Q5_test1` 已生成的 debug 数据。

## 2026-03-25 debug-v2 视窗自适应修复

- 现象：`Q5_test1` 逐帧调试窗口在当前显示器上看不到右侧输入框和底部帮助，用户反馈 `a/d` 没反应且找不到 `Human Height Input`。
- 根因：原始布局把 `roi_debug` 按 `520 px` 宽显示，窄图被放大后高度接近 `1023 px`，再叠加右侧文本面板，整张画布约 `1580 x 1383`，超出常见笔记本屏幕高度。
- 处理：
  - 新增 `resize_to_fit()`，在显示前把整张调试画布自动缩放到 `--window-max-width/--window-max-height` 范围内。
  - 默认 `--roi-width` 从 `520` 下调到 `320`，降低纵向拉伸。
  - 帮助区新增 `click viewer once to focus the window` 提示。
- 影响：
  - 右侧 `Human Height Input` 和底部 help 在当前屏幕上应当可见。
  - 不改变缓存内容，只改变 viewer 展示尺寸。

## 2026-03-25 debug-v2 人工输入浮层

- 现象：即使窗口经过自适应缩放，用户仍反馈“右侧输入框看不到”。
- 处理：
  - 在主图左下角新增固定可见的 `Human Height Input` 浮层，复用同一套输入状态。
  - 非编辑态也填充深色背景，避免叠在照片上时文字不可读。
- 影响：
  - 即使右侧面板被窗口管理器裁掉，用户仍能看到 `human_height_mm` 当前值和 `[editing]` 状态。
  - `i -> 输入数字 -> Enter` 的工作流不变。

## 2026-03-25 debug-v2 鼠标点击进入输入

- 现象：截图确认左下角 `Human Height Input` 浮层已经可见，但用户仍反馈按 `i` 没有明显进入编辑态。
- 处理：
  - 让 `draw_input_box()` 返回浮层矩形。
  - viewer 新增 `cv2.setMouseCallback()`，点击左下角 `Human Height Input` 浮层即可进入编辑态。
  - help 文案改为 `i or click box: edit human height label [mm]`。
- 影响：
  - 录入人工标签不再依赖单一键盘热键。
  - 后续可以直接用鼠标点输入框，再输入数字并按 `Enter` 保存。

## 2026-03-25 debug-v2 左上状态字改红

- 用户反馈：左上主图里的黄色状态字不够醒目，希望改成红色。
- 处理：
  - `build_session()` 里写入缓存照片的左上状态字颜色从黄 `(0,255,255)` 改为红 `(0,0,255)`。
  - `compose_view()` 在显示时再覆盖一层红色状态字，并先画黑底条，保证即使复用旧缓存也会立即看到红字。
- 影响：
  - 当前 `Q5_test1` 使用 `--reuse-cache` 也会直接显示红色状态字。
  - 后续重新构建的 `photo/frame_*.jpg` 也会原生保存为红字版本。

## 2026-03-25 debug-v2 默认翻帧步长改为 5

- 用户反馈：逐帧审阅时默认 `a/d` 每次只跳 `1` 帧，切换太慢。
- 处理：
  - 新增 `SMALL_STEP_FRAMES = 5`。
  - `a/d` 和空格共用的小步长导航从 `±1` 改为 `±5`。
  - 面板内提示和 help 文案同步更新为 `a/d +/-5`。
- 影响：
  - 不改变 `w/s = ±10`、`z/e = ±50` 的大步长行为。
  - 当前 `Q5_test1` viewer 重启后即可生效。

## 2026-03-26 0325 新场景数据重跑

- 新数据：
  - 目录：`/data/a/slosh_bags/0325`
  - 静止包：`slosh_QQ0_static_55But58ForReal_20260325_203619.bag`、`slosh_QQ5_static_55BUt58FoForReal_20260325_204039.bag`
  - 运动包：`slosh_QQ0_s_test1_20260325_203739.bag`、`slosh_QQ5_sttest1_20260325_204117.bag`、`slosh_QQ5_test2_20260325_204526.bag`、`slosh_QQ5_test3_20260325_204626.bag`
- 新标定：
  - 首帧导出：`/data/a/realsense_validation_v2/calibration/0325/Q0_static/frame_export/frame_000000.png`、`/data/a/realsense_validation_v2/calibration/0325/Q5_static/frame_export/frame_000000.png`
  - 采用标定：`/data/a/realsense_validation_v2/calibration/0325/scene_0325_multiscale_raw.yaml`
  - 关键检查：`rectified ruler-point x spread = 3.2164 px`
- 静止包结果：
  - `Q0_static`：`631` 帧，`valid_v2=359`，`reportable=256`，`center median=0.990 mm`，`peak median=1.309 mm`
  - `Q5_static`：`762` 帧，`valid_v2=479`，`reportable=5`，`center median=0.824 mm`，`peak median=1.309 mm`
- 运动包提取结果：
  - `Q0_test1`：`1241` 帧，`valid_v2=384`，`reportable=139`，`center_max=12.286 mm`，`peak_max=7.511 mm`
  - `Q5_test1`：`2363` 帧，`valid_v2=585`，`reportable=76`，`center_max=2.835 mm`，`peak_max=5.176 mm`
  - `Q5_test2`：`1080` 帧，`valid_v2=232`，`reportable=91`，`center_max=4.550 mm`，`peak_max=11.083 mm`
  - `Q5_test3`：`1319` 帧，`valid_v2=633`，`reportable=154`，`center_max=7.853 mm`，`peak_max=8.849 mm`
- 零对齐后 `center vs /slosh/height`：
  - `Q0_test1`：`MAE=0.200 mm`，`RMSE=0.229 mm`，`Corr=0.586`
  - `Q5_test1`：`MAE=0.221 mm`，`RMSE=0.298 mm`，`Corr=0.257`
  - `Q5_test2`：`MAE=0.043 mm`，`RMSE=0.076 mm`，`Corr=0.717`
  - `Q5_test3`：`MAE=0.219 mm`，`RMSE=0.294 mm`，`Corr=0.751`
- 零对齐后 `peak vs /slosh/height_pred_max`：
  - `Q0_test1`：`MAE=1.001 mm`，`RMSE=1.555 mm`，`Corr=0.082`
  - `Q5_test1`：`MAE=1.323 mm`，`RMSE=1.630 mm`，`Corr=0.233`
  - `Q5_test2`：`MAE=0.386 mm`，`RMSE=1.077 mm`，`Corr=0.409`
  - `Q5_test3`：`MAE=1.177 mm`，`RMSE=1.239 mm`，`Corr=0.328`
- 当前判断：
  - 新场景确实抑制了上一组里由深色背景触发的极端坏点，`peak_max` 已明显回落，不再出现上一组 `20~30 mm` 级别的离谱值。
  - `center` 仍然是稳定主口径，但这组数据和 `/slosh/height` 的一致性不像上一组 `Q5_test1` 那么强，说明这次模型侧激励或实验节奏与视觉量级更接近“中等一致”而不是“近乎重合”。
  - `peak` 仍然不适合作为主口径；即使背景改善，`peak vs pred_max` 也只是局部改善，没有变成稳定代理。

## 2026-03-26 动力学训练数据导出脚本

- 新增脚本：`scripts/export_slosh_dynamics_dataset.py`
- 目标：
  - 从 `debug_session.csv` 直接导出“动力学输入 -> 液面高度”的监督学习样本。
  - 输入采用固定长度历史时间窗，不再使用单时刻标量，避免把有记忆的晃动系统错误地建模成静态映射。
- 当前导出特征：
  - `v_mps`
  - `omega_radps`
  - `ax_cmd_mps2 = dv/dt`
  - `ay_model_mps2 = v * omega`
  - `imu_ay_mps2`
- 输出文件：
  - `slosh_dynamics_dataset.npz`
  - `slosh_dynamics_samples.csv`
  - `slosh_dynamics_metadata.json`
- 脚本参数要点：
  - 数据源：`--debug-dir` 或 `--session-csv`
  - 目标列：`--target-column`，默认 `human_height_mm`
  - 可选兜底：`--fallback-target-column`
  - 窗口：`--history-frames`、`--stride`
- 烟雾验证：
  - 因 `Q0_test1` 当前还没有人工标签，先用 `center_rel_mm_v2` 作为代理目标验证脚本链路。
  - 命令：
    - `python3 export_slosh_dynamics_dataset.py --debug-dir /data/a/realsense_validation_v2/debug/0325/Q0_test1 --target-column center_rel_mm_v2 --history-frames 30 --stride 1 --out-dir /data/a/realsense_validation_v2/datasets/0325/Q0_test1_center_proxy`
  - 结果：
    - 导出样本 `1088`
    - 张量形状：`X.shape = (1088, 30, 5)`，`y.shape = (1088,)`
    - 输出目录：`/data/a/realsense_validation_v2/datasets/0325/Q0_test1_center_proxy`
- 当前判断：
  - 这条“动力学-only”数据链已经可用。
  - 等人工标注积累后，把 `--target-column` 切回 `human_height_mm` 即可生成正式训练集。

## 2026-03-26 补充 imu_ax 到 debug/export 链

- 背景：
  - IMU 轴与容器平面前后/左右对齐，但 IMU 不在 `base_link` 原点。
  - 当前训练不一定必须使用 IMU，但为了后续多模态建模，先把原始平面加速度相关量尽量完整记下。
- 处理：
  - `debug_liquid_vs_mpc_frame_by_frame_v2.py` 新增读取 `/imu/data.linear_acceleration.x`，并写入 `debug_session.csv` 的 `imu_ax` 列。
  - viewer 面板状态字同步显示 `imu_ax` 与 `imu_ay`。
  - `export_slosh_dynamics_dataset.py` 的导出特征从 `5` 维扩展为 `6` 维：
    - `v_mps`
    - `omega_radps`
    - `ax_cmd_mps2`
    - `ay_model_mps2`
    - `imu_ax_mps2`
    - `imu_ay_mps2`
- 影响：
  - 后续训练时可自由决定是否使用 IMU 分量，而不用因为当前没有记录而返工重采。
  - 已有旧版 `debug_session.csv` 不会自动补出 `imu_ax`；需要重建对应 debug 会话后，导出脚本才能拿到该列。

## 2026-03-26 人工标注视图改为干净图

- 背景：
  - 人工输入 `human_height_mm` 时，原 viewer 左图与 ROI 图都叠了较多调试标记，容易遮挡液面边界。
  - 当前标注阶段更需要“清晰底图 + 单一零位参考”，而不是同时看所有检测叠加。
- 调整：
  - `debug_liquid_vs_mpc_frame_by_frame_v2.py` 改为缓存干净图：
    - 全图仅保留零位参考线
    - 旋正 ROI 仅保留零位水平参考线
    - 去掉 `RS visual peak`、`main center`、ROI 框、顶部状态字等图上叠加
    - 去掉参考线旁的 `0 mm` 字样，避免文字本身干扰人工观察
  - viewer 左侧大图顶部保留红色抬头：`RS visual peak` 与 `/slosh/height`
  - `Human Height Input` 输入框改回左侧大图左下角，便于一边看图一边录入
  - 右侧继续保留数值面板与历史曲线
- 目的：
  - 让人工标注时看到的液面边界尽量接近原始图像。
  - 仍保留右侧数值面板、历史曲线和标签输入，不影响当前 debug/标注流程。

## 2026-03-26 新增人眼标签分析脚本并验证 Q0_test1 偏置

- 新增脚本：
  - `scripts/analyze_human_labels_vs_realsense_v2.py`
- 目标：
  - 离线分析 `debug_session.csv` 中的人眼标签与 `center / peak / /slosh/height` 的误差关系。
  - 给出 `center` 的常数偏置修正建议，并评估修正前后指标。
- `Q0_test1` 实跑结果：
  - 输入：`/data/a/realsense_validation_v2/debug/0325/Q0_test1/debug_session.csv`
  - 标注帧数：`248`
  - 原始指标：
    - `center_rel_mm_v2 vs human_height_mm`: `MAE=0.971868`, `RMSE=0.988874`, `Corr=0.699503`
    - `peak_rel_mm_v2 vs human_height_mm`: `MAE=1.328404`, `RMSE=1.358630`, `Corr=0.676175`
    - `slosh_height_mm vs human_height_mm`: `MAE=0.113260`, `RMSE=0.300735`, `Corr=0.560418`
  - `center` 偏置估计：
    - 零标签帧中位偏置：`0.978398 mm`
    - 全标签中位偏置：`0.967979 mm`
    - 推荐常数修正：`center_corrected = max(0, center_rel_mm_v2 - 0.978398)`
  - 常数修正后的 `center`：
    - `MAE=0.090698`, `RMSE=0.163163`, `Corr=0.735856`
  - 线性拟合修正：
    - `target ~= 0.615335 * center_rel_mm_v2 - 0.556659`
    - `MAE=0.100313`, `RMSE=0.151013`, `Corr=0.723664`
- 当前判断：
  - 在这批 `Q0_test1` 人眼标签上，`center` 主要问题不是形状跟不上，而是接近常数的零位正偏置。
  - 第一优先级应是把“常数偏置修正”做成可选输出，而不是先继续调 `peak` 或大改 detector。

## 2026-03-26 接入 center 偏置修正并复核 0325 模型有效性

- 代码改动：
  - `config/liquid_measurement_v2.yaml`
    - 新增 `postprocess.center_bias_correction_mm: 0.978398`
  - `extract_liquid_height_v2_from_bag.py`
    - 新增 CLI 参数 `--center-bias-correction-mm`
    - 默认从 config 读取 `postprocess.center_bias_correction_mm`
    - CSV 新增：
      - `center_bias_correction_mm_v2`
      - `height_center_rel_mm_signed_bias_corrected_v2`
      - `height_center_rel_mm_bias_corrected_v2`
    - 主曲线与 debug 视频中的主液面 `center`，在 bias>0 时改用修正后输出
  - `compare_realsense_vs_mpc_slosh_v2.py`
    - 新增 CLI 参数 `--center-bias-correction-mm`
    - 若 CSV 已含 `height_center_rel_mm_bias_corrected_v2`，优先用它做主对比
    - 若 CSV 还是旧格式，则可按 CLI 传入的常数偏置现场计算修正后的 `center` 主序列
- 运行时验证：
  - `extract_liquid_height_v2_from_bag.py` 用 `Q0_test1` 前 `5` 帧做烟雾测试通过
  - 输出确认包含新列，且 `center_bias_correction_mm_v2 = 0.978398`
- 用 `0.978398 mm` 偏置修正复核 `0325` 四个运动 bag 的 `/slosh/height` 有效性：
  - `Q0_test1`
    - 主液面 `center_rel_mm_bias_corrected_v2 vs /slosh/height`: `MAE=0.187`, `RMSE=0.210`, `Corr=0.630`
  - `Q5_test1`
    - `MAE=0.184`, `RMSE=0.277`, `Corr=0.220`
  - `Q5_test2`
    - `MAE=0.016`, `RMSE=0.060`, `Corr=0.719`
  - `Q5_test3`
    - `MAE=0.197`, `RMSE=0.268`, `Corr=0.746`
  - 合并全部 reportable 帧：
    - `center_rel_mm_bias_corrected_v2 vs /slosh/height`: `n=460`, `MAE=0.156`, `RMSE=0.225`, `Corr=0.379`
    - `peak_rel_mm_v2 vs /slosh/height_pred_max`: `n=441`, `MAE=0.984`, `RMSE=1.389`, `Corr=-0.120`
- 当前判断：
  - 修正后的 `center` 已经明显优于原始 `center`，并且能作为当前主液面代理继续使用。
  - `/slosh/height` 在部分 bag 上有效，尤其 `Q5_test2/Q5_test3`；但跨 bag 稳定性一般，`Q5_test1` 明显偏弱。
  - `/slosh/height_pred_max` 当前仍不能作为 `peak` 的可靠对应量。

## 2026-03-26 按新主线重跑 0325 verify 输出

- 处理：
  - 重新运行 `extract_liquid_height_v2_from_bag.py` 覆盖 `0325` 下 `Q0_static / Q5_static / Q0_test1 / Q5_test1 / Q5_test2 / Q5_test3`
  - 重新运行 `compare_realsense_vs_mpc_slosh_v2.py` 覆盖 `4` 个运动 bag 的标准对比图与对齐 CSV
- 核查：
  - 新版 `liquid_height_v2.csv` 已包含：
    - `center_bias_correction_mm_v2`
    - `height_center_rel_mm_signed_bias_corrected_v2`
    - `height_center_rel_mm_bias_corrected_v2`
  - 静止/低晃动中位数明显回到零位附近：
    - `Q0_static`: 原始 `center` 中位数 `0.989112 mm`，修正后 `0.010714 mm`
    - `Q5_static`: 原始 `0.809394 mm`，修正后 `0.000000 mm`
    - `Q0_test1`: 原始 `0.992515 mm`，修正后 `0.014117 mm`
    - `Q5_test1/Q5_test2/Q5_test3`: 修正后中位数均为 `0.000000 mm`
- 说明：
  - compare 图中出现的小于 `0` 的值，来自 `initial zero align` 后的相对偏差，不是提取脚本输出了负的物理液位。
  - `extract` 主输出里的 `height_center_rel_mm_v2` 与 `height_center_rel_mm_bias_corrected_v2` 仍然经过 `positive_only(...)`，不会小于零。

## 2026-03-26 输出 0325 验证 README

- 新增文件：
  - `/data/a/realsense_validation_v2/verify/0325/README.md`
- 文档内容：
  - 汇总 `0325` 静止包与运动包的验证范围
  - 说明当前主液面口径为 `height_center_rel_mm_bias_corrected_v2`
  - 分析 `reportable` 帧上 `/slosh/height` 与 RealSense 主液面的误差、相关性、置信度分布
  - 比较 `Q0_test1` 与 `Q5_test1/2/3` 的 `/slosh/height` 强度（基于 zero-align 后的 `|height| p90/max`）
  - 解释 compare 图中出现负值的原因：来自 `initial zero align`，不是物理液位小于零
  - 明确给出当前判断：暂不建议进入“双侧都标 0-25 mm”主线

## 2026-03-26 新增路径与激励对比方案文档

- 新增/更新文件：
  - `对比路径方案.md`
- 目的：
  - 把“是否需要先比较 bag 的路径和运动激励”收敛成一份独立方案
  - 先回答 `Q0` 与 `Q5` 的 bag 是否同任务可比，再讨论“抑制是否有效”
- 文档要点：
  - 明确这条线的核心不是只比较几何路径，而是比较：
    - 实际 `odom` 路径
    - `cmd_vel`
    - `ax/ay` 激励
    - `local_path/global_path_smooth`
    - `/slosh/height`
  - 固定本轮比较对象：
    - `Q0_test1`
    - `Q5_test1`
    - `Q5_test2`
    - `Q5_test3`
  - 规划独立脚本：
    - `scripts/compare_bag_paths_and_excitation_0325.py`
  - 规划独立输出目录：
    - `/data/a/realsense_validation_v2/verify/0325/path_compare/`

## 2026-03-28 新增 slosh model 离线重放脚本

- 新增文件：
  - `scripts/replay_slosh_model_from_bag.py`
- 目的：
  - 基于 bag 中已记录的 `/slosh/ax_est`、`/slosh/ay_est`、`/slosh/omega_est_used`、`/slosh/alpha_est`、`/slosh/state`
  - 离线重放当前工程里的 slosh 动力学
  - 对比：
    - bag 原始 `/slosh/height`
    - bag 原始 `/slosh/height_pred_max`
    - RealSense 主液面 `height_center_rel_mm_bias_corrected_v2`
- 模型口径：
  - 明确按当前工程实现重放 `Lp` 版本：
    - 状态更新仍是线性 MSD 的离散模型
    - `--use-nonlinear-model` 只切换高度映射系数 `L -> NL`
    - 不会把动力学本体变成论文里的完整非线性 EOM
- 参数覆盖：
  - 支持命令行改动：
    - `liquid_height`
    - `damping_ratio`
    - `mode_index`
    - `offset_x/offset_y`
    - `L/NL height mapping`
    - `parabola term on/off`
- 输出：
  - `slosh_recomputed_compare.png`
  - `slosh_recomputed_aligned.csv`
  - `slosh_recomputed_summary.json`
- 实现修正：
  - 修复 CSV 表头在未启用 zero-align 时出现空列名的问题
  - 新增 `recomputed_vs_realsense` 指标，避免只看到 `bag_vs_realsense` 而无法判断参数改动有没有实际收益
- 本地验证：
  - `python3 -m py_compile scripts/replay_slosh_model_from_bag.py`
  - `Q5_test1` 烟雾测试：
    - 默认参数 `h=0.055, L, parabola=on`
    - `bag vs recomputed`: `MAE=0.000982 mm`, `RMSE=0.007245 mm`, `Corr=0.999073`
    - `recomputed vs RealSense`: `MAE=0.106636 mm`, `RMSE=0.179012 mm`, `Corr=0.407366`
  - `Q5_test1` 参数敏感性：
    - `h=0.055 -> 0.058` 对曲线几乎没有影响
    - 切到 `NL` 高度映射后：
      - `bag vs recomputed`: `MAE=0.016439 mm`, `RMSE=0.030573 mm`, `Corr=0.999074`
      - `recomputed vs RealSense`: `MAE=0.102455 mm`, `RMSE=0.168740 mm`, `Corr=0.407512`
- 当前结论：
  - bag 中记录的 `/slosh/*` 输入足以高精度复现当前 `/slosh/height`
  - 只把 `liquid_height` 从 `0.055` 改到 `0.058`，对当前场景的输出几乎没有可见影响
  - 单看 `Q5_test1`，`NL` 高度映射相对 `L` 只带来很小的 RealSense 拟合改善，暂时不够支撑直接改线上配置
  - 默认输出目录已从“bag 同目录”改为：
    - `/data/a/realsense_validation_v2/debug/<bag批次>/slosh_replay/<bag_stem>/`
    - 当前 `0325` 数据如果不显式传 `--out-dir`，会自动落到 `/data/a/realsense_validation_v2/debug/0325/slosh_replay/<bag_stem>/`

## 2026-03-28 修复 paper_nl 重放输出并完成首轮对比

- 修复文件：
  - `scripts/replay_slosh_model_from_bag.py`
- 修复内容：
  - 修复 `paper_nl_total_height` 中抛物面项被重复乘以 `1000` 的量纲错误
  - 把 `paper_nl_modal_height`、`paper_nl_total_height` 正式写入 `slosh_recomputed_aligned.csv`
  - 新增 `bag_vs_paper_nl`、`paper_nl_vs_realsense` 指标，并在 `--replay-mode paper_nl/both` 时打印到终端和写入 `summary.json`
- 本地验证：
  - `python3 -m py_compile scripts/replay_slosh_model_from_bag.py`
  - `linear_engineering` 验证目录：
    - `/data/a/realsense_validation_v2/debug/0325/slosh_replay/replay_review_linear_fix`
  - `paper_nl` 验证目录：
    - `/data/a/realsense_validation_v2/debug/0325/slosh_replay/replay_review_paper_nl_fix`
- `Q5_test1` 修复后结果：
  - `bag_vs_recomputed`:
    - `MAE=0.000982 mm`
    - `RMSE=0.007245 mm`
    - `Corr=0.999073`
  - `bag_vs_paper_nl`:
    - `MAE=0.017071 mm`
    - `RMSE=0.032493 mm`
    - `Corr=0.997182`
  - `recomputed_vs_realsense`:
    - `MAE=0.106636 mm`
    - `RMSE=0.179012 mm`
    - `Corr=0.407366`
  - `paper_nl_vs_realsense`:
    - `MAE=0.103687 mm`
    - `RMSE=0.170847 mm`
    - `Corr=0.392996`
- 当前判断：
  - 在 `Q5_test1` 这包上，论文 `paper_nl` 与 bag 当前工程 `/slosh/height` 的变化趋势和幅值都相当接近
  - `paper_nl` 相对当前工程 `Lp` 只带来了很小的误差变化，不足以改变现阶段主结论
  - 可以继续把主精力放回 `RealSense` 与 `/slosh/height` 的分析，而不是先重构在线 slosh model
  - `paper_nl_replay_plan.md` 的核心内容已经并入 `README.md` 与本日志，原计划文件可以删除

## 2026-03-28 新增 0325 分 bag RealSense center vs /slosh/height 分析

- 新增文件：
  - `scripts/analyze_realsense_center_vs_slosh_0325.py`
- 目标：
  - 按 bag 分析 `height_center_rel_mm_bias_corrected_v2` 对 `/slosh/height` 的误差、偏置和坏点
  - 同时保留：
    - raw 误差（不做 zero-align）
    - zero-align 误差（如果该 bag 已有 `mpc_realsense_aligned_v2.csv`）
- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325/center_vs_slosh_analysis/`
- 输出文件：
  - `center_vs_slosh_summary.csv`
  - `center_vs_slosh_outliers.csv`
  - `center_vs_slosh_summary.json`
  - `README.md`
  - `<bag>_center_vs_slosh.png`
- 本地验证：
  - `python3 -m py_compile scripts/analyze_realsense_center_vs_slosh_0325.py`
  - 正式输出已生成到上述目录
- `0325` 当前结果：
  - `Q0_static`:
    - `raw_bias_median=0.011131 mm`
    - `raw_MAE=0.016701 mm`
  - `Q5_static`:
    - `raw_bias_median=0.000000 mm`
    - `raw_MAE=0.000000 mm`
    - 但 `reportable=5/762`，静止包通过率过低，不能把它当成强结论
  - `Q0_test1`:
    - `raw_bias_median=0.007978 mm`
    - `raw_MAE=0.089600 mm`
    - `zero_MAE=0.187325 mm`
  - `Q5_test1`:
    - `raw_bias_median=0.000000 mm`
    - `raw_MAE=0.182175 mm`
    - `zero_MAE=0.184293 mm`
  - `Q5_test2`:
    - `raw_bias_median≈0`
    - `raw_MAE=0.016438 mm`
    - `zero_MAE=0.016438 mm`
    - 当前是最干净的一包
  - `Q5_test3`:
    - `raw_bias_median≈0`
    - `raw_MAE=0.078259 mm`
    - `zero_MAE=0.196902 mm`
- 坏点判断：
  - 多数 top outlier 没有表现出明显的 `low_confidence / high_fit_rms`
  - 当前更像：
    - 模型与视觉主液面的残余不一致
    - 或者小幅偏置/时序差异
  - 而不是单纯的视觉检测崩坏
- 当前结论：
  - `Q5_test2` 可以作为后续主样本
  - `Q5_test1` 是当前最值得继续做坏点深挖的一包
  - `Q0_static/Q0_test1` 仍有轻微正偏置，但量级已经很小

## 2026-03-29 新增 Q5_test1 / Q5_test2 相位差与坏点分段分析

- 新增文件：
  - `scripts/analyze_q5_phase_and_outliers_0325.py`
- 目标：
  - 用 `Q5_test2` 作为主参考包
  - 单独深挖 `Q5_test1` 的相位差和大误差分段
  - 避免只看全局误差均值，改成：
    - `lag sweep`
    - 近零最优 lag
    - 大误差分段
- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325/q5_phase_outlier_analysis/`
- 输出文件：
  - `phase_summary.csv`
  - `phase_summary.json`
  - `Q5_test1_outlier_segments.csv`
  - `Q5_test1_segments/*.png`
  - `Q5_test1_lag_sweep.png`
  - `Q5_test1_lag_overlay.png`
  - `Q5_test2_lag_sweep.png`
  - `Q5_test2_lag_overlay.png`
  - `README.md`
- 本地验证：
  - `python3 -m py_compile scripts/analyze_q5_phase_and_outliers_0325.py`
  - `python3 scripts/analyze_q5_phase_and_outliers_0325.py --help`
  - 正式输出已生成到上述目录
- 当前结果：
  - `Q5_test1`
    - `lag=0`: `MAE=0.197789 mm`, `RMSE=0.292501 mm`, `Corr=0.175171`
    - 近零最优 lag：`-240 ms`
    - 近零最优 lag 下：`Corr=0.366997`, `RMSE=0.252165 mm`
    - 全局最优相关 lag：`1010 ms`
    - 但这个量级接近重复振荡周期，不能直接当作物理时延
  - `Q5_test2`
    - `lag=0`: `MAE=0.016438 mm`, `RMSE=0.059554 mm`, `Corr=0.718568`
    - 近零最优 lag：`-250 ms`
    - 近零最优 lag 下：`Corr=0.836447`, `RMSE=0.046396 mm`
    - 全局最优相关 lag：`-760 ms`
    - 同样更像周期性重复，而不是直接物理时延
  - `Q5_test1` 当前坏点阈值：`abs(error) >= 0.624032 mm`
  - 导出的高误差分段共 `3` 段：
    - `frames 893-894`
    - `frame 970`
    - `frame 1047`
- 现阶段判断：
  - `Q5_test1` 和 `Q5_test2` 的近零最优 lag 都在约 `-250 ms`，说明两包都存在类似量级的时序偏移
  - 但 `Q5_test1` 即使允许近零 lag 修正，相关性和 RMSE 仍明显差于 `Q5_test2`
  - 所以 `Q5_test1` 的问题不只是相位差，还包含更明显的幅值/模型不一致或局部异常时段
  - 下一步应优先看：
    - `Q5_test1` 三个高误差分段对应的实物画面
    - 这些分段里 `RealSense center` 和 `/slosh/height` 的幅值谁在偏大/偏小

## 2026-03-29 新增 Center 位置可视复核图导出

- 新增文件：
  - `scripts/render_center_overlay_review_0325.py`
- 目标：
  - 针对 `Q5_test1` 高误差分段，导出干净的 ROI 复核图
  - 在同一张图上直接叠加：
    - `0 mm`
    - `RealSense center`
    - `/slosh/height` 映射后的高度线
  - 用来判断：
    - `RealSense center` 是否在视觉上偏高
    - `/slosh/height` 是否更像偏低
- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325/q5_phase_outlier_analysis/center_overlay_review/`
- 输出文件：
  - `review_summary.csv`
  - `README.md`
  - `segment_*/frame_*_review.png`
- 本地验证：
  - `python3 -m py_compile scripts/render_center_overlay_review_0325.py`
  - `python3 scripts/render_center_overlay_review_0325.py --help`
  - 正式输出已生成
- 当前状态：
  - 已按 `Q5_test1_outlier_segments.csv` 导出 `16` 张复核图
  - 可直接逐帧判断红线（`RealSense center`）和蓝线（`/slosh/height`）谁更贴近真实液面
  - 脚本现已支持 `--visual-zero-offset-mm`
    - 正值会把 `0 mm / RealSense center / /slosh/height` 三条线一起在 ROI 中上移
    - 用于快速验证“零位整体低了约 0.2 mm”这类系统偏差假设
  - 已正式导出 `0.2 mm` 偏移版本：
    - `/data/a/realsense_validation_v2/verify/0325/q5_phase_outlier_analysis/center_overlay_review_offset_0p2mm/`

## 2026-03-29 annotate_height_ruler_v2.py 支持前后两条静止液面线取中线

- 修改文件：
  - `scripts/annotate_height_ruler_v2.py`
- 新增参数：
  - `--still-level-mode`
    - `single_line`：旧流程，点击一条可见静止液面线
    - `front_back_midpoint`：新流程，分别点击前后两条可见静止液面线，脚本自动取中线作为 `0 mm`
- 实现方式：
  - 交互上新增：
    - `still_front_left / still_front_right`
    - `still_back_left / still_back_right`
  - 保存时：
    - `geometry_roi.still_level_line` 仍写入中线，保持提取脚本兼容
    - 额外记录：
      - `geometry_roi.still_level_front_line`
      - `geometry_roi.still_level_back_line`
      - `height_mapping.height_zero_annotation_mode`
- 兼容性：
  - 旧单线模式不变
  - `extract_liquid_height_v2_from_bag.py` 无需修改，继续读取 `still_level_line`
- 本地验证：
  - `python3 -m py_compile scripts/annotate_height_ruler_v2.py`
  - `python3 scripts/annotate_height_ruler_v2.py --help`
- 当前建议：
  - 对你现在能同时看到试管前后两条液面线的静止图，优先使用：
    - `--still-level-mode front_back_midpoint`

## 2026-03-29 0325 新零位重标后，按新 bias 重新提取并汇总 Q5

- 新标定文件：
  - `/data/a/realsense_validation_v2/calibration/0325/scene_0325_multiscale_raw_rezero.yaml`
- 备份策略：
  - 不覆盖旧目录
  - 旧结果保留在：
    - `/data/a/realsense_validation_v2/verify/0325_rezero/`
  - 新 bias 重跑结果写到：
    - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/`
- 新 bias 选取：
  - 基于新 `Q5_static` 的 `height_center_rel_mm_v2` reportable 中位数
  - 取值：`1.233421 mm`
- 重跑对象：
  - `Q5_static`
  - `Q5_test1`
  - `Q5_test2`
- 汇总分析目录：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/center_vs_slosh_analysis/`
- 当前结果：
  - `Q5_static`
    - `raw_bias_median=0.000000 mm`
    - `raw_MAE=0.007837 mm`
    - `reportable=461/762`
  - `Q5_test1`
    - `raw_bias_median=0.041769 mm`
    - `raw_MAE=0.093253 mm`
    - `reportable=435/2363`
  - `Q5_test2`
    - `raw_bias_median=0.046210 mm`
    - `raw_MAE=0.092495 mm`
    - `reportable=24/1080`
- 当前判断：
  - `front_back_midpoint` 的静止液面零位定义是有效的
  - 旧 `0.978398 mm` bias 在新零位下已失效
  - 换成 `1.233421 mm` 后，`Q5_static` 已基本贴零
  - `Q5_test1 / Q5_test2` 也回到 `~0.09 mm` 量级，明显好于仅重标零位、不改 bias 的版本

## 2026-03-29 基于 0325_rezero_bias 重跑 Q5 相位差与坏点分析

- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/q5_phase_outlier_analysis/`
- 关键文件：
  - `phase_summary.csv`
  - `Q5_test1_outlier_segments.csv`
  - `center_overlay_review/`
- 当前结果：
  - `Q5_test1`
    - `lag=0`: `MAE=0.141411 mm`, `RMSE=0.213660 mm`, `Corr=0.452617`
    - 近零最优 lag：`-60 ms`
    - 近零最优 lag 下：`Corr=0.549369`, `RMSE=0.200139 mm`
    - `reportable_frames=192`
  - `Q5_test2`
    - `lag=0`: `MAE=0.092495 mm`, `RMSE=0.121033 mm`, `Corr=0.688187`
    - 近零最优 lag：`-190 ms`
    - 近零最优 lag 下：`Corr=0.721093`, `RMSE=0.107795 mm`
    - `reportable_frames=24`
- `Q5_test1` 当前高误差分段：
  - `frames 888-925`
  - `frame 954`
  - `frame 650`
  - `frame 1004`
  - `frame 75`
  - `frames 777-795`
- 可视复核：
  - 已基于新分段导出：
    - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/q5_phase_outlier_analysis/center_overlay_review/`
  - 共 `36` 张复核图
- 当前判断：
  - 重新标定零位并更新 bias 后，`Q5_test1` 相比旧版本已经明显改善
  - 但 `Q5_test1` 仍然弱于 `Q5_test2`
  - 剩余差异不再像“整体零位问题”，而更像：
    - 少量局部异常时段
    - 外加小幅近零 lag
  - 人工逐段复核结论（基于 `center_overlay_review/`）：
    - `segment_01 (frames 888-925)`: `/slosh/height` 偏低更明显
    - `segment_02 (frame 954)`: `/slosh/height` 偏低更明显
    - `segment_03 (frame 650)`: `/slosh/height` 偏低更明显
    - `segment_04 (frame 1004)`: `RealSense center` 与 `/slosh/height` 两边都差不多
    - `segment_05 (frame 75)`: `RealSense center` 偏高更明显
    - `segment_06 (frames 777-795)`: `RealSense center` 与 `/slosh/height` 两边都差不多
  - 汇总：
    - `6` 段里有 `3` 段更像 `/slosh/height` 偏低
    - `1` 段更像 `RealSense center` 偏高
    - `2` 段两边都存在偏差
  - 这说明：
    - `Q5_test1` 的残余误差主因更偏向模型侧局部低估，而不是视觉 `center` 系统性偏高
    - 但视觉侧也仍然存在少量局部异常帧，不能把所有误差都归到模型

## 2026-03-29 Q5_test1 分段输入与状态分析

- 新增脚本：
  - `scripts/analyze_q5_test1_segment_inputs_0325.py`
- 目的：
  - 针对 `0325_rezero_bias/Q5_test1` 的坏点段，逐段查看：
    - `RealSense center` vs `/slosh/height`
    - `/slosh/ax_est`、`/slosh/ay_est`
    - `/slosh/omega_est_used`、`/slosh/alpha_est`
    - `/slosh/state = [eta_x, eta_x_dot, eta_y, eta_y_dot]`
- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/q5_segment_input_analysis/`
- 关键文件：
  - `segment_input_state_summary.csv`
  - `segment_input_state_summary.json`
  - `segments/*.png`
  - `README.md`
- 验证：
  - `python3 -m py_compile scripts/analyze_q5_test1_segment_inputs_0325.py`
  - `python3 scripts/analyze_q5_test1_segment_inputs_0325.py --help`
  - `/tmp/q5_segment_input_analysis_smoke/` 烟雾测试通过
- 当前结果摘要：
  - `segment_01 (888-925)`
    - `max|error|=1.026055 mm`
    - `max|ay|=0.500183 m/s²`
    - `max|omega|=0.996788 rad/s`
    - `max|eta_modal|=0.000413 m`
  - `segment_02 (954)`
    - `max|error|=1.008308 mm`
    - `max|ay|=0.574966 m/s²`
    - `max|omega|=1.026435 rad/s`
    - `max|eta_modal|=0.000471 m`
  - `segment_03 (650)`
    - `max|error|=0.621469 mm`
    - `max|ax|=0.336933 m/s²`
    - `max|ay|=0.015271 m/s²`
    - `max|omega|=0.121857 rad/s`
    - `max|eta_modal|=0.000330 m`
  - `segment_04 (1004)`
    - `mean_error=-0.237278 mm`
    - 该段不是模型低估，而更像两边都在附近、甚至 `/slosh/height` 略高
  - `segment_05 (75)`
    - `max|error|=0.467041 mm`
    - `ax/ay/omega/alpha/state` 都接近 `0`
    - 该段更像视觉 `center` 局部偏高或残余零位/检测问题，不像模型动态低估
  - `segment_06 (777-795)`
    - `max|error|=0.443032 mm`
    - `max|omega|=1.128924 rad/s`
    - 但 `max|ay|=0.069333 m/s²`、`max|eta_modal|=0.000188 m`
- 当前判断：
  - `Q5_test1` 的高误差段不是单一原因
  - `segment_01/02` 确实更像模型在较强横向激励和较大模态位移下局部偏低
  - `segment_03` 更像前后向主导激励下的局部低估
  - `segment_05` 说明视觉侧仍有少量局部异常，不能把所有残余误差归到 `/slosh/height`
  - 总体上：
    - `/slosh/height` 作为“当前时刻主液面高度”估计是可用的
    - 但 `Q5_test1` 在局部动态段仍存在模型侧低估，且少量静态/近静态帧存在视觉侧异常

## 2026-03-29 补跑 0325_rezero_bias 缺失 bag 并刷新总对比图

- 目的：
  - 把 `0325_rezero_bias` 从原来的半套结果补齐到全套 `6` 个 bag
  - 统一在同一套：
    - `scene_0325_multiscale_raw_rezero.yaml`
    - `center_bias_correction_mm = 1.233421`
    下查看 `center_vs_slosh`
- 新补跑 bag：
  - `Q0_static`
  - `Q0_test1`
  - `Q5_test3`
- 输出目录：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/Q0_static/`
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/Q0_test1/`
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/Q5_test3/`
- 刷新后的总分析目录：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/center_vs_slosh_analysis/`
- 当前摘要：
  - `Q0_static`
    - `raw_bias_median=0.125361 mm`
    - `raw_MAE=0.134216 mm`
    - `reportable=9/631`
  - `Q5_static`
    - `raw_bias_median=0.000000 mm`
    - `raw_MAE=0.007837 mm`
    - `reportable=461/762`
  - `Q0_test1`
    - `raw_bias_median=0.142784 mm`
    - `raw_MAE=0.189001 mm`
    - `reportable=181/1241`
  - `Q5_test1`
    - `raw_bias_median=0.041769 mm`
    - `raw_MAE=0.093253 mm`
    - `reportable=435/2363`
  - `Q5_test2`
    - `raw_bias_median=0.046210 mm`
    - `raw_MAE=0.092495 mm`
    - `reportable=24/1080`
  - `Q5_test3`
    - `raw_bias_median=0.040547 mm`
    - `raw_MAE=0.063274 mm`
    - `reportable=297/1319`
- 当前判断：
  - `0325_rezero_bias` 现在已经是完整主基线
  - `Q5` 三个运动包整体上优于 `Q0_test1`
  - 其中：
    - `Q5_test3` 当前最好
    - `Q5_test1 / Q5_test2` 居中
    - `Q0_test1` 明显更差
  - `Q0_static` 的通过帧很少，不能把它和 `Q5_static` 直接并列为强静态结论

## 2026-03-29 同步结果目录 README 结论

- 已将当前主结论同步到：
  - `/data/a/realsense_validation_v2/verify/0325_rezero_bias/center_vs_slosh_analysis/README.md`
- 同步目的：
  - 让结果目录本身就能直接回答“当前效果如何、当前判断是什么”
  - 避免主文档和结果目录 README 结论不一致
- 当前结果目录 README 已明确写入：
  - `/slosh/height` 已经可以作为“当前时刻主液面高度”的工程估计量使用
  - 但它还不能当绝对真值标准
  - `Q5_test3` 当前最好，`Q5_test1/Q5_test2` 居中，`Q0_test1` 明显更差
  - 当前不能仅凭这批 `Q0/Q5` bag 的幅值差，直接下“Q=5 必然优于 Q=0”的强结论
