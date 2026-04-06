# 监督学习方案 Log

## 当前结构框架

这段固定放在文件开头，用于让新对话先快速对齐本方案当前的主结构，再继续看后面的迭代日志。

### 1. 任务层

- 主任务 `T1`：
  - 目标是当前时刻最大液面抬升 `MSH`
  - 主标签：
    - `human_peak_y_rect_px`
    - `human_peak_mm`
- 辅助任务 `T_aux`：
  - 目标是主液面中心诊断通道
  - 辅助标签：
    - `human_center_y_rect_px`
    - `human_height_mm`

当前主张：

- `peak / MSH` 是主训练口径、主验证口径、主结论口径
- `center` 只保留为辅助诊断量，不再作为主证据链任务

### 2. 标签层

- 第一性视觉标签：
  - `human_peak_y_rect_px`
  - `human_center_y_rect_px`
- 派生物理标签：
  - `human_peak_mm`
  - `human_height_mm`
- 原则：
  - 模型优先预测 `y_rect`
  - `mm` 由固定的 still-level 与固定的 `F(y)` 派生
  - 同一批实验中，`y_rect -> mm` 映射必须固定，不能不同模型各用一套后处理

### 3. Baseline 层

- `B0`：旧峰值规则链
  - `peak_rel_mm_v2`
- `B1`：物理 proxy baseline
  - `/slosh/height`
- `B2`：dynamics-only baseline
  - `MLP`
  - `TCN`
- `B0_aux`：辅助诊断 baseline
  - `bias-corrected center_rel_mm_v2`

结论顺序固定为：

1. 旧峰值规则链
2. `/slosh/height`
3. dynamics-only
4. pure vision peak estimator
5. fusion helper model

### 4. 模型层

#### 4.1 Track P: Pure Vision Evidence Chain

目的：

- 建立不依赖 `/slosh/height` 输入的独立视觉证据链

##### P1：单帧 peak 视觉模型

- 输入：
  - 单帧旋正 ROI 图像
  - 建议先用灰度图
- 主输出：
  - `pred_peak_y_rect_px`
- 派生输出：
  - `pred_human_peak_mm`
- 可选辅助输出：
  - `pred_center_y_rect_px`
  - `pred_human_height_mm`
  - `pred_confidence`
  - `pred_valid`

推荐最小结构：

- `Conv(3x3, 16) -> ReLU -> MaxPool`
- `Conv(3x3, 32) -> ReLU -> MaxPool`
- `Flatten -> MLP(64) -> ReLU -> Linear(1)`

这版先回答：

- 单帧 ROI 里是否已经包含足够的峰值 / MSH 信息

##### P2：短时序 peak 视觉模型

- 输入：
  - 最近 `K` 帧旋正 ROI 图像序列
- 主输出：
  - `pred_peak_y_rect_px`
- 派生输出：
  - `pred_human_peak_mm`
- 可选辅助输出：
  - `pred_center_y_rect_px`
  - `pred_human_height_mm`
  - `pred_confidence`

推荐结构：

- 每帧共享轻量 CNN encoder
- 时序头优先级：
  1. `temporal average + MLP`
  2. `TCN`
  3. `small GRU`
- 输出头：
  - `Linear -> pred_peak_y_rect_px`
  - 可选 `Linear -> pred_center_y_rect_px`

当前主线判断：

- `P2` 是最值得投入的 pure vision 主模型
- 但实现顺序仍然是先 `P1` 再 `P2`

#### 4.2 Track F: Fusion Helper Model

目的：

- 在 pure vision 站住后追求更高工程精度
- 但它不再是独立证据链

可融合输入：

- ROI 图像或 ROI 短时序
- dynamics-only 特征序列
- `peak_rel_mm_v2`
- 可选 `center_rel_mm_v2`
- `/slosh/height`

推荐结构：

- 图像分支复用 `P1/P2`
- 动力学分支用 `MLP` 或 `TCN`
- `Concat(z_img, z_dyn, optional rule/proxy features)`
- `MLP(64~128) -> ReLU -> Linear head(s)`

硬边界：

- 如果模型把 `/slosh/height` 或旧规则输出喂进去，它只能叫融合辅助器
- 它不能再被用于反过来证明 `/slosh/height` 自己是对的

### 5. Dynamics-Only 结构

#### B2-MLP

- 输入：
  - `K x D` 动力学窗口展平
- 当前最小特征：
  - `v_mps`
  - `omega_radps`
  - `ax_cmd_mps2`
  - `ay_model_mps2`
  - `imu_ax_mps2`
  - `imu_ay_mps2`
- 结构：
  - `MLP(64~128) -> ReLU -> MLP(32~64) -> ReLU -> Linear`
- 默认输出：
  - `pred_human_peak_mm`

#### B2-TCN

- 输入：
  - `K x D` 动力学窗口序列
- 结构：
  - `2~3` 层 `1D temporal conv`
  - `ReLU -> temporal pooling -> MLP -> Linear`
- 默认输出：
  - `pred_human_peak_mm`

### 6. 损失层

- 主损失：
  - `L_peak = Huber(pred_peak_y_rect_px, human_peak_y_rect_px)`
- 辅助损失：
  - `L_center = Huber(pred_center_y_rect_px, human_center_y_rect_px)`
- 弱物理约束：
  - `L_mm = Huber(pred_peak_mm, human_peak_mm)`

总损失建议：

- `T1-only`
  - `L = L_peak`
- `T1 + T_aux`
  - `L = lambda1 * L_peak + lambda2 * L_center + lambda3 * L_mm`

当前推荐权重：

- `lambda1 = 1.0`
- `lambda2 = 0.2 ~ 0.4`
- `lambda3 = 0.1`

原则：

- `peak` 是主任务，权重大
- `center` 是辅助诊断任务
- `mm` 只作弱约束，不反过来主导训练
- `peak loss` 只在存在 `human_peak` 标签的样本上启用
- `center loss` 只在存在 `human_center` 标签的样本上启用

### 7. 置信度与拒识层

- `pred_confidence` 是可选输出，不是当前 `v1` 必需项
- 当前优先级：
  1. 先把 `pred_peak_y_rect_px` 做稳
  2. 再决定是否加 `pred_confidence`
  3. 最后再把 `abstain` 接入完整评估链

若后续启用 `pred_confidence`：

- `confidence >= tau_high`：
  - 正常输出
- `tau_low <= confidence < tau_high`：
  - 低置信输出
- `confidence < tau_low`：
  - `abstain`
  - 回退旧规则链或人工复核

### 8. 验收层

- 主指标看 `T1 = human_peak_mm`
- 辅助指标看 `T_aux = human_height_mm`

`0.1 mm` 的口径固定为：

- 当前目标门槛
- 不是现阶段已证明能力

建议分两级验收：

- 一级：
  - `static / near-zero peak jitter <= 0.05 ~ 0.08 mm`
- 二级：
  - held-out bag 上 `peak / MSH MAE <= 0.10 ~ 0.15 mm`

只有当 held-out bag 上的 `T1` 指标稳定过线后，才可以宣称具备 `0.1 mm` 级别的 MSH 识别能力。

## 日志

后续修改、实验结论、口径修正和待办，统一从这里往下追加。

## 2026-03-31 当前状态总结

### 1. 方案状态

- 主方案已经冻结为 `MSH-first`：
  - 主任务：`human_peak_*`
  - 辅助任务：`human_center_*`
- 当前主结论边界已经锁死：
  - `pure vision` 才能作为独立证据链
  - `fusion` 只能作为增强器
  - `0.1 mm` 当前只能被视为目标门槛，不是已证明能力

### 2. 已完成代码

当前 `SL_` 监督学习最小闭环脚本已经落地：

- `SL_build_supervised_manifest.py`
- `SL_make_splits.py`
- `SL_supervised_common.py`
- `SL_train_baseline.py`
- `SL_eval_baseline.py`
- `SL_infer_on_debug_session.py`

当前实现状态：

- `B2-MLP` 已打通 `train / eval / infer`
- `B2-TCN` 已打通 `train / eval / infer`
- 当前这些结果只证明基础设施可用，不代表 `peak/MSH` 主任务已经可用

### 3. 当前数据状态

旧 debug 根目录：

- `/data/a/realsense_validation_v2/debug`

当前统计结论：

- `peak` 标签只有 `9` 条
- 且全部集中在 `0330/Q0_test2`
- 因此当前 **没有可用的主任务 `peak/MSH train-val-test split`**

辅助任务 `center` 的情况：

- `center` 标签共有 `744` 条
- 可按 `bag` 切出一版 `train / val / test`
- 但这只能支持辅助诊断，不等于主任务已打通

### 4. 0330 重标注根目录

为了避免污染旧标签，已经新建：

- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q0_static`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q0_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q0_test2`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q5_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q5_test2`

这 5 个 session 当前状态：

- `debug_session.csv` 已生成
- `debug_session.json` 已生成
- `human_labels.csv` 已清空并重置为只有表头
- 旧目录 `/data/a/realsense_validation_v2/debug/0330` 未被覆盖

### 5. 当前标注器状态

当前使用脚本：

- `scripts/debug_liquid_vs_mpc_frame_by_frame_v2.py`

已新增 viewer 内零线手调功能：

- `r / f`：零线 `-1 / +1 px`
- `t / v`：零线 `-5 / +5 px`
- `b`：零线偏移重置为 `0 px`

说明：

- 这个偏移只影响 viewer 里的参考线显示
- 不会改动当前 `v2` 原始检测结果
- 偏移会记录到 `debug_session.json` 的 `viewer_zero_offset_px`

当前限制仍然存在：

- 标注器保存的是：
  - `human_height_mm`
  - `human_peak_mm`
- 还**不会**正式保存：
  - `human_peak_y_rect_px`
  - `human_center_y_rect_px`
  - `label_quality`
  - `label_valid`
  - `label_skip_reason`

因此：

- 当前人工真值仍然是 `mm` 标签
- 后续如果要严格切到“第一性视觉标签”，需要单独改标注器

### 6. 当前操作边界

已经明确的协作边界：

- 人工标签应由用户自己输入
- 代理不得替用户主观填写 `human_peak_mm`
- 我负责：
  - 搭建脚手架
  - 整理清单
  - 启动标注器
  - 汇总统计
  - 检查标注分布
  - 做训练/评估脚本

### 7. 当前建议

当前最稳的顺序仍然是：

1. 保持 `0330` 新重标根目录不变
2. 由用户自己继续补 `human_peak_mm`
3. 优先把 `peak/MSH` 主任务标签补到可用 split
4. 之后再回到主线监督学习实验

---

## 2026-04-06：0401 第一轮人工真值状态与下一步决策

### 1. 已完成的 0401 标注

当前 `0401` 已完成 5 个 session 的人工 `human_peak_mm` 标注：

- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q10_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q10_test2`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q0_test3`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q5_test3`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q5_static`

当前标签数量：

- `Q10_test1`: `195`
- `Q10_test2`: `139`
- `Q0_test3`: `162`
- `Q5_test3`: `220`
- `Q5_static`: `78`

合计：

- `794` 条人工标签

### 2. 与当前 FSL split 的关系

当前 `0401` 的 FSL split 为：

- `train`: `Q0_test1 + Q0_test2 + Q5_test1 + Q5_test2`
- `val`: `Q0_test3`
- `test`: `Q10_test1 + Q10_test2 + Q5_test3`

因此，当前已完成人工真值的 4 个运动 bag 全都位于 holdout 侧：

- `Q0_test3`：当前 `val`
- `Q10_test1`：当前 `test`
- `Q10_test2`：当前 `test`
- `Q5_test3`：当前 `test`

`Q5_static` 不在主 `train/val/test` 中，只作为 near-zero sanity check。

### 3. 当前最重要的结论

当前 `0401` 已经有了一批**严格 holdout 真值**，但**还没有任何人工真值训练集**。

也就是说：

- 现在适合先做：
  - `human_peak_mm vs /slosh/height`
  - `human_peak_mm vs FSL visual pseudolabel model`
  - bag-wise 曲线、误差、峰值幅值检查
- 现在**不适合**直接开始真正的 `SL` 监督学习训练
  - 因为缺少人工真值 `train` bags
  - 若强行把当前这批 holdout 真值拿去训练，会污染最终验证口径

### 4. 当前推荐顺序

当前最稳的下一步顺序应改为：

1. 先跑 `0401` 当前这批 `val/test` 真值分析
2. 锁定：
   - `/slosh/height` 对人工峰值真值的误差
   - `FSL visual` 对人工峰值真值的误差
   - 哪些 bag / 哪些时间段偏差最大
3. 在不动当前 holdout 真值的前提下，再去补人工真值训练集：
   - `Q0_test1`
   - `Q0_test2`
   - `Q5_test1`
   - `Q5_test2`
4. 之后才进入真正的 `SL` 监督学习训练

### 5. 真正进入 SL 训练前的口径

如果后续要做真正的 `SL` 监督学习训练，当前建议保持：

- `train-human`: `Q0_test1 + Q0_test2 + Q5_test1 + Q5_test2`
- `val-human`: `Q0_test3`
- `test-human`: `Q10_test1 + Q10_test2 + Q5_test3`
- `static-check`: `Q5_static`

原则：

- 当前已经标过的 `val/test` 真值不回流进训练集
- 先把当前 holdout 真值分析做完，再决定是否值得继续补 train bags

### 6. 当前决策

当前问题“想做真正的 SL 监督学习训练，现在应该怎么做”的回答是：

- **是，应该先做这批 `val/test` 真值分析。**
- 原因不是拖延训练，而是因为：
  - 这批真值已经天然是严格 holdout
  - 现在先分析，能避免后面把错误方向训练得更深
  - 也能判断是否值得继续补 `train-human` bags

## 2026-04-06：0401 holdout 真值分析结果

### 1. 输入与产物

本轮真值分析基于：

- 人工峰值 manifest：
  - `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/fsl_artifacts/FSL_human_peak_manifest_0401_holdout/SL_supervised_manifest.csv`
- 当前视觉伪标签 checkpoint：
  - `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/fsl_runs/FSL_visual_pseudolabel_0401/FSL_visual_pseudolabel.pt`
- 评估脚本：
  - `scripts/FSL_eval_visual_vs_human.py`

输出目录：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/fsl_runs/FSL_visual_vs_human_0401_holdout`

主要输出文件：

- `FSL_visual_vs_human_summary.json`
- `FSL_visual_vs_human_predictions.csv`
- `FSL_visual_vs_human_bagwise.csv`
- `FSL_visual_vs_human_tracking_only_summary.json`
- `FSL_visual_vs_human_tracking_only_predictions.csv`
- `FSL_visual_vs_human_tracking_only_bagwise.csv`

### 2. 全标签混合统计

这里把 `Q5_static` 和 motion bags 的全部已标帧混在一起统计。

总行数：

- `794`

相对 `human_peak_mm` 的总体结果：

- `/slosh/height`：
  - `MAE = 0.2581 mm`
  - `RMSE = 0.4732 mm`
  - `Corr = 0.5327`
- `pred_visual_fsl`：
  - `MAE = 0.2954 mm`
  - `RMSE = 0.4732 mm`
  - `Corr = 0.5217`
- `center_rel_mm_v2`：
  - `MAE = 0.6156 mm`
  - `coverage = 53.65%`
- `peak_rel_mm_v2`：
  - `MAE = 1.0792 mm`
  - `coverage = 53.65%`

这一版只能做粗略参考，因为大量起止静止段和 `Q5_static` 会把结果冲淡。

### 3. tracking-only 主统计

为了更接近真正的 `MSH` 验证，本轮按已确认的 `TRACKING` 帧范围重算了 motion bags：

- `Q10_test1`: `frame 387 -> 1189`
- `Q10_test2`: `frame 305 -> 824`
- `Q0_test3`: `frame 267 -> 851`
- `Q5_test3`: `frame 250 -> 1031`

`Q5_static` 单独作为 static sanity check，不混入主统计。

tracking-only 样本数：

- `556`

相对 `human_peak_mm` 的 tracking-only 结果：

- `/slosh/height`：
  - `MAE = 0.3614 mm`
  - `RMSE = 0.5590 mm`
  - `Corr = 0.4039`
  - `coverage = 100%`
- `pred_visual_fsl`：
  - `MAE = 0.3677 mm`
  - `RMSE = 0.5539 mm`
  - `Corr = 0.3832`
  - `coverage = 100%`
- `center_rel_mm_v2`：
  - `MAE = 0.6636 mm`
  - `Corr = 0.3582`
  - `coverage = 55.22%`
- `peak_rel_mm_v2`：
  - `MAE = 1.1575 mm`
  - `Corr = 0.1071`
  - `coverage = 55.22%`

当前更可信的结论应以这一版 `tracking-only` 为准，而不是全时段混算。

### 4. static-only sanity check

`Q5_static` 单独结果：

- `/slosh/height`：
  - `MAE ≈ 0`
  - 静止零位几乎完全贴合人工 `0`
- `pred_visual_fsl`：
  - `MAE = 0.0972 mm`
  - 存在接近 `0.1 mm` 的静态正偏置

说明：

- `/slosh/height` 在静止近零时表现正常
- 当前 `FSL visual` 在 `0401` 上学到了明显的静态 offset

### 5. bag-wise 观察

tracking-only 下，`/slosh/height` 与 `pred_visual_fsl` 的 bag-wise `MAE` 为：

- `Q0_test3`
  - `/slosh/height`: `0.3840 mm`
  - `pred_visual_fsl`: `0.3350 mm`
- `Q10_test1`
  - `/slosh/height`: `0.3210 mm`
  - `pred_visual_fsl`: `0.3250 mm`
- `Q10_test2`
  - `/slosh/height`: `0.3878 mm`
  - `pred_visual_fsl`: `0.3679 mm`
- `Q5_test3`
  - `/slosh/height`: `0.3670 mm`
  - `pred_visual_fsl`: `0.4308 mm`

观察：

- 当前 `FSL visual` 并没有稳定压过 `/slosh/height`
- 在 `Q0_test3`、`Q10_test2` 上略优
- 在 `Q10_test1` 上基本持平
- 在 `Q5_test3` 上明显更差
- `v2` 几何链两条口径依然明显落后，而且覆盖率不足

### 6. 当前阶段结论

基于 `0401` 当前这批严格 holdout 真值：

- `/slosh/height` 仍然是当前最稳的主 baseline
- 当前 `FSL visual pseudolabel` 已经有信号，但还没有稳定超过 `/slosh/height`
- 当前 `v2 center/peak` 不足以作为主证据链
- 真正的 `SL` 训练仍值得做，但目的应是：
  - 用人工 `human_peak_mm` 把 `visual model` 从伪标签空间拉向真实 `MSH`
  - 而不是继续在 `/slosh/height` 伪标签空间里做更多拟合

### 7. 下一步决策

当前最合理的下一步是：

1. 保持现有 holdout 真值不变
2. 开始补 `train-human` bags：
   - `Q0_test1`
   - `Q0_test2`
   - `Q5_test1`
   - `Q5_test2`
3. 用这些训练集标签启动真正的 `SL` 监督学习训练
4. 固定当前 holdout：
   - `val`: `Q0_test3`
   - `test`: `Q10_test1 + Q10_test2 + Q5_test3`

也就是说，当前结论已经足够支持进入真正的 `SL` 训练阶段，但训练集必须来自尚未标注的 train bags。

## 2026-04-06：进入真正的 SL 训练主线

### 1. 当前主线

当前 `0401` 的主线已经收敛为：

1. `holdout-human` 已经完成并分析完毕
2. 保持当前 holdout 不再回流进训练
3. 开始补 `train-human` bags
4. 用 `train-human` 启动真正的 `SL` 监督学习训练
5. 固定用当前 holdout 做验证与测试

### 2. 当前固定 split

- `train-human`：
  - `Q0_test1`
  - `Q0_test2`
  - `Q5_test1`
  - `Q5_test2`
- `val-human`：
  - `Q0_test3`
- `test-human`：
  - `Q10_test1`
  - `Q10_test2`
  - `Q5_test3`
- `static-check`：
  - `Q5_static`

### 3. 当前执行顺序

当前开始进入 `train-human` 标注阶段，执行顺序固定为：

1. `Q0_test1`
2. `Q0_test2`
3. `Q5_test1`
4. `Q5_test2`

原则：

- 只标 `TRACKING` 主运动段
- 主标 `human_peak_mm`
- 不再让 holdout bags 进入训练
- 标完这 4 个 train bags 后，才启动真正的 `SL` 训练

### 4. train-human 重标目录与起始范围

已创建新的 train-human 重标目录：

- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q0_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q0_test2`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q5_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0401/Q5_test2`

这些目录的口径为：

- 复用原始 `debug_session.csv / debug_session.json`
- `photo` 与 `roi_debug` 直接指向原 `debug/0401` 缓存
- `human_labels.csv` 重置为空表头

当前 `TRACKING` 主运动段范围：

- `Q0_test1`：
  - `frame 263 -> 779`
- `Q0_test2`：
  - `frame 322 -> 891`
- `Q5_test1`：
  - `frame 546 -> 1110`
- `Q5_test2`：
  - `frame 344 -> 717`

当前已启动：

- `Q0_test1`
  - viewer start index = `263`

### 5. 当前 train-human 标注进度

已完成：

- `Q0_test1`
  - 已保存 `164` 条 `human_peak_mm` 标签
- `Q0_test2`
  - 已保存 `151` 条 `human_peak_mm` 标签
- `Q5_test1`
  - 已保存 `167` 条 `human_peak_mm` 标签
- `Q5_test2`
  - 已保存 `154` 条 `human_peak_mm` 标签

## 2026-04-06：0401 train-human 完成并进入第一版真正 SL 训练

### 1. train-human 最终标签数量

当前 `0401` 的人工 `human_peak_mm` 标签数量为：

- `Q0_test1`: `164`
- `Q0_test2`: `151`
- `Q5_test1`: `167`
- `Q5_test2`: `154`
- `Q0_test3`: `162`
- `Q5_test3`: `220`
- `Q10_test1`: `195`
- `Q10_test2`: `139`
- `Q5_static`: `78`

合计：

- `1430` 条 `human_peak_mm`

其中真正的 `SL` 训练主 split 为：

- `train`: `636`
- `val`: `162`
- `test`: `554`

### 2. 本轮新增的数据与脚本

已生成完整 human manifest：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all/SL_supervised_manifest.csv`
- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all/SL_supervised_manifest_metadata.json`

已生成兼容 `SL_supervised_common.py` 的 split 文件：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all/SL_human_peak_splits_0401.json`

本轮新增真正的视觉真标签训练脚本：

- `scripts/SL_train_visual_human.py`

定位：

- 输入：单帧 `roi_debug_path`
- 目标：`human_peak_mm`
- 结构：`2-layer CNN + 1-layer MLP + ReLU + linear head`
- 评估：同时输出 `/slosh/height`、`peak affine`、`center affine` 三类 baseline 对照

### 3. 第一版真正 SL 训练结果

输出目录：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_v1`

主要文件：

- `SL_visual_human.pt`
- `SL_visual_human_history.csv`
- `SL_visual_human_predictions.csv`
- `SL_visual_human_summary.json`
- `SL_visual_human_bagwise.csv`

训练概况：

- `device = cpu`
- `best_epoch = 12`
- `best_val_mae = 0.1870 mm`

held-out `test` 相对 `human_peak_mm` 的结果：

- `SL visual`
  - `MAE = 0.2070 mm`
  - `RMSE = 0.3304 mm`
  - `Corr = 0.8794`
- `/slosh/height`
  - `MAE = 0.2852 mm`
  - `RMSE = 0.4944 mm`
  - `Corr = 0.5298`
- `center affine`
  - `MAE = 0.3048 mm`
  - `coverage = 61.37%`
- `peak affine`
  - `MAE = 0.4147 mm`
  - `coverage = 61.37%`

这意味着：

- 第一版真正的 `SL visual` 已经稳定超过 `/slosh/height`
- 相对 `/slosh/height` 的 test `MAE` 改善约为 `27%`
- 相对 `center affine` 的 test `MAE` 改善约为 `32%`

### 4. bag-wise 结果

held-out test bags 上，`SL visual` 的 `MAE` 分别为：

- `Q10_test1`: `0.1426 mm`
- `Q10_test2`: `0.2623 mm`
- `Q5_test3`: `0.2291 mm`

同一批 bags 的 `/slosh/height` 为：

- `Q10_test1`: `0.2649 mm`
- `Q10_test2`: `0.3116 mm`
- `Q5_test3`: `0.2865 mm`

观察：

- `SL visual` 在 3 个 held-out test bags 上全部优于 `/slosh/height`
- `Q10_test1` 提升最明显
- `Q5_test3` 也从先前 `FSL visual < /slosh/height` 翻转成 `SL visual > /slosh/height`

### 5. 当前阶段结论

到当前这一步，`0401` 主线已经出现了关键转折：

- 伪标签阶段：
  - `FSL visual` 还没有稳定超过 `/slosh/height`
- 真标签阶段：
  - 第一版真正的 `SL visual` 已经稳定超过 `/slosh/height`

因此，当前最重要的结论是：

- `human_peak_mm` 真标签监督是有效的
- 当前单帧视觉 `SL` 已经具备继续投入的价值
- 视觉主线现在已经不只是“可行”，而是已经在 held-out test 上优于当前工程 proxy

### 6. 下一步建议

当前最合理的下一步顺序：

1. 基于 `SL_visual_human_predictions.csv` 做 val/test 曲线可视化
2. 检查 `Q10_test2` 和 `Q5_test3` 的主要残差模式
3. 再决定第二轮优化是：
   - 加入短时序 `K-frame` 视觉模型
   - 还是先做更干净的 raw ROI 输入链

也就是说，当前主线已经从“是否值得做真正的 SL”进入“如何继续把真正的 SL 做强”。

## 2026-04-06：SL 曲线诊断与 IMU replay 对照

### 1. 本轮新增脚本

为了沿着当前主线继续做诊断，本轮新增/扩展了两部分：

- 新增 `scripts/SL_plot_visual_human_curves.py`
  - 读取 `SL_visual_human_predictions.csv`
  - 绘制 `val/test` 曲线
  - 支持把离线 replay 曲线叠加进同一张图
- 扩展 `scripts/replay_slosh_model_from_bag.py`
  - 新增 `--ay-source`
    - `slosh_ay_est`
    - `slosh_imu_ay_filtered`
    - `imu_data_y`
  - 新增 `--omega-source`
    - `slosh_omega_est_used`
    - `slosh_imu_omega_z_filtered`
    - `cmd_vel_omega`
    - `odom_omega`

默认行为不变：

- 如果不显式传新参数，仍然按原来的 `/slosh/ax_est`、`/slosh/ay_est`、`/slosh/omega_est_used` 做 replay

### 2. 本轮 replay 口径

本轮对 `0401` 的 held-out bags 单独跑了一版：

- `ay_source = slosh_imu_ay_filtered`
- `omega_source = slosh_omega_est_used`

输出根目录：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_v1/replay_imu_ay`

对应 bags：

- `slosh_Q0_test3_20260401_212122`
- `slosh_Q5_test3_20260401_211230`
- `slosh_Q10_test1_20260401_212244`
- `slosh_Q10_test2_20260401_212357`

### 3. IMU replay 相对 bag 原始 /slosh/height

`bag vs imu-recomputed` 的结果为：

- `Q0_test3`
  - `MAE = 0.1184 mm`
  - `Corr = 0.8378`
- `Q5_test3`
  - `MAE = 0.2336 mm`
  - `Corr = 0.8432`
- `Q10_test1`
  - `MAE = 0.2182 mm`
  - `Corr = 0.6779`
- `Q10_test2`
  - `MAE = 0.1842 mm`
  - `Corr = 0.7872`

说明：

- 只把 `ay` 从 `slosh_ay_est` 切到 `slosh_imu_ay_filtered` 后，重放出的 `/slosh/height` 与 bag 原始 `/slosh/height` 已经出现了明显偏移
- 也就是说，当前 bag 原始 `/slosh/height` 并不是“IMU ay 版”输出

### 4. IMU replay 相对 human_peak_mm

把 `imu replay` 插值到当前 `SL_visual_human_predictions.csv` 的 labeled 时刻后：

- 总体：
  - `count = 689`
  - `MAE = 0.4092 mm`
  - `RMSE = 0.5893 mm`
  - `Corr = 0.4625`

bag-wise：

- `Q0_test3`
  - `MAE = 0.3380 mm`
- `Q10_test1`
  - `MAE = 0.3970 mm`
- `Q10_test2`
  - `MAE = 0.4229 mm`
- `Q5_test3`
  - `MAE = 0.4691 mm`

对比当前 held-out test 的主要结果：

- `SL visual`：
  - `test MAE = 0.2070 mm`
- bag 原始 `/slosh/height`：
  - `test MAE = 0.2852 mm`
- `IMU ay replay`：
  - `val+test MAE ≈ 0.4092 mm`

因此，本轮结论非常明确：

- 仅仅把 `ay` 切到 IMU，并不会让当前 slosh replay 更接近 `human_peak_mm`
- 当前 `IMU ay replay` 反而明显劣于 bag 原始 `/slosh/height`
- 这说明此时的主要提升方向已经不在“把 `/slosh/height` 改成 IMU ay 版”，而在视觉真标签监督本身

### 5. 曲线可视化输出

当前带 IMU replay overlay 的曲线图已生成：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_v1/curves_imu_replay/SL_visual_human_curves_all.png`
- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_v1/curves_imu_replay/SL_visual_human_curves_val.png`
- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_v1/curves_imu_replay/SL_visual_human_curves_test.png`

图中已同时叠加：

- `human_peak_mm`
- `SL visual`
- bag 原始 `/slosh/height`
- `IMU ay replay`
- `center affine`
- `peak affine`

### 6. 当前主线结论

到这一轮为止，当前主线已经进一步收敛：

- 不需要优先回去继续折腾 `/slosh/height` 的 `IMU ay` 版本
- 当前最值得继续推进的是视觉真标签主线

因此，下一步优先级应改成：

1. 继续沿 `SL visual` 主线推进
2. 第二版优先尝试：
   - `K-frame` 短时序视觉模型
3. `raw ROI` 输入链作为后续增强项

也就是说，当前结论支持：

- **下一步先做短时序视觉 SL，而不是先做 IMU-driven slosh model 修补。**

## 2026-04-06：第二版 K-frame 短时序视觉 SL

### 1. 本轮新增脚本

本轮新增：

- `scripts/SL_train_visual_temporal_human.py`

当前实现口径：

- 输入：最近 `K` 帧 `roi_debug_path`
- 当前参数：
  - `history_frames = 5`
  - `history_step = 1`
- 结构：
  - 每帧共享 `2-layer CNN`
  - `frame embedding`
  - `temporal average`
  - `MLP + linear head`
- 目标：
  - `human_peak_mm`

也就是说，这是一个**最小 temporal-average baseline**，不是最终时序结构。

### 2. 第一版 K-frame 结果

输出目录：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_k5_v1`

主要文件：

- `SL_visual_temporal_human.pt`
- `SL_visual_temporal_human_history.csv`
- `SL_visual_temporal_human_predictions.csv`
- `SL_visual_temporal_human_summary.json`
- `SL_visual_temporal_human_bagwise.csv`

训练概况：

- `best_epoch = 10`
- `best_val_mae = 0.2284 mm`
- `train = 636`
- `val = 162`
- `test = 554`

held-out `test` 结果：

- `SL visual temporal (K=5)`：
  - `MAE = 0.2390 mm`
  - `RMSE = 0.4020 mm`
  - `Corr = 0.7380`
- `SL visual single-frame`：
  - `MAE = 0.2070 mm`
  - `RMSE = 0.3304 mm`
  - `Corr = 0.8794`
- `/slosh/height`：
  - `MAE = 0.2852 mm`

结论：

- 当前这版 `K-frame` 短时序模型仍然优于 `/slosh/height`
- 但它**没有超过**当前单帧 `SL visual`

### 3. bag-wise 结果

`K-frame` 的 held-out test `MAE`：

- `Q10_test1`: `0.2083 mm`
- `Q10_test2`: `0.2848 mm`
- `Q5_test3`: `0.2374 mm`

对比单帧 `SL visual`：

- `Q10_test1`: `0.1426 mm`
- `Q10_test2`: `0.2623 mm`
- `Q5_test3`: `0.2291 mm`

也就是说：

- 3 个 held-out test bags 上，当前 `K-frame temporal-average` 都没有超过单帧版

### 4. 当前解释

当前结果说明：

- 不是“时序一定没用”
- 而是**当前这版最小 temporal-average 结构不对**

更可能的原因有两点：

1. 目标是“当前时刻峰值高度”
   - 直接对过去 `K` 帧做平均，会把当前峰值信息抹平
2. 当前输入仍然是 `roi_debug_path`
   - 不是更干净的 raw ROI

因此，这一轮实验的价值在于：

- 排除了“简单 temporal average 就能自然超过单帧”的假设

### 5. 当前主线调整

基于这一轮结果，当前主线应更新为：

1. 保留当前单帧 `SL visual` 作为 `0401` 第一版最佳模型
2. 不继续堆同类 `temporal average` 结构
3. 下一步更值得做的是：
   - `raw ROI` 输入链
   - 或“当前帧锚定”的时序结构，而不是简单平均过去帧

更明确地说：

- **当前下一步不应该是继续加大 `K` 或继续试同类平均时序。**
- **下一步应优先转向 raw ROI，或者设计保留当前帧主导权的 temporal head。**

## 2026-04-06：raw ROI 单帧真标签 SL

### 1. 本轮动机

上一轮 `K-frame temporal-average` 没有超过单帧版，说明当前主问题不在“帧数不够”，而更可能在输入里仍然混入了 `roi_debug_path` 的调试叠加信息。

因此，这一轮主线调整为：

1. 不再继续堆同类 `temporal average`
2. 先补一条最小 `raw ROI` 输入链
3. 用和当前 `SL_visual_human_0401_v1` 完全相同的 human split，重新训练单帧真标签 `SL`

### 2. 实现内容

新增脚本：

- `scripts/SL_export_raw_rectified_roi.py`

该脚本的做法是：

- 读取现有 human manifest：
  - `sl_artifacts/SL_human_peak_manifest_0401_all/SL_supervised_manifest.csv`
- 不重读 bag，不改现有 debug session
- 直接复用每一行里的：
  - `photo_path`
  - `calibration_path`
- 从缓存全图 `photo_path` 中按 calibration 裁出 ROI
- 调用现有 `rectify_roi_and_calibration(...)`
- 导出无叠加的 `raw_rectified_roi_path`
- 写出新的 manifest：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_supervised_manifest_raw_roi.csv`

同时保留 split：

- `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_human_peak_splits_0401.json`

说明：

- 这仍然不是“重新从 bag 导 raw sensor frame”
- 它是“从缓存 full photo 导出的无叠加 rectified ROI”
- 但它已经比 `roi_debug_path` 更接近我们真正想要的视觉输入

### 3. raw ROI 导出结果

artifact：

- raw ROI manifest：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_supervised_manifest_raw_roi.csv`
- metadata：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_raw_rectified_roi_metadata.json`
- raw ROI 图像根目录：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/raw_rectified_roi/`

导出统计：

- `rows = 1430`
- `exported = 1430`
- 第二次重跑时 `reused = 1430`

并已确认：

- `raw_rectified_roi` 尺寸与原 `roi_debug_path` 一致：
  - `329 x 527`

### 4. 训练设置

训练脚本仍沿用：

- `scripts/SL_train_visual_human.py`

只是把 `--image-column` 改成通用 manifest 列名，不再限制为 `roi_debug_path/photo_path`。

本轮训练：

- manifest：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_supervised_manifest_raw_roi.csv`
- split：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi/SL_human_peak_splits_0401.json`
- image column：
  - `raw_rectified_roi_path`
- output：
  - `sl_runs/SL_visual_human_0401_raw_roi_v1/`

### 5. raw ROI 结果

summary：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/SL_visual_human_summary.json`

关键结果：

- `best_epoch = 17`
- `best_val_mae = 0.1852 mm`
- `test MAE = 0.1905 mm`
- `test RMSE = 0.2965 mm`
- `test Corr = 0.8932`

对比上一版 `roi_debug_path` 单帧模型：

- `SL_visual_human_0401_v1`
  - `best_val_mae = 0.1870 mm`
  - `test MAE = 0.2070 mm`
  - `test RMSE = 0.3304 mm`
  - `test Corr = 0.8794`

也就是说：

- raw ROI 单帧版相对上一版单帧模型，`test MAE` 从 `0.2070` 降到 `0.1905 mm`
- 提升约 `8.0%`

对比 `/slosh/height` baseline：

- `/slosh/height test MAE = 0.2852 mm`

所以当前关系已经变成：

- `SL visual single-frame raw ROI`
  **>`**
- `SL visual single-frame roi_debug`
  **>`**
- `/slosh/height`

### 6. bag-wise 结果

bag-wise csv：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/SL_visual_human_bagwise.csv`

raw ROI 单帧 held-out test `MAE`：

- `Q10_test1`: `0.1452 mm`
- `Q10_test2`: `0.2456 mm`
- `Q5_test3`: `0.1958 mm`

对比旧单帧 `roi_debug_path`：

- `Q10_test1`: `0.1426 -> 0.1452 mm`
- `Q10_test2`: `0.2623 -> 0.2456 mm`
- `Q5_test3`: `0.2291 -> 0.1958 mm`

解释：

- `Q10_test1` 略微回退
- `Q10_test2` 和 `Q5_test3` 有明显改善
- 总体 test 汇总仍然明确更优

### 7. 曲线图

本轮已补 raw ROI 版本曲线图：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/curves_imu_replay/SL_visual_human_curves_all.png`
- `sl_runs/SL_visual_human_0401_raw_roi_v1/curves_imu_replay/SL_visual_human_curves_val.png`
- `sl_runs/SL_visual_human_0401_raw_roi_v1/curves_imu_replay/SL_visual_human_curves_test.png`

其中仍叠加了：

- `human_peak_mm`
- `SL visual`
- `/slosh/height`
- affine baselines
- `IMU replay /slosh/height`

### 8. 当前主线更新

基于这一轮结果，当前主线更新为：

1. `SL_visual_human_0401_raw_roi_v1` 成为当前最佳模型
2. `roi_debug_path` 单帧版降为对照基线
3. `K-frame temporal-average` 仍不值得继续
4. 下一步更值得做的是：
   - 在 `raw ROI` 上做“当前帧锚定”的 temporal head
   - 或进一步把输入从缓存 `photo_path` 升级成“直接从 bag 导出的 raw rectified ROI”

更直接地说：

- **现在最合理的下一步，不是回去继续调 `/slosh/height`，也不是继续堆简单 temporal-average。**
- **现在应该围绕 `raw ROI` 这条已经验证有效的输入链继续推进。**

## 2026-04-06：方案文档状态同步与 `/slosh/height` 误差观察

### 1. 方案文档已同步

已同步更新：

- `监督学习方案.md`

本次同步点：

- 把“当前阶段不新增脚本”改成“实际执行进展以 log 和 artifact 为准”
- 把 `0401` 当前真实进度写入 `1.3 当前阶段结论`
- 增加 `12.1 当前执行状态`
- 在 `14. 当前版本结论` 中明确：
  - `raw ROI` 单帧 `SL visual` 已是当前最佳模型
  - 当前 `K-frame temporal-average` 不是下一步主线

### 2. 当前 `/slosh/height` 相对 human peak 的位置

在 `0401` true human held-out test 上：

- 当前最佳 `SL visual raw ROI`：
  - `MAE = 0.1905 mm`
  - `RMSE = 0.2965 mm`
  - `Corr = 0.8932`
- `/slosh/height`：
  - `MAE = 0.2852 mm`
  - `RMSE = 0.4944 mm`
  - `Corr = 0.5298`

因此：

- `/slosh/height` 是**有信号的工程 proxy**
- 但它已经明显落后于当前 best `SL visual`
- 它不能再被当作“已经足够接近 human peak / MSH”的最终证据链

### 3. `/slosh/height` 误差的当前观察

按 test 幅值区间粗看：

- `target <= 0.2 mm`
  - `/slosh/height MAE = 0.1721 mm`
  - 且总体偏高，`bias_mean = +0.1698 mm`
- `0.2 < target <= 0.5 mm`
  - `/slosh/height MAE = 0.1974 mm`
  - 仍偏高，`bias_mean = +0.1178 mm`
- `target > 0.5 mm`
  - `/slosh/height MAE = 0.6221 mm`
  - 转为明显偏低，`bias_mean = -0.5018 mm`

当前解释：

- 小幅值段 `/slosh/height` 倾向于“抬得太高”
- 大幅值段 `/slosh/height` 倾向于“压得太低”
- 这更像是**动态范围压缩/幅值响应不足**，不是单一常数偏置问题

### 4. bag-wise 观察

`/slosh/height` 的 held-out test `MAE`：

- `Q10_test1`: `0.2649 mm`
- `Q10_test2`: `0.3116 mm`
- `Q5_test3`: `0.2865 mm`

对应 `SL visual raw ROI`：

- `Q10_test1`: `0.1452 mm`
- `Q10_test2`: `0.2456 mm`
- `Q5_test3`: `0.1958 mm`

说明：

- `/slosh/height` 三个 bag 都还有明显误差
- `Q10_test2` 是目前更难的一包
- 但问题不止在单一 bag，而是三包上都存在

### 5. 当前可用结论

现在可以更稳地说：

- `/slosh/height` 不是“完全无效”
- 它能提供一定趋势信息
- 但它对当前 human peak / MSH 的解释力还不够强
- 当前最主要的问题更像是：
  - 低幅值偏高
  - 高幅值偏低
  - 相关性不足
  - 难以稳定跟住真正峰值幅值

## 2026-04-06：`/slosh/height` vs 当前最佳 SL visual 的单独对比

### 1. 本轮目的

为了先单独分析 `/slosh/height` 的有效性，这一轮暂时不讨论 IMU，不混入其他 baseline，只比较两条曲线：

- `bag /slosh/height`
- 当前最佳 `SL visual`

这里的当前最佳模型是：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/SL_visual_human.pt`

### 2. 新增产物

新增脚本：

- `scripts/SL_plot_slosh_vs_visual_only.py`

输出目录：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/curves_slosh_vs_visual_only/`

产物包括：

- `SL_vs_slosh_only_all.png`
- `SL_vs_slosh_only_val.png`
- `SL_vs_slosh_only_test.png`
- `SL_vs_slosh_only_summary.json`
- `SL_vs_slosh_only_bagwise.csv`

这些图里只保留：

- `SL visual`
- `bag /slosh/height`

不再画 human、affine、IMU replay。

### 3. val/test 汇总对比

`val`：

- `/slosh/height`
  - `MAE = 0.2898 mm`
  - `RMSE = 0.5115 mm`
  - `Corr = 0.4079`
- `SL visual`
  - `MAE = 0.1852 mm`
  - `RMSE = 0.2769 mm`
  - `Corr = 0.8827`

`test`：

- `/slosh/height`
  - `MAE = 0.2852 mm`
  - `RMSE = 0.4944 mm`
  - `Corr = 0.5298`
- `SL visual`
  - `MAE = 0.1905 mm`
  - `RMSE = 0.2965 mm`
  - `Corr = 0.8932`

因此：

- 当前最佳 `SL visual` 在 val/test 上都明显优于 `/slosh/height`

### 4. bag-wise test 对比

`test` bag-wise：

- `Q10_test1`
  - `/slosh/height = 0.2649 mm`
  - `SL visual = 0.1452 mm`
- `Q10_test2`
  - `/slosh/height = 0.3116 mm`
  - `SL visual = 0.2456 mm`
- `Q5_test3`
  - `/slosh/height = 0.2865 mm`
  - `SL visual = 0.1958 mm`

说明：

- 三个 held-out test bags 上，`SL visual` 都比 `/slosh/height` 更好
- `Q10_test2` 仍然是当前更难的一包

### 5. 一个很关键的观察

按 test 全部 554 帧逐帧看：

- `SL visual` 更接近 human 的帧：`272`
- `/slosh/height` 更接近 human 的帧：`282`

但总体 `MAE` 仍然是 `SL visual` 明显更低。

这说明：

- `SL visual` 不是靠“每一帧都略微更好”赢的
- 它更像是**修掉了 `/slosh/height` 的大失真帧**

### 6. 大失真帧观察

一些代表性坏例：

- `Q10_test1 frame 607`
  - `human = 3.8`
  - `/slosh/height = 0.123`
  - `SL visual = 1.503`
- `Q10_test2 frame 735`
  - `human = 3.0`
  - `/slosh/height = 0.285`
  - `SL visual = 1.224`
- `Q5_test3 frame 635`
  - `human = 2.5`
  - `/slosh/height = 0.232`
  - `SL visual = 1.241`

这些例子说明：

- `/slosh/height` 在真正高峰值时，存在明显压幅
- `SL visual` 也没有完全追上真值
- 但它比 `/slosh/height` 更接近真实峰值

### 7. 幅值区间上的解释

按 test 目标幅值拆分：

- `target <= 0.2 mm`
  - `/slosh/height MAE = 0.1721`
  - `SL visual MAE = 0.1156`
- `0.2 < target <= 0.5 mm`
  - `/slosh/height MAE = 0.1974`
  - `SL visual MAE = 0.1370`
- `target > 0.5 mm`
  - `/slosh/height MAE = 0.6221`
  - `SL visual MAE = 0.4101`

这进一步支持当前判断：

- `/slosh/height` 的核心问题不是简单常数偏置
- 更像是**对大峰值响应不足、幅值被压缩**

### 8. 当前这一步的结论

在只比较：

- `SL visual`
- `/slosh/height`

的条件下，当前结论已经足够明确：

- `/slosh/height` 有趋势信息，但还不够接近当前真实 MSH
- 当前最佳 `SL visual raw ROI` 明显优于 `/slosh/height`
- `/slosh/height` 最值得怀疑的问题是：
  - 动态范围压缩
  - 高峰值严重低估
  - 大失真帧处理能力不足

### 9. 图像级 debug 入口

为了直接看“模型输出和真值差多少”，新增：

- `scripts/SL_render_visual_prediction_debug.py`

当前已针对最佳模型：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/SL_visual_human_predictions.csv`

生成图像级 debug 目录：

- `sl_runs/SL_visual_human_0401_raw_roi_v1/debug_images_test/`

其中包括 4 组图：

- `SL_debug_test_worst_visual.png`
  - 当前 `SL visual` 自己误差最大的 test 样本
- `SL_debug_test_worst_slosh.png`
  - `/slosh/height` 误差最大的 test 样本
- `SL_debug_test_visual_best_gain.png`
  - `SL visual` 相比 `/slosh/height` 获益最大的样本
- `SL_debug_test_slosh_best_gain.png`
  - `/slosh/height` 相比 `SL visual` 更接近真值的样本

每张 tile 都直接标了：

- `human`
- `SL`
- `slosh`
- 两者的绝对误差
- `gain(SL)`，即 `/slosh/height` 误差减去 `SL` 误差

### 10. 当前最值得先看的图像样本

优先级最高的几张是：

- `Q10_test1 frame 607`
  - `human = 3.8`
  - `SL = 1.503`
  - `slosh = 0.123`
- `Q10_test2 frame 735`
  - `human = 3.0`
  - `SL = 1.224`
  - `slosh = 0.285`
- `Q5_test3 frame 635`
  - `human = 2.5`
  - `SL = 1.241`
  - `slosh = 0.232`
- `Q5_test3 frame 750`
  - `human = 2.4`
  - `SL = 1.243`
  - `slosh = 0.139`

这些例子最能说明：

- `/slosh/height` 在高峰值处明显压幅
- `SL visual` 也没有完全打到真值
- 但它更像是在沿着正确方向逼近真实峰值

### 11. 非破坏性 Crop Preview 页面

为了在不破坏现有标注数据集的前提下探索“更小 ROI 是否有助于降低误差”，新增：

- `scripts/SL_build_crop_preview_page.py`

作用：

- 从现有 `SL_supervised_manifest_raw_roi.csv` 中抽取代表性样本图；
- 把少量 `raw_rectified_roi` 复制到独立输出目录；
- 生成一个可本地打开的 `index.html` 页面；
- 页面中可调 `x / y / w / h`，叠加 `zero_y` 参考线与当前 crop 预览；
- 可导出单独 JSON，不改 manifest、不改原图、不改 human labels。

本次产物：

- 页面目录：`sl_artifacts/SL_crop_preview_0401_raw_roi/`
- 页面入口：`sl_artifacts/SL_crop_preview_0401_raw_roi/index.html`
- 元信息：`sl_artifacts/SL_crop_preview_0401_raw_roi/SL_crop_preview_metadata.json`
- 样本图目录：`sl_artifacts/SL_crop_preview_0401_raw_roi/images/`

说明：

- 当前页只是“非破坏性 preview”，用于先人工挑统一 crop。
- 若后续用户确定了统一 crop，再单独做 crop ablation，不覆盖现有 `raw_rectified_roi` 数据集。
- 页面现已支持“锁定当前样本”：
  - 在浏览器里选到想要的样本后，点击 `锁定当前样本`；
  - 当前样本与当前 crop 会写入浏览器本地存储；
  - 刷新页面后仍保持这张样本图，不需要重新选择。

### 12. 固定 Crop Ablation：上下各裁 1/4

由于交互式 preview 在用户当前 IDE 环境中不稳定，本次改为直接做一个固定规则的 crop ablation：

- 保留 rectified ROI 的完整宽度；
- 纵向上裁掉上 1/4，下裁掉下 1/4；
- 即对 `0401` 当前 `329x527` 的 raw ROI，统一变成：
  - `y0 = 132`
  - `h = 263`
  - `x = 0`
  - `w = 329`

本次新增：

- `scripts/SL_export_center_half_roi.py`

作用：

- 从现有 `raw_rectified_roi_path` 派生新的 `center_half_roi_path`；
- 同步写入 `center_half_zero_y_px`；
- 非破坏性，不覆盖现有 raw ROI 数据集。

产物：

- 派生 manifest：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_center_half_roi/SL_supervised_manifest_center_half_roi.csv`
- 元信息：
  - `sl_artifacts/SL_human_peak_manifest_0401_all_center_half_roi/SL_center_half_roi_metadata.json`
- 训练结果：
  - `sl_runs/SL_visual_human_0401_center_half_roi_v1/`

结果对比：

- 原始 `raw ROI` 单帧 best：
  - `val MAE = 0.1852 mm`
  - `test MAE = 0.1905 mm`
  - `test RMSE = 0.2965 mm`
- `center-half ROI` 单帧：
  - `best val MAE = 0.1716 mm`
  - `test MAE = 0.1894 mm`
  - `test RMSE = 0.2848 mm`

bag-wise test：

- `Q10_test1`
  - raw: `0.1452`
  - center-half: `0.1391`
- `Q10_test2`
  - raw: `0.2456`
  - center-half: `0.2720`
- `Q5_test3`
  - raw: `0.1958`
  - center-half: `0.1818`

当前判断：

- “上下各裁 1/4”是有信号的，整体上略优于完整 raw ROI；
- 但提升很小，不是质变；
- 它更像一个可保留的小改进项，而不是当前误差主因的决定性修复。

### 13. 训练集补标候选帧脚本

为了避免后续“继续补数据集”时人工整包翻帧，本次新增：

- `scripts/SL_select_candidate_frames.py`

定位：

- 对指定 `debug_session` 目录直接跑当前 best `SL raw ROI` 模型；
- 不生成整套中间图像，不导出大缓存；
- 只输出候选帧 CSV/JSON，用于指导下一轮人工补标。

当前排序逻辑：

- 只针对未标注 `human_peak_mm` 的帧（`--skip-labeled`）；
- 在每帧上计算：
  - `pred_sl_visual_mm`
  - `slosh_height_mm`
  - `peak_rel_mm_v2`
  - `confidence_v2`
  - `accept_for_peak_report_v2`
- 综合得分由以下几项组成：
  - 大峰值优先
  - `SL` 与 `/slosh/height` 分歧大优先
  - `v2` 接受门失败加分
  - `confidence_v2` 低加分
- 同一 bag 内再做 `min_frame_gap=10` 的去重，避免一段峰值附近连续挑很多帧。

注意：

- `peak_rel_mm_v2` 只在 `accept_for_peak_report_v2=1` 时参与幅值打分；
- 且加了 `peak_v2_cap_mm=5.0`，避免旧规则链极端 outlier 直接把排序带偏。

本次输出目录：

- `/data/a/realsense_validation_v2/sl_candidate_frames/0401_train_hard_examples_v1/`

主要文件：

- `SL_candidate_frames_selected.csv`
- `SL_candidate_frames_all_scored.csv`
- `SL_candidate_frames_summary.json`

本次运行范围：

- `debug_reannotate/0401/Q0_test1`
- `debug_reannotate/0401/Q0_test2`
- `debug_reannotate/0401/Q5_test1`
- `debug_reannotate/0401/Q5_test2`

结果：

- 输入未标帧：`4533`
- 预测成功：`4533`
- 选出候选帧：`160`
  - 默认 `40` 帧 / bag

当前 top 样本特点：

- 大多集中在 `pred_sl_visual_mm` 已经明显抬高、但 `/slosh/height` 仍偏低的帧；
- 同时叠加 `v2_fail + low_conf`，属于典型“高峰值难例”；
- 这批帧比继续均匀抽帧更适合作为下一轮 train-human 补标入口。

### 14. 下一轮补标口径

当前下一轮人工补标，**优先只补训练集**，不动现有 `val/test` 真值包。

原因：

- 当前 `val/test` 已经形成了相对干净的 holdout 评估口径；
- 下一步的主要目标是提升模型训练质量，而不是继续扩张评估集；
- 因此应优先使用 `SL_candidate_frames_selected.csv` 中来自 `train-human` 四个 bag 的高价值候选帧。

当前补标范围：

- `Q0_test1`
- `Q0_test2`
- `Q5_test1`
- `Q5_test2`

当前不优先补：

- `Q0_test3`
- `Q5_test3`
- `Q10_test1`
- `Q10_test2`

说明：

- 除非后续发现 holdout 覆盖明显不足，否则不建议在这一轮继续给 `val/test` 加人工真值；
- 先把训练集高峰值难例补强，再重训并看 held-out test 是否继续下降。

### 15. 候选难例补标后的重训结果

本轮完成：

- 使用 `SL_candidate_frames_selected.csv` 在 `train-human` 四个 bag 上补标；
- train-human peak 数量从原来的 `636` 提升到 `809`：
  - `Q0_test1 = 212`
  - `Q0_test2 = 193`
  - `Q5_test1 = 209`
  - `Q5_test2 = 195`
- 重建了 `SL_human_peak_manifest_0401_all/` 和 `SL_human_peak_manifest_0401_all_raw_roi/`
- 重训输出：
  - `sl_runs/SL_visual_human_0401_raw_roi_v2_after_train_candidates/`

结果对比：

- 原始 best `v1 raw ROI`
  - `train = 636`
  - `best val MAE = 0.1852`
  - `test MAE = 0.1905`
  - `test Corr = 0.8932`
- 候选难例补标后的 `v2`
  - `train = 809`
  - `best val MAE = 0.2015`
  - `test MAE = 0.2128`
  - `test Corr = 0.8708`

bag-wise test：

- `Q10_test1`
  - `v1 = 0.1452`
  - `v2 = 0.1801`
- `Q10_test2`
  - `v1 = 0.2456`
  - `v2 = 0.2947`
- `Q5_test3`
  - `v1 = 0.1958`
  - `v2 = 0.1900`

当前判断：

- 这批“候选难例”补标并没有带来整体提升，反而让 held-out test 变差；
- 当前最可能的原因不是“多标一定更好”，而是：
  - 难例分布过于偏向 `v2_fail + low_conf` 极端帧；
  - 这批新增样本对当前单帧模型来说过难，且分布与 holdout 不完全一致；
  - 只强化这类样本，可能把训练分布带偏了。

结论：

- 当前 best 模型仍然保持为：
  - `sl_runs/SL_visual_human_0401_raw_roi_v1/`
- `v2_after_train_candidates` 保留为一次有价值的失败实验，不作为新的主线 best。

### 16. 阶段性清理与当前保留主线

本轮阶段性清理只删除“明确不再用于主线、且可再现”的垃圾文件，不删除当前 best、失败实验和保留对照实验本体。

本次已删除：

- `scripts/__pycache__/`
- `/data/a/realsense_validation_v2/sl_candidate_frames/0401_train_hard_examples_v1/SL_candidate_frames_all_scored.csv`

此前已删除：

- `scripts/SL_build_crop_preview_page.py`
- `sl_artifacts/SL_crop_preview_0401_raw_roi/`

当前继续保留的主线与对照产物：

- 当前主线 best：
  - `sl_runs/SL_visual_human_0401_raw_roi_v1/`
- 保留的失败实验：
  - `sl_runs/SL_visual_human_0401_raw_roi_v2_after_train_candidates/`
- 保留的小型 crop ablation：
  - `sl_runs/SL_visual_human_0401_center_half_roi_v1/`
- 保留的时序对照实验：
  - `sl_runs/SL_visual_temporal_human_0401_k5_v1/`

当前判断：

- 现阶段最稳的主线仍然是 `raw ROI single-frame v1`；
- `hard-candidate` 补标重训是一次有价值的失败实验，说明“只补极端难例”会把训练分布带偏；
- 若继续补标，应优先尝试“高峰值难例 + 中等难度正常峰值”的混合补标，而不是继续只追 `v2_fail + low_conf` 极端样本。
