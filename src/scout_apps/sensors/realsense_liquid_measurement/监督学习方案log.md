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
