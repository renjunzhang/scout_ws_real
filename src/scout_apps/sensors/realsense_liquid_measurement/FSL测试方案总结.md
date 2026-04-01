# FSL 测试方案总结

## 1. 本次 FSL 的定位

本次 FSL 指的是：

- **FSL = 视觉伪标签预训练最小闭环**
- 使用现有 `realsense_liquid_measurement` 的 `manifest / split` 框架
- 不改现有 `B2 dynamics-only` 训练链
- 先做一条最小可运行的视觉分支：

\[
\text{single-frame ROI image} \rightarrow \texttt{/slosh/height}
\]

它的定位不是：

- 不是最终 `MSH` 真值证据链
- 不是 `human_peak_mm` 监督学习主线
- 不是控制器替代方案
- 不是 fusion 模型

它当前只回答一个问题：

> 在固定视角、固定试管、固定背景条件下，单帧 ROI 图像里是否已经包含足够信息去拟合 `/slosh/height` 这条工程 proxy。

---

## 2. 本次 FSL 的解空间

### 2.1 当前允许模型学习的映射

当前 FSL 的解空间是：

\[
\texttt{roi\_debug\_path(single frame)} \rightarrow \texttt{slosh\_height\_mm}
\]

也就是：

- 输入是 **单帧 ROI 图像**
- 标签是 **同一时刻的 `/slosh/height`**
- 任务是 **离线回归**

### 2.2 当前解空间的边界

当前版本 **不包含** 以下能力：

- 不做多帧时序输入
- 不做 `ROI + dynamics` 融合
- 不做 `human_peak_mm` 或 `human_height_mm` 监督
- 不直接预测真实 `MSH`
- 不解决跨视角、跨容器、跨背景泛化
- 不直接使用 raw rectified ROI

### 2.3 当前版本为什么成立

当前版本先用 `roi_debug_path` 而不是重新导 raw ROI，原因是：

- 现有 `manifest` 已经稳定记录了逐帧 ROI 图像路径
- 可以在不改 `B2` 链的前提下最快打通一条视觉闭环
- 先证明“视觉伪标签预训练可运行且有信号”，再决定是否值得往 raw ROI 和时序模型继续投入

因此当前 FSL 的正确理解是：

- **它是视觉预训练分支**
- **它不是最终真值链**

---

## 3. 文件结构

### 3.1 复用的现有文件

本次 FSL 直接复用了现有监督学习基础设施中的两层：

- `scripts/SL_build_supervised_manifest.py`
- `scripts/SL_make_splits.py`

它们负责：

- 统一汇总 debug session
- 构建 manifest
- 按 `bag` 切 train / val / test

### 3.2 本次新增的 FSL 文件

本次新增的视觉 FSL 文件有：

- `scripts/FSL_train_visual_pseudolabel.py`
- `scripts/FSL_plot_visual_pseudolabel_curves.py`

它们分别负责：

- 视觉伪标签训练
- val / test 曲线可视化

### 3.3 当前数据组织目录

#### `fsl_artifacts/`

这个目录保存的是 **训练前的数据组织产物**。

当前目录：

- `fsl_artifacts/FSL_visual_manifest_0330/SL_supervised_manifest.csv`
- `fsl_artifacts/FSL_visual_manifest_0330/SL_supervised_manifest_metadata.json`
- `fsl_artifacts/FSL_visual_manifest_0330/splits/SL_supervised_splits.json`
- `fsl_artifacts/FSL_visual_manifest_0330/splits/SL_supervised_split_groups.csv`

作用：

- 定义有哪些样本
- 定义这些样本来自哪些 bag
- 定义 `train / val / test` 如何按 bag 切分

#### `fsl_runs/FSL_visual_pseudolabel_0330/`

这个目录保存的是 **本次训练与评估结果**。

当前目录：

- `FSL_visual_pseudolabel.pt`
- `FSL_visual_pseudolabel_history.csv`
- `FSL_visual_pseudolabel_predictions.csv`
- `FSL_visual_pseudolabel_summary.json`
- `FSL_visual_pseudolabel_bagwise.csv`
- `curves/FSL_visual_curves_all.png`
- `curves/FSL_visual_curves_val.png`
- `curves/FSL_visual_curves_test.png`

作用：

- 保存 checkpoint
- 保存训练过程
- 保存逐帧预测
- 保存 summary 指标
- 保存 bag-wise 评估
- 保存曲线图

---

## 4. 监督学习结构

### 4.1 输入

当前输入列：

- `roi_debug_path`

具体形式：

- 单帧 ROI 图像
- 读入后转灰度
- resize 到固定尺寸

当前尺寸：

- `96 x 192`

### 4.2 标签

当前目标列：

- `slosh_height_mm`

也就是：

- 用 `/slosh/height` 作为伪标签
- 不使用当前人工 `human_peak_mm`
- 不使用当前人工 `human_height_mm`

### 4.3 模型结构

第一版视觉模型是：

1. `Conv2d(1 -> 16, 3x3) + ReLU + MaxPool2d(2)`
2. `Conv2d(16 -> 32, 3x3) + ReLU + MaxPool2d(2)`
3. `AdaptiveAvgPool2d((8, 16))`
4. `Flatten`
5. `Linear -> hidden(64) + ReLU`
6. `Linear -> 1`

这对应一个非常明确的结构：

- **2 层 CNN**
- **1 层 MLP**
- **ReLU**
- **线性回归头**

### 4.4 训练目标

训练任务是：

\[
\hat h_t \approx \texttt{slosh\_height\_mm}
\]

本质上属于：

- 单值回归
- 视觉伪标签预训练

### 4.5 不变的部分

本次明确保持不变：

- `scripts/SL_train_baseline.py` 不改
- `B2 dynamics-only` 链继续保留为 baseline

因此当前结构是两条并行线：

- `SL_*`：现有 `B2 dynamics-only`
- `FSL_*`：本次新增视觉伪标签预训练

---

## 5. 数据集与测试集

### 5.1 数据范围

本次只使用 `0330` 的 4 个 motion bags：

- `Q0_test1`
- `Q0_test2`
- `Q5_test1`
- `Q5_test2`

对应来源目录是：

- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q0_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q0_test2`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q5_test1`
- `/data/a/realsense_validation_v2/debug_reannotate/0330/Q5_test2`

不使用：

- `Q0_static`

### 5.2 当前 split

本次固定 split 是：

- `train`: `Q0_test1 + Q5_test1`
- `val`: `Q0_test2`
- `test`: `Q5_test2`

这里的切分单位是：

- **bag**

不是：

- 帧随机切分

因此：

- 训练集不会混入测试 bag
- 不存在“把训练集当测试集”的情况

### 5.3 当前样本数

当前 summary 中记录的样本数是：

- `train = 1925`
- `val = 776`
- `test = 677`

这些样本都是：

- 单帧 ROI 图像样本
- 目标值为同帧 `slosh_height_mm`

---

## 6. 当前结果

基于当前 held-out `test bag = Q5_test2`，结果如下：

- `train-mean` baseline：`MAE = 0.0962`
- `peak affine` baseline：`MAE = 0.0870`
- `center affine` baseline：`MAE = 0.0901`
- `visual pseudolabel model`：`MAE = 0.0600`
- `visual pseudolabel model`：`corr = 0.7426`

这说明：

1. 当前视觉伪标签最小闭环已经跑通；
2. 在 held-out test bag 上，视觉模型明显优于简单 baseline；
3. 当前 FSL 是有效的视觉预训练入口。

---

## 7. 当前结论

当前最稳的结论是：

- `FSL` 已经形成一条独立、最小可运行的视觉伪标签预训练闭环；
- 当前解空间是 `single-frame ROI -> /slosh/height`；
- 它已经证明“视觉预训练这条路有信号”；
- 但它仍然不是最终 `MSH` 真值链，也不是最终论文主证据链。

当前更合理的下一步是三选一：

1. 做第二组交换 split，检查当前视觉伪标签结论是否稳定；
2. 把输入从 `roi_debug_path` 升级成更干净的 raw rectified ROI；
3. 后续再把视觉预训练权重迁移到 `human_peak_mm / MSH` 真标签任务。
