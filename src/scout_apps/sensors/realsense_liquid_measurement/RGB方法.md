# RGB 像素液面高度方案

## 核心思路

不依赖深度学习，直接在 RGB 图像中用**逐列竖向梯度**定位气液界面。  
配合试管两侧标尺（物理标尺或试管刻度贴）做一次标定，实现 pixel_y → mm 的分段线性转换。

---

## 椭圆液面问题（必读）

单侧摄像头以斜角观察圆柱试管时，水平圆面的液面在图像中**投影为椭圆**：

```
         摄像头
            \
    ─────────\────── 液面远侧可见边（椭圆上沿）
     试管内液体 \
    ─────────────── 液面近侧边（椭圆下沿）
```

| 特征 | 图像中的位置 | 含义 |
|------|-------------|------|
| 椭圆上沿 | y 坐标较小（图像靠上） | 液面远侧，穿过管壁可见 |
| 椭圆下沿 | y 坐标较大（图像靠下） | 液面近侧，靠摄像头一侧 |
| **椭圆中点** | 上下沿 y 均值 | **真实液位高度** |

如果只检测其中一条边，会引入系统性偏差。假设摄像头倾角使椭圆上下沿相差 10 px，  
换算 0.5 mm/px 时偏差就是 2.5 mm，远超 ±0.3 mm 目标。

**解决方案：**
- **检测**：对每列同时找上下两个边缘，取均值（dual_edge 模式）
- **标定**：对每个刻度点两次——先点上沿，再点下沿，自动取均值

---

## 误差来源分析

| 来源 | 量级估算 | 解决方式 |
|------|----------|---------|
| 只检测一侧椭圆边缘 | 数 mm（斜视角越大越严重） | 使用 dual_edge 模式 |
| 弯月面（管壁两侧液面上翘） | ~0.1–0.5 mm | `center_col_fraction` 只用管中心列 |
| 标定点点错（点边缘而非中点） | 与实际偏差等量 | 2-click 标定自动取均值 |
| 光线变化导致梯度漂移 | ~0.1–0.3 mm | 调整 `gradient_min_strength` |
| 液体晃动时弯月面形状变化 | 残余误差 | 中心列检测可部分抑制 |

---

## 文件位置

```
scripts/
├── RGB_calibrate.py          # 交互式标定工具
└── RGB_infer_from_bag.py     # 从 rosbag 批量检测液面
```

---

## 快速上手

### Step 1：导出参考帧

从 bag 导出一帧液体**静止**时的参考图（用于标定）：

```bash
python scripts/calibrate_liquid_roi.py \
    --bag /data/a/bags/Q0_test1.bag \
    --frame-index 50 \
    --max-frames 1
# 输出: /data/a/bags/Q0_test1_frames/frame_000050.png
```

### Step 2：交互式标定

**推荐用法（`--trace-zero-mm`，自动测量椭圆高度）：**

```bash
python scripts/RGB_calibrate.py \
    --image /data/a/bags/Q0_test1_frames/frame_000050.png \
    --ruler-heights-mm 0,5,10,15,20 \
    --ruler-clicks-per-height 2 \
    --trace-zero-mm \
    --output-yaml /data/a/calib/Q0_rgb_calib.yaml
```

**点击顺序：**

| 步骤 | 操作 | 按键 |
|------|------|------|
| 1–2 | 点左侧管壁上端、下端 | 左键 |
| 3–4 | 点右侧管壁上端、下端 | 左键 |
| 5 | **沿 0mm 椭圆弧线描多个点**（≥3 个，密集点更准）<br>屏幕实时显示：蓝线=远侧边、橙线=近侧边、**绿线=自动计算的中点** | 左键 |
| 6 | 确认 0mm 弧线，锁定中点和椭圆高度 | **Space** |
| 7… | 对每个刻度（5mm, 10mm…）：先点椭圆上沿，再点椭圆下沿 | 左键×2 |
| 最后 | 保存标定 | **Enter** |

> 不带 `--trace-zero-mm`：0mm 也按 2-click 模式（先上沿、后下沿），适合快速标定。  
> 按 `u` 撤销上一次点击，`r` 全部重置，`q`/`Esc` 退出。

**描点模式的额外收益：**  
脚本从 0mm 弧线的 y 范围自动计算 `max_ellipse_height_px`（实测值×1.2 +4px 余量），  
直接写入 YAML，不需要手动估计。

**输出：**
- `Q0_rgb_calib.yaml`：标定结果
- `frame_000050_rgb_calib_annotated.png`：标注预览，核查点击位置

### Step 3：从 bag 推理

```bash
python scripts/RGB_infer_from_bag.py \
    --bag /data/a/bags/Q0_test1.bag \
    --calibration /data/a/calib/Q0_rgb_calib.yaml \
    --out-dir /data/a/results/Q0_test1_rgb \
    --time-offset 5.0 \
    --debug-every 30
```

**输出文件：**

| 文件 | 内容 |
|------|------|
| `*_rgb_heights.csv` | frame_index, stamp_sec, y_px_roi, height_mm, confidence |
| `*_rgb_curve.png` | 液面曲线 + 置信度热图 |
| `debug_frames/debug_XXXXXX.png` | 带检测线的 ROI 可视化（绿线=液位中点，橙线=椭圆半高范围，青线=有效列范围） |

---

## 检测算法（dual_edge 模式）

```
对管内中心列区域 [x_center ± center_col_fraction/2 × tube_width] 中的每一列：

1. 灰度 + 高斯平滑（blur_kernel）
2. Sobel Y 梯度
3. 找所有局部负梯度峰（< -gradient_min_strength）
4. 取最上方峰（椭圆上沿）和最下方峰（间距 ≤ max_ellipse_height_px）的均值
   → 该列的液面 y 估计（双边缘成功）
   若只找到1个峰 → 退化为单峰（记录但权重较低）
5. IQR 过滤所有列的 y 估计，取中位数 → 当前帧液面 y_px
6. 分段线性插值 → h_mm
```

**confidence 计算：**
- `valid_frac`：有效列比例
- `dual_frac`：双边缘成功列比例
- `confidence = valid_frac × (0.5 + 0.5 × dual_frac)`
- confidence 越高说明双边缘检测越稳定，结果越可信

---

## 标定 YAML 结构

```yaml
roi:
  x: 420      # 全图坐标
  y: 180
  w: 120
  h: 300

tube_inner:
  x_left: 18   # 相对 ROI 左上角
  x_right: 98

rotation_deg: 0.8   # 试管轴线偏离垂直方向的角度（度）

calibration:
  mode: piecewise_linear
  reference_points:       # 每点是椭圆上下沿均值（真实液位像素坐标）
    - {y_px_roi: 268.0, h_mm: 0.0}
    - {y_px_roi: 218.5, h_mm: 5.0}
    - {y_px_roi: 169.0, h_mm: 10.0}
    - {y_px_roi: 119.5, h_mm: 15.0}
    - {y_px_roi: 70.0,  h_mm: 20.0}

detection_defaults:
  detection_mode: dual_edge       # dual_edge（推荐）或 single_edge
  max_ellipse_height_px: 40       # 椭圆上下沿最大允许间距（像素）
  center_col_fraction: 0.6        # 只用管径中心 60% 的列
  blur_kernel: 5
  gradient_min_strength: 3.0
  search_top_fraction: 0.02
  search_bottom_fraction: 0.98
  iqr_fence: 1.5
  min_valid_column_fraction: 0.15
```

---

## 常用 CLI 参数速查

### RGB_calibrate.py

| 参数 | 说明 |
|------|------|
| `--image` | 参考帧路径（必填） |
| `--ruler-heights-mm` | 刻度序列，严格递增，e.g. `0,5,10,15,20`（必填） |
| `--ruler-clicks-per-height` | `1` 单次点中点 / `2` 先点上沿再点下沿（推荐，默认 1） |
| `--trace-zero-mm` | 启用 0mm 弧线描点模式（推荐），Space 确认，自动计算中点和椭圆高度 |
| `--output-yaml` | 输出 YAML 路径 |
| `--output-image` | 标注预览图路径 |
| `--max-wall-x-skew-px` | 管轴倾斜警告阈值（px/100px，默认 6.0） |

### RGB_infer_from_bag.py

| 参数 | 说明 |
|------|------|
| `--bag` | rosbag 路径（必填） |
| `--calibration` | 标定 YAML 路径（必填） |
| `--topic` | 图像话题（默认 `/camera/color/image_raw`） |
| `--out-dir` | 输出目录 |
| `--time-offset` | 跳过开头 N 秒 |
| `--every` | 每 N 帧处理一次 |
| `--max-frames` | 最多处理 N 帧 |
| `--debug-every` | 每 N 帧保存 debug 图（0=不保存） |
| `--detection-mode` | `dual_edge`（默认）或 `single_edge` |
| `--max-ellipse-height-px` | 覆盖椭圆最大高度阈值 |
| `--center-col-fraction` | 覆盖中心列比例（0~1） |
| `--gradient-min-strength` | 覆盖最小梯度阈值 |
| `--blur-kernel` | 覆盖平滑内核大小（奇数） |

---

## 调参建议

先跑 `--debug-every 1 --max-frames 50`，打开 debug 图观察绿线（液位中点）位置。

| 现象 | 调整方向 |
|------|----------|
| 绿线正确但 confidence 低（双边缘失败多） | 增大 `--max-ellipse-height-px`（如 40→70） |
| 绿线在管壁/背景上乱跳 | 增大 `--gradient-min-strength`（如 3→8） |
| 大部分帧检测失败 | 减小 `--gradient-min-strength`（如 3→1.5） |
| 检测抖动但趋势正确 | 增大 `--blur-kernel`（如 5→9） |
| 弯月面区域影响大 | 减小 `--center-col-fraction`（如 0.6→0.4，只用更中心的列） |
| 摄像头非常正对液面（椭圆不明显） | 改为 `--detection-mode single_edge` |

**跨批次稳定性：** 同一摄像机固定安装时，同一个 YAML 可复用所有 bag。换试管或重新安装后重新标定即可。

---

## 与监督学习方案对比

| | 监督学习方案（GRU） | RGB 方案 |
|---|---|---|
| 依赖数据 | 需要人工标注 bag | 只需一张参考帧做标定 |
| 跨批次稳定性 | 依赖训练集分布 | 不受批次影响 |
| 精度（已知条件） | MAE ~0.16 mm | 取决于标定质量和对比度 |
| 高峰段表现 | 存在压峰问题 | 直接测量，无压峰 |
| 适用场景 | 离线评估、论文指标 | 快速验证、新 bag 无标注 |

两种方案可互补：RGB 方案先快速判断趋势，监督学习方案用于精确评估。

---

## 2026-04-22 红色液体阶段交接说明

### 这次修改的目的

当前实物实验已经从“黑色液体”切换为“红色液体”。切换目的不是为了继续优化原来的黑边梯度法，而是为了解决之前实物图像中的根本问题：

- 黑色液体 + 黑色刻度 + 黑色标签 + 管壁暗边同时存在
- `RGB_infer_from_bag.py` 的灰度梯度峰值法会稳定吸附到固定黑边
- 即使加入 `near_edge`、`tracker`、收紧搜索带，也仍会高置信度地检测错目标

因此，红色液体阶段的目标变成：

1. 先验证红色液体是否具备更好的视觉可分性
2. 再决定继续沿用“椭圆液面高度法”，还是切换到“红色液柱上边界法”

### 当前红色液体图像的实际情况

对 `/data/a/slosh_bags/real/0422` 的 block2 三条包（Q0/Q5/Q10）抽帧检查后，已经确认：

- 红色液柱主体与背景、黑色刻度线的颜色分离明显好于黑色液体阶段
- 但液面在图像中**不再明显表现为可稳定标注的椭圆上下沿**
- 肉眼观察更接近“一条红色液柱的上边界”，而不是“可测椭圆厚度的液面投影”

这带来一个直接结论：

- **红色液体阶段不再适合把“椭圆上下沿中点”作为主观测量模型**
- 更适合把“红色液柱上边界”作为新的视觉代理量

### 红色液体阶段对旧 RGB 椭圆链的结论

已经试过继续用旧链：

- `RGB_calibrate.py`
- `RGB_infer_from_bag.py`
- `--trace-zero-mm`
- `--near-edge-only`

但当前现象是：

1. 旧方法虽然能跑通
2. 但仍然在识别灰度强边，而不是显式利用红色信息
3. 在红色液体图像上，`trace-zero-mm` 得到的 `measured_ellipse_height_px` 甚至只有 `3 px`
4. 这说明当前图像里并不存在可稳定使用的“椭圆厚度”信息

因此，当前结论是：

- **旧的椭圆液面高度链，不再作为红色液体阶段的主方法**
- 保留代码和文档用于回溯，但不建议继续作为主分析方法调参

### 当前已经新增的红色液体原型脚本

已新增：

- [`red_liquid_infer_from_bag.py`](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py)

该脚本当前做的事情是：

1. 读取 ROI / tube 内边界 / 旋转信息
2. 在 ROI 内做红色 HSV 分割
3. 在试管内部列范围内找红色液柱的顶部边界
4. 输出：
   - `y_top`
   - CSV
   - 时间曲线
   - debug 图

初步试跑结果（block2 Q5）已经证明：

- 不再锁到顶部刻度线或底部黑边
- 能稳定检测红色液柱上边界
- debug 图中检测线贴在红色液柱上沿

这说明红色分割法是当前更有希望的主链。

### 红色液体阶段的正式方案（推荐）

当前推荐不再使用“椭圆厚度 + near_edge 修正”的思路，而改成：

#### 方案：三条竖直高度标尺 + 红色液柱上边界

在 ROI 区域内人工标注三条竖直高度标尺：

- left
- center
- right

每条标尺都标：

- `0 mm`
- `2 mm`
- `4 mm`
- `6 mm`
- `8 mm`

这样每条标尺都提供一组 `y_px -> h_mm` 的参考点。

后续数据处理链路为：

1. 用红色分割方法检测每帧红色液柱上边界
2. 在左/中/右三个横向区域分别得到：
   - `y_top_left`
   - `y_top_center`
   - `y_top_right`
3. 分别用三条标尺做分段线性插值，得到：
   - `h_left_mm`
   - `h_center_mm`
   - `h_right_mm`
4. 再汇总成最终液位指标，例如：
   - `h_mm = median(h_left_mm, h_center_mm, h_right_mm)`

### 为什么要用“三条标尺”

原因不是为了更复杂，而是为了更稳：

- 单条标尺容易受轻微透视和左右折射差异影响
- 三条标尺可以抵抗左右位置误差
- 这比“标轴线点 + 假设整管同一水平线完全一致”更适合当前实物图像

### 当前建议的工作顺序

#### Step 1
完成新的红色液体专用标注工具：

- 选 ROI
- 点左右内壁
- 点三条竖直标尺（left / center / right）

#### Step 2
让 `red_liquid_infer_from_bag.py` 读取这份新标注 YAML

#### Step 3
先只跑 block2 三条包：

- `slosh_Q0_20260422_150442_10block2_Q0.bag`
- `slosh_Q0_20260422_150601_5.bag`
- `slosh_Q0_20260422_150717_10.bag`

原因：

- 这三条包控制侧质量最好
- 最适合先验证新的红色液体视觉链

#### Step 4
如果 block2 跑通，再扩到 10 条包做 compare

### 当前应避免的做法

以下做法不再推荐继续作为主链投入：

- 继续对 `RGB_infer_from_bag.py` 的灰度峰值法做大量补丁
- 强行用 `trace-zero-mm` 去标一个肉眼并不存在的“椭圆厚度”
- 在红色液体阶段继续把 `near_edge_correction_px` 作为核心观测量

### 当前阶段一句话总结

黑色液体阶段的问题是“液面边缘太容易被黑色结构误导”；红色液体阶段已经解决了“颜色可分性”，但同时也暴露出“液面在图像里更像红色液柱上边界，而不是可稳定建模的椭圆液面”。因此当前主方案应切换为：

- **红色液柱上边界检测**
- **三条竖直高度标尺映射到 mm**

而不是继续坚持旧的椭圆液面高度法。

---

## 2026-04-22 当前状态交接（红色液体 compare 阶段）

### 当前主链路

当前红色液体离线分析已经不再以旧的 `RGB_infer_from_bag.py` 椭圆液面链路为主，而是采用：

1. `red_liquid_calibrate.py`
   - 标定 `ROI`
   - 标定试管左右内壁
   - 标定三条竖直高度标尺：`left / center / right`
   - 每条标尺当前使用 `0,2,4,6,8 mm`

2. `red_liquid_infer_from_bag.py`
   - HSV 红色分割
   - 连通域过滤，只保留“接触底部”的主液体区域
   - 在三条横向 band 上分别得到 `h_left / h_center / h_right`
   - CSV 汇总列：
     - `h_mm_final = median(left, center, right)`
     - `h_mm_corr = h_mm_final - h0`
     - `h_mm_smooth_corr = rolling median(h_mm_corr)`
   - PNG 曲线会额外画 `max(L,C,R)`，但单包 CSV 不写 `h_max_lcr` 列
   - `h_max_lcr` 只由 `plot_red_liquid_group_compare.py` 读 CSV 后重新计算，并写入 `group_peak_summary.csv`

### 当前推荐 HSV 参数

当前这轮 0422 红色液体 compare 使用的参数口径为：

```bash
--hue1-low 0 --hue1-high 11 \
--hue2-low 173 --hue2-high 179 \
--sat-min 107 --val-min 99
```

辅助参数口径：

```bash
--bottom-touch-rows 15
--min-component-area 30
--zero-correction-frames 30
--smooth-frames 5
```

说明：

- `sat_min=107` 是重新在原始帧上人工点击红色液柱主体后得到的更保守参数
- `val_min` 当前保留在 `99`，没有采用自动建议的更低值，目的是避免重新放入浅红/高光误检

### 当前 debug 输出

`red_liquid_infer_from_bag.py` 当前会输出两类 debug 图：

1. `debug_frames/`
   - 完整调试图
   - 含文字、遮罩等信息

2. `debug_frames_clear/`
   - 干净版本
   - 只保留原始图像和 `L / C / R` 三条检测线
   - 用于人工判断三条线是否真的贴在液柱上边界

### 当前 compare 批量脚本

当前 compare 主入口脚本：

- [`plot_red_liquid_group_compare.py`](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/plot_red_liquid_group_compare.py)

它当前会对 0422 compare bag 分三组做批量分析：

1. `group1_main_compare`
   - `NOM`
   - `ISR`
   - `FAS_Q5`
   - `PROP2_Q5`

2. `group2_block2_q_sweep`
   - `Q0`
   - `Q5`
   - `Q10`

3. `group3_scheduler_compare`
   - `PROP1_Q5`
   - `FAS_Q5`
   - `PROP2_Q5`

### 当前每组输出的图

当前 compare 输出已经按“组 / 对齐方式 / 文件类型”分层整理。

目录结构：

```text
.tmp_red_group_compare_0422/
  group1_main_compare/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group2_block2_q_sweep/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group3_scheduler_compare/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group_peak_summary.csv
```

其中：

- `csv/`：该组每个 bag 的视觉分析 CSV
- `bag_start/`：以 bag 开始时间为 0 的三张图
- `tracking_start/`：以 `TRACKING` 起点为 0 的三张图
- `motion_start/`：以“速度第一次明显非 0”为 0 的三张图

每个对齐目录里当前固定输出 3 类图：

1. `visual_compare`
   - 同时画：
     - `corr+smooth`
     - `max(L,C,R)`

2. `visual_max_compare`
   - 只画：
     - `max(L,C,R)`

3. `slosh_height_compare`
   - 只画：
     - `/slosh/height`

### 当前已经加入的 3 种时间对齐

当前 compare 图不再只有 `bag_start` 一种时间轴，而是并行输出 3 套：

1. `bag_start`
   - 以 bag 录制开始时间为 0

2. `tracking_start`
   - 以 `/mpc_status` 首次进入 `TRACKING` 为 0

3. `motion_start`
   - 以 `/odom.twist.twist.linear.x` 首次连续超过阈值为 0
   - 当前阈值：
     - `|v| > 0.03 m/s`
     - 连续 `3` 个 odom 样本

结论：

- 当前 `tracking_start` 更适合做“方法开始执行”的比较
- 当前 `motion_start` 更适合看真正运动建立后的液面峰值响应
- `bag_start` 仍然保留，但更偏向诊断用途

### 当前 summary 输出

compare 汇总目录：

- `/home/a/scout_ws/.tmp_red_group_compare_0422`

当前建议先按下面路径找图，不要直接在总目录里翻文件名：

- `group1_main_compare/tracking_start/`
- `group1_main_compare/motion_start/`
- `group2_block2_q_sweep/tracking_start/`
- `group2_block2_q_sweep/motion_start/`

峰值 summary CSV：

- [`group_peak_summary.csv`](/home/a/scout_ws/.tmp_red_group_compare_0422/group_peak_summary.csv)

该表当前已包含：

- `visual_peak_corr_mm`
- `visual_peak_smooth_corr_mm`
- `visual_peak_max_lcr_mm`
- `slosh_height_peak_mm`
- `tracking_start_sec`
- `motion_start_sec`

### 当前建议先看的 6 张图

如果切换对话后要快速接上，不要先翻全部图，先看这 6 张：

#### 主方法比较

1. [`tracking_start/visual_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group1_main_compare/tracking_start/visual_compare.png)
2. [`motion_start/visual_max_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group1_main_compare/motion_start/visual_max_compare.png)
3. [`tracking_start/slosh_height_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group1_main_compare/tracking_start/slosh_height_compare.png)

#### block2 Q 扫描

4. [`tracking_start/visual_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group2_block2_q_sweep/tracking_start/visual_compare.png)
5. [`motion_start/visual_max_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group2_block2_q_sweep/motion_start/visual_max_compare.png)
6. [`tracking_start/slosh_height_compare.png`](/home/a/scout_ws/.tmp_red_group_compare_0422/group2_block2_q_sweep/tracking_start/slosh_height_compare.png)

### 当前阶段的判断边界

当前阶段可以做的是：

- 比较红色液体视觉代理量的趋势和峰值
- 比较 `/slosh/height` 与视觉代理量是否排序一致
- 看不同方法 / 不同 `Q` 下液面包络是否有明显差异

当前还不应直接做的事：

- 把当前时间轴图当成最终论文主图
- 只凭 `/slosh/height` 下主结论
- 忽略 `tracking_start / motion_start` 与 `bag_start` 的差异

### 下一步更高优先级

如果后续继续推进，优先级建议是：

1. 先用当前三种时间对齐图，确认主要比较关系是否稳定
2. 再把最终主图推进到“按固定路径弧长 / 归一化路径进度对齐”
3. 最终论文图优先采用路径进度对齐，而不是纯时间轴

---

## 脚本使用命令速查

本节只记录“怎么跑”，不再重复算法背景。当前有两条链路，使用时不要混用：

- 旧 RGB 椭圆链路：适用于液面椭圆上下沿清晰、颜色不容易被刻度误导的图像
- 红色液体链路：当前 2026-04-22 实物 compare 主链路

### A. 旧 RGB 椭圆链路

#### A1. 从 bag 抽一帧标定图

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/slosh_bags/real/0421/compare/slosh_Q0_20260421_152022_block1_Q0_run1.bag \
  --frame-index 120 \
  --max-frames 1
```

输出目录一般在 bag 同级或脚本打印的位置。选择液面静止、试管清晰的一帧作为 `--image`。

#### A2. 椭圆液面标定

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/RGB_calibrate.py \
  --image /home/a/scout_ws/.tmp_rgb_calib_frames/frame_000120.png \
  --ruler-heights-mm 0,2,4,6,8 \
  --trace-zero-mm \
  --near-edge-only \
  --output-yaml /home/a/scout_ws/Q0_rgb_calib_near.yaml
```

点击规则：

1. 先框选 ROI
2. 点左、右管壁
3. `0 mm` 用描弧方式，`Space` 确认
4. 后续 `2,4,6,8 mm` 按脚本提示点击
5. `Enter` 保存

#### A3. 单个 bag 推理

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/RGB_infer_from_bag.py \
  --bag /data/a/slosh_bags/real/0421/compare/slosh_Q0_20260421_152022_block1_Q0_run1.bag \
  --calibration /home/a/scout_ws/Q0_rgb_calib_near.yaml \
  --debug-every 10 \
  --out-dir /home/a/scout_ws/.tmp_visual_compare/rgb_tracker_nom
```

主要看：

- `*_rgb_heights.csv`
- `*_rgb_curve.png`
- `debug_frames/`

注意：这条链路在黑色液体阶段容易被黑色刻度、标签、管壁暗边误导；当前不作为红色液体 compare 的主链路。

### B. 红色液体链路

#### B1. 从 bag 抽原始帧

先抽静止帧做三标尺标定：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/slosh_bags/real/0422/slosh_Q0_20260422_150601_5.bag \
  --image-topic /camera/color/image_raw \
  --frame-index 0 \
  --out-dir /home/a/scout_ws/.tmp_red_liquid_calib
```

如果要抽高液面或峰值附近帧用于 HSV 取色，可以先从推理 CSV 或曲线里找高峰帧号，再抽对应帧。例如：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/slosh_bags/real/0422/slosh_Q0_20260422_150601_5.bag \
  --image-topic /camera/color/image_raw \
  --frame-index 200 \
  --out-dir /home/a/scout_ws/.tmp_red_liquid_hsv_frames/frame200
```

关键是拿到原始相机帧，不要用带检测线的 debug 图取色。

#### B2. 三条标尺标定

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_calibrate.py \
  --image /home/a/scout_ws/.tmp_red_liquid_calib/frame_000000.png \
  --output-yaml /home/a/scout_ws/red_liquid_3rulers.yaml \
  --ruler-heights-mm 0,2,4,6,8
```

点击流程：

1. 框选 ROI
2. 点左右管壁内边界
3. 点 left 标尺 x 位置
4. 依次点 left 标尺的 `0,2,4,6,8 mm`
5. 点 center 标尺 x 位置
6. 依次点 center 标尺的 `0,2,4,6,8 mm`
7. 点 right 标尺 x 位置
8. 依次点 right 标尺的 `0,2,4,6,8 mm`
9. `Enter` 保存

当前已生成并使用的标定文件：

```text
/home/a/scout_ws/red_liquid_3rulers.yaml
```

#### B3. HSV 取色

在原始帧上点击红色液柱主体，不要点反光、边缘、黑色刻度或 debug 图上的线条：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_sample_hsv.py \
  --images \
  /home/a/scout_ws/.tmp_red_liquid_hsv_frames/frame0/frame_000000.png \
  /home/a/scout_ws/.tmp_red_liquid_hsv_frames/frame200/frame_000200.png \
  /home/a/scout_ws/.tmp_red_liquid_hsv_frames/frame329/frame_000329.png \
  --calibration /home/a/scout_ws/red_liquid_3rulers.yaml
```

当前 0422 compare 使用的 HSV 参数：

```bash
--hue1-low 0 --hue1-high 11 \
--hue2-low 173 --hue2-high 179 \
--sat-min 107 --val-min 99
```

#### B4. 单个 bag 推理

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py \
  --bag /data/a/slosh_bags/real/0422/slosh_Q0_20260422_150601_5.bag \
  --calibration /home/a/scout_ws/red_liquid_3rulers.yaml \
  --debug-every 30 \
  --zero-correction-frames 30 \
  --smooth-frames 5 \
  --hue1-low 0 --hue1-high 11 \
  --hue2-low 173 --hue2-high 179 \
  --sat-min 107 --val-min 99 \
  --out-dir /home/a/scout_ws/.tmp_red_liquid_results/block2_q5
```

主要看：

- `*_red_top.csv`
- `*_red_top_curve.png`
- `debug_frames/`
- `debug_frames_clear/`

当前曲线图默认重点画：

- `corr+smooth`
- `max(L,C,R)`

其中 `debug_frames_clear/` 更适合人工检查检测线是否贴在红色液柱上边界。

#### B5. 0422 compare 批量分析

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/plot_red_liquid_group_compare.py \
  --calibration /home/a/scout_ws/red_liquid_3rulers.yaml \
  --out-dir /home/a/scout_ws/.tmp_red_group_compare_0422
```

输出结构：

```text
/home/a/scout_ws/.tmp_red_group_compare_0422/
  group1_main_compare/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group2_block2_q_sweep/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group3_scheduler_compare/
    csv/
    bag_start/
    tracking_start/
    motion_start/
  group_peak_summary.csv
```

每个对齐目录固定输出：

- `visual_compare.png`
- `visual_max_compare.png`
- `slosh_height_compare.png`

三种对齐含义：

- `bag_start`：以 bag 开始为 0
- `tracking_start`：以 `/mpc_status` 首次进入 `TRACKING` 为 0
- `motion_start`：以 `/odom` 速度首次连续非零为 0

当前正式比较优先看 `tracking_start` 和 `motion_start`，`bag_start` 主要用于诊断。
