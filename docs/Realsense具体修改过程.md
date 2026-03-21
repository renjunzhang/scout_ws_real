# Realsense 具体修改过程

本文档记录 RealSense 液面测量链从“当前已有 bag”到“离线脚本第一版跑通”的具体执行清单。

当前阶段目标不是闭环控制，而是：

- 先建立一条**真实液面外部证据链**
- 先把**单试管液面高度时间序列**稳定提出来
- 先服务于 `Q_slosh=0 / 5` 的 A/B 实验对比

## 目录

- 1. 当前阶段总目标
- 2. 离线脚本开始前的人工作业清单
  - 2.1 先选 bag
  - 2.2 先从静止 bag 里截一帧参考图
  - 2.3 必须手动给出的 5 个初始化量
- 3. 写脚本前需要先确认的可视条件
  - 3.1 对焦
  - 3.2 反光
  - 3.3 入镜完整性
  - 3.4 对比度
- 4. 离线脚本第一版的最小目标
  - 4.1 输入
  - 4.2 每帧处理步骤
  - 4.3 输出
- 5. 当前人工执行顺序
- 6. 当前已实现的离线脚本及其用法
  - 6.1 第一步：从 bag 导出 RGB 参考图
  - 6.2 第二步：在导出的 PNG 上手动标定
  - 6.3 `annotate_liquid_roi.py` 的交互顺序
  - 6.4 标定完成后会输出什么
  - 6.5 第三步：批量提取液面像素时序
  - 6.6 当前最推荐的实际操作方式
- 7. 当前阶段不要做的事
- 8. 当前一句话执行建议

---

## 1. 当前阶段总目标

当前第一版只做：

- 单个试管
- 单个相机
- RGB 图像
- 固定 ROI
- 黑墨水液体
- 侧视液面线提取
- 离线分析优先

当前第一版不做：

- 控制闭环
- depth 主量测
- 双相机
- 3D 液面重建
- 自动试管检测

---

## 2. 离线脚本开始前的人工作业清单

在写离线脚本之前，先把下面这些人工初始化项准备好。

### 2.1 先选 bag

先从现有 bag 中挑两类：

- 1 个**静止 bag**
  - 用于做初始化和算法稳定性检查
- 1 个**运动 bag**
  - 用于检查液面波动能不能被合理跟踪

当前优先级建议：

- 第一优先级：静止 bag
- 第二优先级：轻微运动 bag

选择标准：

- 试管完整入镜
- 对焦清楚
- 液面边界清楚
- 反光不要太严重
- 画面不要频繁丢帧

---

### 2.2 先从静止 bag 里截一帧参考图

从静止 bag 中选一帧最干净的图像，作为“人工初始化参考图”。

要求：

- 试管完整可见
- 静止液面平稳
- 背景板和刻度清楚
- 黑墨水液体区域和背景对比明显

这张图后面要用来手动标定。

---

### 2.3 必须手动给出的初始化量

第一版**必须人工给出**下面这些量，不建议自动推断。

#### 1. `roi`

作用：

- 只保留单个试管附近区域
- 减少无关背景和反光干扰

需要给出：

- `roi.x`
- `roi.y`
- `roi.w`
- `roi.h`

要求：

- 框住整个试管可见区域
- 保留少量上下左右边界
- 不要把太多其他结构框进去

#### 2. `tube_inner.x_left`

作用：

- 标记试管左内壁位置
- 后续只在真实液体可能出现的内部区域做液面搜索

要求：

- 尽量标到“内壁”而不是外壁
- 后续算法只在左右内壁之间做列扫描

#### 3. `tube_inner.x_right`

作用：

- 标记试管右内壁位置

要求：

- 与左内壁配套使用
- 保证扫描区域只落在试管内部

#### 4. `calibration.still_level_px`

作用：

- 记录静止液面基线
- 后续 `height_peak_rel_mm` 都是相对这个基线计算

要求：

- 在静止参考图上手动标一条液面基线
- 尽量取试管中部的稳定位置

#### 5. `calibration.mm_per_pixel`（可选）

作用：

- 把像素高度转换成毫米

当前状态说明：

- 如果你**还没有贴背景标尺**，这一项可以先不做
- 当前先跑通流程时，可以先使用 **px_only 模式**
- 等后续贴好背景标尺后，再回头补 `mm_per_pixel`

建议做法：

- 优先用背景标尺做换算
- 不建议直接把试管外径 `28 mm` 当正式标定值

原因：

- 玻璃外壁和内壁并不等价
- 近距离透视和畸变会带来误差
- 背景标尺更直观、更稳

---

## 3. 写脚本前需要先确认的可视条件

在真正开始跑离线脚本前，先人工确认下面这些条件。

### 3.1 对焦

确认：

- 液面边界不是糊的
- 试管边缘不是大面积虚焦

如果不满足：

- 先调整相机距离或焦点
- 不要直接开始调算法

### 3.2 反光

确认：

- 玻璃表面反光不会把液面边界完全盖住

如果反光太重：

- 先调整相机角度
- 先调整背景板材质
- 先调整光照

### 3.3 入镜完整性

确认：

- 试管顶部、液面区域、主要晃动范围都在图像内

如果不满足：

- 先改安装位置或 ROI

### 3.4 对比度

确认：

- 黑墨水液体区域明显比背景更暗
- 阈值分割从视觉上是有可能成功的

如果不满足：

- 先调背景
- 先调光照
- 必要时调墨水浓度

---

## 4. 离线脚本第一版的最小目标

离线脚本第一版不要追求复杂功能，先达到下面这几个目标就够了。

### 4.1 输入

输入至少包括：

- bag 文件路径
- 图像话题名
- 相机信息话题名（第一版可以先不用，但接口留好）
- `roi`
- `tube_inner`
- `still_level_px`
- `mm_per_pixel`

### 4.2 每帧处理步骤

每帧最小处理流程：

1. 读取图像
2. 裁 ROI
3. 转灰度
4. 轻度滤波
5. 阈值分割液体暗区
6. 在试管内部逐列向下搜索液面候选点
7. 在中央可信区域做鲁棒直线拟合
8. 在固定内部评估点上读取左右液面位置
9. 用上一帧结果做时间门控
10. 输出当前帧 `valid` 和 `meniscus_confidence`

### 4.3 输出

第一版至少输出：

- `csv`
  - `stamp`
  - `height_left_px`
  - `height_right_px`
  - `height_peak_px`
  - `height_peak_rel_px`
  - `valid`
  - `meniscus_confidence`
- `debug_video` 或逐帧 debug 图像
  - 叠加 ROI
  - 叠加试管内壁
  - 叠加液面线
  - 叠加当前数值

如果后续已经补了背景标尺，再扩展输出：

- `height_left_rel_mm`
- `height_right_rel_mm`
- `height_peak_rel_mm`

---

## 5. 当前人工执行顺序

建议严格按下面顺序执行。

1. 先挑静止 bag
2. 选一帧静止参考图
3. 手动标：
   - `roi`
   - `tube_inner.x_left`
   - `tube_inner.x_right`
   - `still_level_px`
   - `mm_per_pixel`（有背景标尺时再补）
4. 用静止 bag 跑第一版离线脚本
5. 先看 debug 图像是不是叠对
6. 再看 `csv` 数值是不是平稳
7. 之后再换运动 bag
8. 最后才开始做 A/B 实验指标对比

---

## 6. 当前已实现的离线脚本及其用法

当前已经落下来的三个脚本位于：

- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py`
- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py`
- `/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py`

这三个脚本的定位分别是：

- `calibrate_liquid_roi.py`
  - 从 bag 里导出 RGB 参考图
- `annotate_liquid_roi.py`
  - 在导出的参考图上手动标定 `ROI / tube_inner / still_level_px / mm_per_pixel`
- `extract_liquid_height_from_bag.py`
  - 从 bag 里批量提取液面像素时序，并输出 `csv + debug video`

---

### 6.1 第一步：从 bag 导出 RGB 参考图

当前 bag 路径：

- `/data/a/bags/realsense_session_2026-03-21_17-47-55.bag`
  - 运动场景
- `/data/a/bags/realsense_session_2026-03-21_17-48-52.bag`
  - 纯静止场景

第一步建议优先使用静止 bag：

- `/data/a/bags/realsense_session_2026-03-21_17-48-52.bag`

当前 bag 中实际使用的 RGB 话题是：

- `/camera/color/image_raw`

#### 用法 A：导出多张候选图

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --image-topic /camera/color/image_raw \
  --every 120 \
  --max-frames 5
```

说明：

- 这条命令会每隔 120 帧导出 1 张图
- 最多导出 5 张
- 用来挑一张最适合做手动标定的静止参考图

默认输出目录：

- `/data/a/bags/realsense_session_2026-03-21_17-48-52_frames/`

输出内容包括：

- `frame_000000.png`
- `frame_000120.png`
- ...
- `frames.csv`

#### 用法 B：只导出某一张指定帧

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --image-topic /camera/color/image_raw \
  --frame-index 200
```

说明：

- 适合你已经知道想看哪一帧时使用

默认输出目录同样在：

- `/data/a/bags/realsense_session_2026-03-21_17-48-52_frames/`

#### 用法 C：如果不确定图像话题名，先列出 bag 中图像话题

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --list-topics
```

---

### 6.2 第二步：在导出的 PNG 上手动标定

假设你已经导出了一张静止参考图，例如：

- `/data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000120.png`

当前这个脚本支持两种模式：

- `px_only`
  - 没有背景标尺时先打通通路
- `ruler`
  - 有背景标尺时再做毫米换算

#### 模式 A：先打通通路（没有标尺）

运行：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py \
  --image /data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000120.png \
  --mode px_only
```

这个模式下：

- 只需要点击 `ROI / 左内壁 / 右内壁 / 静止液面`
- 不要求点击标尺点
- 输出 YAML 中 `mm_per_pixel` 会是 `null`

#### 模式 B：后续补毫米标定（有标尺）

运行：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py \
  --image /data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000120.png \
  --mode ruler \
  --calibration-distance-mm 10
```

这里的 `--calibration-distance-mm 10` 表示：

- 你最后点击的两点之间，对应现实中的 `10 mm`

如果你量的是 `20 mm`，那就改成：

```bash
--calibration-distance-mm 20
```

---

### 6.3 `annotate_liquid_roi.py` 的交互顺序

进入脚本后，按下面顺序操作：

1. 先拖框选择 `ROI`
2. 点击左内壁
3. 点击右内壁
4. 点击静止液面
5. 如果是 `ruler` 模式，再点击标尺点 A
6. 如果是 `ruler` 模式，再点击标尺点 B
7. 全部完成后按 `Enter` 保存

辅助按键：

- `r`
  - 重置当前点击点位
- `q` 或 `Esc`
  - 退出，不保存

注意：

- 左内壁、右内壁、静止液面，这 3 个点应当点在 `ROI` 内
- `still_level` 建议点击：
  - **上方那条更清晰、后续算法更可能抓到**的表观液面边界
- 标尺点 A、B 只在 `ruler` 模式下使用
- 如果有背景标尺：
  - 点背景标尺
  - 不要点试管表面

---

### 6.4 标定完成后会输出什么

默认会在原图同目录输出两份文件：

- `frame_000000_calibration.yaml`
- `frame_000000_annotated.png`

其中：

- `*_calibration.yaml`
  - 后续给液面提取脚本读取
- `*_annotated.png`
  - 用来人工复核：你标的位置到底对不对

YAML 中当前最关键的字段包括：

- `roi`
- `tube_inner.x_left`
- `tube_inner.x_right`
- `calibration.still_level_px`
- `calibration.mm_per_pixel`

其中：

- 在 `px_only` 模式下：
  - `mm_per_pixel = null`
- 在 `ruler` 模式下：
  - `mm_per_pixel` 会根据你点击的两点和 `--calibration-distance-mm` 自动计算

---

### 6.5 第三步：批量提取液面像素时序

当你已经完成：

- 导出静止参考图
- 生成 `frame_000000_calibration.yaml`

就可以开始跑真正的液面提取脚本：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml
```

如果当前只想先测试通路，建议先限制帧数：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml \
  --out-dir /tmp/realsense_liquid_extract_test \
  --max-frames 60
```

当前第三个脚本做的事情是：

- 读取 bag 中的 RGB 图像
- 按 calibration YAML 裁剪 `ROI`
- 只在 `tube_inner.x_left ~ x_right` 内搜索液面
- 先做阈值分割
- 必要时用 Sobel-Y 做回退
- 在中央可信区域做液面候选点拟合
- 再在固定内部评估点上读取左右液面位置
- 再用上一帧结果做帧间跳变门控
- 为每一帧输出：
  - `height_left_px`
  - `height_right_px`
  - `height_peak_px`
  - `height_left_rel_px`
  - `height_right_rel_px`
  - `height_peak_rel_px`
  - `meniscus_confidence`
  - `temporal_jump_px`
  - `valid`

默认输出：

- `liquid_height.csv`
- `liquid_debug.mp4`
- `liquid_height_peak_curve.png`
  - 默认只显示每一帧的 `height_peak_rel_px`
  - 无效帧会用灰色 `x` 标出来
  - 如果后续补齐 `mm_per_pixel`，还会显示 `height_peak_rel_mm`

如果不想生成曲线图，可以给 `extract_liquid_height_from_bag.py` 增加：

```bash
--skip-plot
```

如果还想把 `meniscus_confidence` 画成第二行子图，可以加：

```bash
--plot-confidence
```

当前口径说明：

- `height_*_px` 是 ROI 坐标系内的液面位置
- `height_*_rel_px` 是相对 `still_level_px` 的抬升量
- 后续补标尺后，`height_*_rel_mm` 才有物理意义
- `meniscus_confidence` 是覆盖率、拟合质量和时间门控的综合评分
- `temporal_jump_px` 是相对上一帧有效结果的最大跳变
- 当前不再把绝对 `height_left_mm / height_right_mm / height_peak_mm` 当正式输出口径
- 如果 `mm_per_pixel = null`，脚本会继续正常工作，但 `*_rel_mm` 列留空

如果当前静止包跑出来整体还偏在正值，例如：

- `height_peak_rel_px` 长时间稳定在 `+5 px` 左右

说明手工点击的 `still_level_px` 和算法拟合线之间还存在常量偏移。

这时不必手工去猜 `still_level_px`，可以直接用第三个脚本做一次**静止包自动归零基线**：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml \
  --auto-zero-baseline
```

作用是：

- 用前若干个有效静止帧估计更合理的 `still_level_px`
- 自动生成一份新的 calibration YAML
- 后续你应当用这份新的 calibration 重新跑静止包和运动包

默认会在原 calibration 同目录生成：

- `frame_000000_calibration_auto_zero.yaml`

### 6.6 当前最推荐的实际操作方式

当前建议你先这样做：

1. 用静止 bag 导出 3 到 5 张候选图
2. 挑一张最清楚、反光最少的
3. 如果还没有贴标尺，先用 `px_only` 模式做一次人工标定
4. 保存生成的 YAML
5. 保存带标注的 PNG
6. 先人工检查：
   - `ROI` 是否框得合适
   - 左右内壁是否点对
   - 静止液面是否点对
7. 先用第三个脚本跑静止 bag 的前几十帧
8. 先检查 `liquid_height.csv` 和 `liquid_debug.mp4` 是否正常
9. 然后再换运动 bag
10. 等后续贴好背景标尺，再用 `ruler` 模式回补 `mm_per_pixel`

---

## 7. 当前阶段不要做的事

- 不要一开始就自动检测试管
- 不要一开始就上学习模型
- 不要一开始就把视觉结果闭环进控制器
- 不要一开始就做双相机
- 不要一开始就追求 3D 最大液面真值
- 不要还没确认图像质量就先陷入算法调参

---

## 8. 当前一句话执行建议

先拿**静止 bag**做一次人工初始化，把 `ROI / 试管左右内壁 / 静止液面基线` 这 4 个必需量先定死；如果还没有背景标尺，就先让 `mm_per_pixel` 保持 `null`，先把离线脚本通路打通。第一版先追求“提得稳”，不要追求“全自动”。
