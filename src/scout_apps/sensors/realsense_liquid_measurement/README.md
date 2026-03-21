# realsense_liquid_measurement

`realsense_liquid_measurement` 用于承载 RealSense 液面测量链的离线脚本与后续实时节点。

当前已经实现三个离线脚本：

- `scripts/calibrate_liquid_roi.py`
- `scripts/annotate_liquid_roi.py`
- `scripts/extract_liquid_height_from_bag.py`

第一个脚本的作用是：

- 读取 rosbag 中的 RGB 图像话题
- 导出一帧或多帧 PNG
- 给后续手动标定 `ROI / tube_inner / still_level_px / mm_per_pixel` 提供参考图

第二个脚本的作用是：

- 打开导出的 PNG
- 手动框选 `ROI`
- 手动点击：
  - 左内壁
  - 右内壁
  - 静止液面
  - 可选标尺两点
- 输出一份 YAML 标定文件
- 同时保存一张带标注的参考图

默认输出位置：

- 如果不显式指定 `--out-dir`，脚本会把导出的帧写到 **bag 同目录**
- 也就是说，你当前的 bag 在 `/data/a/bags` 下时，导出结果默认也会落在 `/data/a/bags/<bag_stem>_frames/`

## 当前推荐用法

先用静止 bag 导出几张候选图：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --image-topic /camera/color/image_raw \
  --every 120 \
  --max-frames 5
```

上面这条命令默认会把图片写到：

```text
/data/a/bags/realsense_session_2026-03-21_17-48-52_frames/
```

如果只想导出某一张：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --frame-index 200
```

如果你想强制写到某个指定目录，也可以显式传：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --frame-index 200 \
  --out-dir /data/a/bags/manual_calibration_frames
```

## 第二步：手动标定 ROI 和参考几何

假设你已经导出了一张静止参考图：

```text
/data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000000.png
```

### 模式 A：没有背景标尺，先打通通路

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py \
  --image /data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000000.png \
  --mode px_only
```

这个模式下：

- 只需要点击 `ROI / 左内壁 / 右内壁 / 静止液面`
- 不需要点击标尺
- 输出 YAML 中 `mm_per_pixel` 会写成 `null`

### 模式 B：有背景标尺时再做毫米标定

运行：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/annotate_liquid_roi.py \
  --image /data/a/bags/realsense_session_2026-03-21_17-48-52_frames/frame_000000.png \
  --mode ruler \
  --calibration-distance-mm 10
```

交互顺序是：

1. 先拖框选择 `ROI`
2. 点击左内壁
3. 点击右内壁
4. 点击静止液面上方那条更清晰的表观液面线
5. 如果是 `ruler` 模式，再点击标尺点 1
6. 如果是 `ruler` 模式，再点击标尺点 2
7. 全部完成后按 `Enter` 保存

界面提示规则：

- `still level`
  - 选**上方那条更清晰、后续算法更可能抓到**的液面边界
- `ruler points`
  - 点**背景标尺**
  - 不要点试管表面

默认会输出：

- `frame_000000_calibration.yaml`
- `frame_000000_annotated.png`

## 第三步：批量提取液面像素时序

在 `px_only` 标定已经完成后，可以直接从 bag 里批量提取：

- `height_left_px`
- `height_right_px`
- `height_peak_px`
- `height_left_rel_px`
- `height_right_rel_px`
- `height_peak_rel_px`
- `valid`
- `meniscus_confidence`

运行示例：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml
```

如果你只想先小样本检查通路，可以限制帧数并写到临时目录：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml \
  --out-dir /tmp/realsense_liquid_extract_test \
  --max-frames 60
```

默认输出目录：

- 如果不显式指定 `--out-dir`，结果会写到 **bag 同目录**
- 也就是：
  - `/data/a/bags/<bag_stem>_liquid_measurement/`

如果你想对**静止 bag 自动归零基线**，并直接生成一份修正后的 calibration YAML：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration.yaml \
  --auto-zero-baseline
```

这条命令会：

- 用前 `80` 个有效帧估计静止基线
- 输出建议的 `calibration.still_level_px`
- 默认在原 calibration 同目录生成：
  - `frame_000000_calibration_auto_zero.yaml`

如果你想显式指定输出路径，可以加：

```bash
--write-adjusted-calibration /some/path/frame_000000_calibration_auto_zero.yaml
```

生成修正后的 calibration 后，再用它重新跑提取脚本：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-48-52.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_auto_zero.yaml
```

接下来就可以用这份修正后的 calibration 去跑运动 bag：

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/extract_liquid_height_from_bag.py \
  --bag /data/a/bags/realsense_session_2026-03-21_17-47-55.bag \
  --calibration /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/config/frame_000000_calibration_auto_zero.yaml
```

当前脚本会输出：

- `liquid_height.csv`
  - 每帧的时间戳、有效性、置信度、拟合质量、帧间跳变、左右液面位置、相对静止液面的抬升量
- `liquid_debug.mp4`
  - 只显示 ROI 区域，并叠加：
    - 左右内壁
    - 中央视为可信的拟合带
    - 左右固定内部评估点
    - 静止液面基线
    - 上一帧先验线
    - 每列候选液面点
    - 拟合内点和拟合线
    - 当前帧数值与置信度
- `liquid_height_peak_curve.png`
  - 默认只显示 `height_peak_rel_px` 的逐帧曲线
  - 有效点会按 `meniscus_confidence` 着色，并带颜色条
  - 图中会自动标出当前包里最高峰对应的 `peak_rel_px` 和 `confidence`
  - 无效帧会用灰色 `x` 标出来，但不会作为可信液面高度来解读
  - 如果后续补齐 `mm_per_pixel`，图中还会额外显示 `peak_rel_mm`

如果你只想跑数值和曲线，不想生成 debug 视频，可以加：

```bash
--skip-debug-video
```

如果你只想保留 `csv + debug video`，不生成曲线图，可以加：

```bash
--skip-plot
```

如果你还想在曲线图下面再加一行 `meniscus_confidence`，可以显式加：

```bash
--plot-confidence
```

当前口径说明：

- `height_*_px`
  - 是 **ROI 坐标系内的液面位置**
- `height_*_rel_px`
  - 是相对 `still_level_px` 的抬升量
  - 在图像里液面越高，`y` 越小，所以抬升量按 `still_level_px - current_y` 计算
- `height_*_rel_mm`
  - 只有在背景标尺补齐并得到 `mm_per_pixel` 后才有意义
- `meniscus_confidence`
  - 是当前帧的综合质量分数，综合了覆盖率、拟合残差、斜率和时间门控
- `temporal_jump_px`
  - 是相对上一帧有效结果的最大液面跳变量
- 当前不再把 `height_left_mm / height_right_mm / height_peak_mm` 当成正式输出口径
- 如果 `mm_per_pixel = null`，脚本仍然正常运行，但 `*_rel_mm` 列会留空

## 曲线图怎么看

- 横轴：`frame_index`
  - 表示第几帧
- 主纵轴：`height_peak_rel_px`
  - 表示当前帧最高液面相对静止基线抬升了多少像素
  - 值越大，说明液面抬得越高
  - `0` 表示接近静止液面
- 颜色条：`meniscus_confidence`
  - 每个有效点的颜色表示该帧液面检测置信度
  - 越接近颜色条高端，说明该点越可信
- 灰色虚线：`0`
  - 静止液面基线
- 灰色 `x`
  - `valid=0` 的帧
  - 这些帧算法没有把结果当成可信液面高度
- 峰值标注框
  - 自动标出当前包里最高峰对应的 `peak_rel_px`
  - 同时显示该峰值点的 `meniscus_confidence`
- 上方时间轴：`relative time`
  - 相对第一帧的时间
- 右侧纵轴：`peak_rel_mm`
  - 只有 `mm_per_pixel` 存在时才显示

## CSV 字段说明

- `frame_index`
  - bag 中当前被处理的帧编号
- `stamp`
  - 当前帧时间戳，单位秒
- `valid`
  - 这一帧是否被判定为可信测量
  - `1` 表示可用，`0` 表示不建议直接拿来做峰值分析
- `meniscus_confidence`
  - 当前帧液面检测质量分数，范围大致 `0~1`
  - 它是辅助质量指标，不是液面高度本身
- `threshold_value`
  - 当前帧阈值分割实际使用的灰度阈值
- `valid_columns`
  - 当前帧找到液面候选点的总列数
- `central_valid_columns`
  - 中央可信带内可用于拟合的列数
- `fit_points`
  - 最终参与液面线拟合的点数
- `fit_rms_px`
  - 拟合残差 RMS，越小通常越稳
- `fit_slope`
  - 液面线斜率
  - 转弯或晃动时，这个值可能明显偏离 0
- `temporal_jump_px`
  - 相对上一帧有效结果的最大液面跳变量
  - 过大通常意味着误检或突变
- `temporal_gate_passed`
  - 这一帧是否通过时序门控
  - `1` 表示通过，`0` 表示存在明显时序异常
- `temporal_rejected_columns`
  - 因时序门控被拒掉的候选列数
- `temporal_recovered_columns`
  - 在 fallback/门控后恢复可用的候选列数
- `height_left_px`
  - 拟合液面线在左侧固定内部评估点上的位置
- `height_right_px`
  - 拟合液面线在右侧固定内部评估点上的位置
- `height_peak_px`
  - 当前帧左右评估点中更高那一侧对应的位置
- `height_left_rel_px`
  - 左侧液面相对静止基线的抬升量
- `height_right_rel_px`
  - 右侧液面相对静止基线的抬升量
- `height_peak_rel_px`
  - 当前帧最高液面相对静止基线的抬升量
  - 这是当前最关键的主指标
- `height_left_rel_mm`
  - 左侧相对抬升量的毫米值，仅在有标尺时有效
- `height_right_rel_mm`
  - 右侧相对抬升量的毫米值，仅在有标尺时有效
- `height_peak_rel_mm`
  - 最高液面相对抬升量的毫米值，仅在有标尺时有效
