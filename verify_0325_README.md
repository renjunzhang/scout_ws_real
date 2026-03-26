# 0325 RealSense v2 与 `/slosh/height` 验证总结

## 范围

- 目录：`/data/a/realsense_validation_v2/verify/0325`
- 标定：`/data/a/realsense_validation_v2/calibration/0325/scene_0325_multiscale_raw.yaml`
- 静止包：
  - `Q0_static`
  - `Q5_static`
- 运动包：
  - `Q0_test1`
  - `Q5_test1`
  - `Q5_test2`
  - `Q5_test3`
- 当前主液面口径：
  - `height_center_rel_mm_bias_corrected_v2`
- 当前固定零位修正：
  - `center_bias_correction_mm_v2 = 0.978398 mm`

## 先看结论

1. 修正后的 `center` 已经可以作为当前主液面代理继续使用。静止包中位数已经回到零附近。
2. `/slosh/height` 在部分工况有效，但跨 bag 稳定性一般。`Q5_test2`、`Q5_test3` 表现明显好于 `Q5_test1`。
3. `peak` 仍然不适合作为主液面，也不能稳定对应 `/slosh/height_pred_max`。
4. `compare` 图里出现的小于 `0` 的值，不是物理液位小于零，而是 `initial zero align` 之后的相对偏差。
5. 现在不建议马上改成“双侧都标 0-25 mm”。当前主要矛盾不是左右标尺尺度不一致，而是检测坏点、report gate 和零位偏置。

## 静止包结果

偏置修正的直接效果是把静止液面重新压回 `0 mm` 附近。

| bag | 帧数 | reportable | 原始 center 中位数 | 修正后 center 中位数 | 修正后 center p90 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `Q0_static` | 631 | 256 | 0.989 mm | 0.011 mm | 0.049 mm |
| `Q5_static` | 762 | 5 | 0.809 mm | 0.000 mm | 0.000 mm |

说明：

- `extract` 输出的 `height_center_rel_mm_v2` 与 `height_center_rel_mm_bias_corrected_v2` 都经过 `positive_only(...)`，本身不会小于 `0`。
- `Q5_static` 的 `reportable` 只有 `5` 帧，说明当前 report gate 仍然偏紧；但主零位已经压住。

## `/slosh/height` 与 RealSense 的误差 / 置信度

这里看的是运动包中，`reportable` 主液面 `center_rel_mm_bias_corrected_v2` 对 `/slosh/height` 的结果。

| bag | reportable | conf 中位数 | conf p90 | MAE | RMSE | Corr | \|error\| 中位数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `Q0_test1` | 139 | 0.780 | 0.801 | 0.187 mm | 0.210 mm | 0.630 | 0.183 mm |
| `Q5_test1` | 76 | 0.772 | 0.835 | 0.184 mm | 0.277 mm | 0.220 | 0.097 mm |
| `Q5_test2` | 91 | 0.780 | 0.795 | 0.016 mm | 0.060 mm | 0.719 | 0.000 mm |
| `Q5_test3` | 154 | 0.769 | 0.808 | 0.197 mm | 0.268 mm | 0.746 | 0.128 mm |

解读：

- `Q5_test2` 最好，说明在这组工况上，`/slosh/height` 和视觉主液面基本一致。
- `Q5_test3` 的相关性也高，但误差比 `Q5_test2` 大，说明形状一致性还可以，幅值上仍有偏差。
- `Q0_test1` 中等。
- `Q5_test1` 最弱，说明模型在这组工况上没有稳定跟上视觉主液面。

### 置信度是不是能解释误差

当前答案是：解释力有限。

- 所有 `reportable` 帧都已经通过 `report_min_confidence`，所以这批帧的 `meniscus_confidence_v2` 基本都集中在 `0.75` 以上。
- 也就是说，`confidence` 在当前流程里主要起“过 gate”作用，而不是在 reportable 帧内部继续细分误差大小。

各包中 `confidence` 与绝对误差 `|center - /slosh/height|` 的相关性：

| bag | corr(conf, abs_error) | 说明 |
| --- | ---: | --- |
| `Q0_test1` | -0.001 | 基本无相关 |
| `Q5_test1` | 0.244 | 弱相关，不能单独解释误差 |
| `Q5_test2` | 0.021 | 基本无相关 |
| `Q5_test3` | 0.460 | 有一定相关，但仍不足以单独做误差代理 |

结论：

- 当前 `confidence` 不是“误差置信度”。
- 它更像“这帧能不能报”的门控量。
- bag 之间的误差差异，不能简单归因于 `confidence`。

## `Q0_test1` 与 `Q5` 三个 test 的 `/slosh/height` 对比

为了可比性，这里统一看 zero-align 后的 `|/slosh/height|` 强度；中位数本身会被 zero-align 压到零附近，不适合比较晃动强弱。

| bag | reportable | \|`/slosh/height`\| p90 | \|`/slosh/height`\| max | \|RS main center\| p90 | \|RS main center\| max | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `Q0_test1` | 139 | 0.251 mm | 0.416 mm | 0.230 mm | 0.791 mm | 中等晃动，视觉与模型量级接近 |
| `Q5_test1` | 76 | 0.389 mm | 0.842 mm | 0.495 mm | 1.100 mm | 4 个运动包里模型给出的主晃动最强，但拟合最弱 |
| `Q5_test2` | 91 | 0.000 mm | 0.440 mm | 0.000 mm | 0.493 mm | 大部分时间接近静止，只有少量峰值；模型和视觉最一致 |
| `Q5_test3` | 154 | 0.128 mm | 0.412 mm | 0.470 mm | 1.269 mm | 模型给出的主晃动强度明显低于视觉尾部/峰部 |

对这 4 个运动包的直接判断：

- `/slosh/height` 的强度排序，如果看 `p90`：
  - `Q5_test1 > Q0_test1 > Q5_test3 > Q5_test2`
- 如果看 `max`：
  - `Q5_test1` 最高，`Q0_test1 / Q5_test2 / Q5_test3` 接近
- `Q5_test2` 的 `p90 = 0`，说明这包的大多数 reportable 时刻本来就接近静止，不能只看 `max`

## 当前 RealSense 还存在哪些问题

有，而且主要不在标尺映射本身。

### 1. 未报出的坏点还存在

虽然 reportable 主序列已经明显变好，但“全部帧”里仍然能看到很大的非报出坏点。例如：

- `Q0_test1` 全帧修正后 `center` 最大值仍到 `11.307 mm`
- `Q5_test3` 全帧修正后 `center` 最大值仍到 `6.874 mm`

这说明：

- 当前 `center` 的主问题已经不是常数偏置
- 但 detector 仍会在非报出帧里产生明显坏点
- 所以当前 gate 还是必要的

### 2. `peak` 依旧不可靠

静止包和运动包里，`peak` 都明显高于主液面，且和 `/slosh/height_pred_max` 的对应关系仍弱。当前它更适合作为诊断量，不适合升格成主输出。

## 是否需要双侧都标 0-25 mm

当前判断：**暂时不需要**。

原因：

1. 这轮主要问题已经被证明是“常数零位偏置”，不是左右尺度不一致。
2. 新场景标定时 `rectified ruler-point x spread` 很小，当前 1D `y -> mm` 映射仍然成立。
3. 现有大误差更多来自检测坏点和 gate，而不是 `piecewise_linear` 标尺映射本身。

只有在下面两种情况同时出现时，双侧标注才值得进入主线：

- 明确观察到左侧液面和右侧液面长期存在稳定、可重复的尺度差
- 并且准备把主输出升级成 `left_mm / center_mm / right_mm` 三条物理量，而不是继续用 1D `center`

## 建议的下一步

1. 保持 `height_center_rel_mm_bias_corrected_v2` 作为当前主液面口径。
2. 如果后续要做物理绝对量分析，再补一版“禁用 initial zero align”的对比图，避免把相对偏差误看成负液位。
3. 下一轮优先继续抓非报出坏点，而不是先重做双侧标尺。
4. 如果继续扩大人工标签，可以验证 `0.978398 mm` 这个 bias 在更多 bag 上是否稳定成立。
