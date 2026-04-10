# 监督学习方案 Log 2

## 2026-04-08：`human_peak_mm` 加权版重训 `weighted_v1`

### 1. 本轮目的

上一轮未加权的 `mm` 主线：

- [SL_visual_human_0401_raw_roi_relabel_refresh_v2](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2)

其主要问题不是整体随机误差，而是对少数高峰帧存在明显的系统性低估：

- `Q10_test1 frame 607`: `3.4 -> 1.061`, residual `-2.339 mm`
- `Q10_test2 frame 735`: `2.2 -> 0.712`, residual `-1.488 mm`
- `Q5_test3 frame 913`: `2.8 -> 1.060`, residual `-1.740 mm`

因此本轮不改：

- 数据
- split
- backbone
- 输入尺寸

只做最小改动：

- 在 `SL_train_visual_human.py` 中加入 `target-amplitude weighted Huber loss`
- 目标是验证它能否缓解高峰段的压低问题

### 2. 本轮训练命令

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/SL_train_visual_human.py \
  --manifest-csv /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi_relabel_refresh_v2/SL_supervised_manifest_raw_roi_motion_only.csv \
  --split-json /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi_relabel_refresh_v2/splits_mm_motion_only/SL_supervised_splits.json \
  --out-dir /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1 \
  --target-column human_peak_mm \
  --image-column raw_rectified_roi_path \
  --enable-target-weighting
```

当前加权配置：

- thresholds: `0.5, 1.0`
- weights: `1.0, 2.0, 4.0`

含义：

- `<= 0.5 mm`: weight `1`
- `(0.5, 1.0] mm`: weight `2`
- `> 1.0 mm`: weight `4`

### 3. 新 run 与关键产物

新 run：

- [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1)

关键文件：

- [SL_visual_human_summary.json](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/SL_visual_human_summary.json)
- [SL_visual_human_history.csv](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/SL_visual_human_history.csv)
- [SL_visual_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/SL_visual_human_history_curves.png)
- [curves_visual_vs_human_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/curves_visual_vs_human_only)
- [curves_slosh_vs_visual_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/curves_slosh_vs_visual_only)
- [curves_visual_human_full](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1/curves_visual_human_full)

### 4. 总体结果

新 run：

- `best_epoch = 6`
- `best_val_mae = 0.1594 mm`
- `test_mae = 0.2150 mm`
- `test_rmse = 0.2988 mm`
- `test_corr = 0.8993`
- `test_bias_mean = +0.0387 mm`

对比未加权 `v2`：

- 旧 [SL_visual_human_0401_raw_roi_relabel_refresh_v2](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2)
  - `test_mae = 0.2290 mm`
  - `test_rmse = 0.3804 mm`
  - `test_corr = 0.8419`
  - `test_bias_mean = -0.0922 mm`

本轮变化：

- `test_mae: 0.2290 -> 0.2150`
- `test_rmse: 0.3804 -> 0.2988`
- `test_corr: 0.8419 -> 0.8993`
- `bias_mean: -0.0922 -> +0.0387`

与 `/slosh/height` 对比：

- `/slosh/height test_mae = 0.2957 mm`
- 加权版仍明显优于 `/slosh/height`

但和当前 official best 相比：

- 当前 official best 仍是 [SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1)
- `best test MAE = 0.1869 mm`

因此：

- **加权版 `weighted_v1` 明显优于未加权 `relabel_refresh_v2`**
- **但仍未超过 current best `0.1869 mm`**

### 5. 对 3 个重点 test bag 的影响

#### 5.1 `Q10_test1`

- MAE：
  - `0.1831 -> 0.1837 mm`
- bias：
  - `-0.0240 -> +0.0605 mm`
- max abs error：
  - `2.3388 -> 1.8652 mm`

说明：

- 平均 MAE 基本持平
- 但极端压峰帧被明显回收
- 同时整体从轻微低估转成轻微高估

#### 5.2 `Q10_test2`

- MAE：
  - `0.2733 -> 0.2220 mm`
- bias：
  - `-0.0391 -> +0.0266 mm`
- max abs error：
  - `1.4878 -> 1.0569 mm`

说明：

- 这是本轮受益最明显的一个 bag

#### 5.3 `Q5_test3`

- MAE：
  - `0.2417 -> 0.2383 mm`
- bias：
  - `-0.1861 -> +0.0269 mm`
- max abs error：
  - `1.7402 -> 1.2902 mm`

说明：

- 平均 MAE 只小幅改善
- 但系统性低估被显著收回
- 极端高峰误差明显下降

### 6. 三个最差高峰帧的直接对比

- `Q10_test1 frame 607`
  - old: `3.4 -> 1.061`, residual `-2.339`
  - new: `3.4 -> 1.535`, residual `-1.865`
- `Q10_test2 frame 735`
  - old: `2.2 -> 0.712`, residual `-1.488`
  - new: `2.2 -> 1.155`, residual `-1.045`
- `Q5_test3 frame 913`
  - old: `2.8 -> 1.060`, residual `-1.740`
  - new: `2.8 -> 1.510`, residual `-1.290`

说明：

- 本轮加权的收益几乎全部体现在：
  - 提高高峰段预测幅值
  - 缓解之前明显的“压峰”

### 7. 分幅值区间的变化

`test_target_bins` 上的 visual 指标变化：

#### `low_<=0.2`

- MAE：
  - `0.1240 -> 0.1851`
- bias_mean：
  - `+0.0932 -> +0.1820`

说明：

- 低幅值精度明显变差
- 加权后更容易把近零小峰值抬高

#### `mid_0.2_0.5`

- MAE：
  - `0.1116 -> 0.1200`
- bias_mean：
  - `-0.0582 -> +0.0476`

说明：

- 中低幅值也有一定退化
- 偏置从轻微低估转成轻微高估

#### `high_>0.5`

- MAE：
  - `0.5887 -> 0.3683`
- bias_mean：
  - `-0.5839 -> -0.3271`
- corr：
  - `0.6343 -> 0.7505`

说明：

- 高峰段改进非常明确
- 这正对应了本轮的核心目标

### 8. 当前判断

本轮结论可以明确写成：

- **加权 loss 是有效方向**
- 它显著缓解了高峰段的系统性低估
- 它没有只是“随机变好”，而是明确改变了残差结构
- 代价是：
  - 低幅值段零点附近误差变大
  - 总体 bias 从负偏转成了轻微正偏

因此当前更准确的理解是：

- 这轮 `weighted_v1` 不是新的 official best
- 但它证明了：
  - 之前的主问题确实包含“回归到均值 / 压峰”
  - 仅通过训练目标加权，就能把高峰段拉回来一大截

### 9. 下一步建议

下一步不建议停在这个版本，而应继续做更平衡的加权或结构改进：

1. 继续保留加权思想，但降低对低幅值段的副作用
   - 可尝试：
     - thresholds 不变，weights 改为 `1.0,1.5,3.0`
     - 或 `1.0,2.0,3.0`
2. 对近零段单独加约束
   - 例如在 `<= 0.2 mm` 样本上附加更强的 zero-region 正则
3. 若继续单帧路线，优先试：
   - 更高输入分辨率
   - 更少的早期 pooling
4. 若要真正解决连续高峰段压低，后续仍应回到：
   - 短时序 `TCN/GRU`
   - 或 `peak_y_rect` 主监督 + 固定 `F(y)` 投回 `mm`

### 10. 当前阶段结论

截至 `2026-04-08`，当前口径更新为：

- official best 仍是：
  - [SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1)
  - `test MAE = 0.1869 mm`
- 当前最新的加权实验是：
  - [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1)
  - `test MAE = 0.2150 mm`
- 它相对未加权 `relabel_refresh_v2` 明显更好
- 它相对 official best 仍未反超
- 但它已经证明：
  - **“压峰”可以通过训练目标加权被明显缓解**

## 2026-04-08：更保守的加权版 `weighted_v2`

### 1. 本轮目的

`weighted_v1` 的主要收益是：

- 高峰段明显回收

但它也带来了明显副作用：

- `low_<=0.2` 区间 MAE 从 `0.1240` 升到 `0.1851`
- 整体 bias 从负偏转成正偏

因此本轮继续保持：

- thresholds 不变：`0.5, 1.0`

只把权重从：

- `1.0, 2.0, 4.0`

改为：

- `1.0, 1.5, 3.0`

目标是：

- 少牺牲低幅值段
- 保留一部分高峰段回收能力

### 2. 本轮训练命令

```bash
python3 /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/scripts/SL_train_visual_human.py \
  --manifest-csv /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi_relabel_refresh_v2/SL_supervised_manifest_raw_roi_motion_only.csv \
  --split-json /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi_relabel_refresh_v2/splits_mm_motion_only/SL_supervised_splits.json \
  --out-dir /home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2 \
  --target-column human_peak_mm \
  --image-column raw_rectified_roi_path \
  --enable-target-weighting \
  --target-weight-thresholds 0.5,1.0 \
  --target-weight-values 1.0,1.5,3.0
```

### 3. 新 run 与关键产物

新 run：

- [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2)

关键文件：

- [SL_visual_human_summary.json](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2/SL_visual_human_summary.json)
- [SL_visual_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2/SL_visual_human_history_curves.png)
- [curves_visual_vs_human_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2/curves_visual_vs_human_only)
- [curves_slosh_vs_visual_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2/curves_slosh_vs_visual_only)

### 4. 总体结果

`weighted_v2`：

- `best_epoch = 5`
- `best_val_mae = 0.1560 mm`
- `test_mae = 0.1988 mm`
- `test_rmse = 0.3281 mm`
- `test_corr = 0.8704`
- `test_bias_mean = -0.0524 mm`

三版对比：

- base `relabel_refresh_v2`
  - `test_mae = 0.2290`
  - `test_corr = 0.8419`
  - `bias = -0.0922`
- `weighted_v1`
  - `test_mae = 0.2150`
  - `test_corr = 0.8993`
  - `bias = +0.0387`
- `weighted_v2`
  - `test_mae = 0.1988`
  - `test_corr = 0.8704`
  - `bias = -0.0524`

说明：

- `weighted_v2` 的总体 MAE 明显优于：
  - base `v2`
  - `weighted_v1`
- 但仍未超过 current best `0.1869 mm`

### 5. 分幅值区间变化

#### `low_<=0.2`

- base `v2`: `MAE=0.1240`, `bias=+0.0932`
- `weighted_v1`: `MAE=0.1851`, `bias=+0.1820`
- `weighted_v2`: `MAE=0.1055`, `bias=+0.0987`

说明：

- `weighted_v2` 明显修复了 `weighted_v1` 在低幅值段的副作用
- 甚至低于 base `v2`

#### `mid_0.2_0.5`

- base `v2`: `MAE=0.1116`, `bias=-0.0582`
- `weighted_v1`: `MAE=0.1200`, `bias=+0.0476`
- `weighted_v2`: `MAE=0.1346`, `bias=-0.0151`

说明：

- `weighted_v2` 在中幅值段比前两版都略差

#### `high_>0.5`

- base `v2`: `MAE=0.5887`, `bias=-0.5839`, `corr=0.6343`
- `weighted_v1`: `MAE=0.3683`, `bias=-0.3271`, `corr=0.7505`
- `weighted_v2`: `MAE=0.4852`, `bias=-0.4613`, `corr=0.6560`

说明：

- `weighted_v2` 仍优于 base `v2`
- 但它把 `weighted_v1` 的一部分高峰回收让回去了

### 6. 对重点 test bag 的影响

#### `Q10_test1`

- base `v2`: `MAE=0.1831`, `bias=-0.0240`
- `weighted_v1`: `MAE=0.1837`, `bias=+0.0605`
- `weighted_v2`: `MAE=0.1408`, `bias=-0.0154`

说明：

- 这是 `weighted_v2` 最明显受益的一个 bag

#### `Q10_test2`

- base `v2`: `MAE=0.2733`, `bias=-0.0391`
- `weighted_v1`: `MAE=0.2220`, `bias=+0.0266`
- `weighted_v2`: `MAE=0.2083`, `bias=-0.1127`

说明：

- `weighted_v2` 的平均 MAE 最好
- 但 bias 重新转回负偏

#### `Q5_test3`

- base `v2`: `MAE=0.2417`, `bias=-0.1861`
- `weighted_v1`: `MAE=0.2383`, `bias=+0.0269`
- `weighted_v2`: `MAE=0.2442`, `bias=-0.0472`

说明：

- 这个 bag 上 `weighted_v2` 并不占优
- 它更像是在总体口径上折中，而不是继续强化高峰回收

### 7. 三个典型高峰帧的对比

- `Q10_test1 frame 607`
  - base: `1.061`
  - `weighted_v1`: `1.535`
  - `weighted_v2`: `1.523`
- `Q10_test2 frame 735`
  - base: `0.712`
  - `weighted_v1`: `1.155`
  - `weighted_v2`: `0.705`
- `Q5_test3 frame 913`
  - base: `1.060`
  - `weighted_v1`: `1.510`
  - `weighted_v2`: `1.327`

说明：

- `weighted_v2` 对极端高峰帧的回收能力弱于 `weighted_v1`
- 尤其 `Q10_test2 frame 735` 基本退回到 base 水平

### 8. 当前判断更新

截至目前，这三版可以这样理解：

1. base `v2`
   - 零点和中低幅值更稳
   - 但高峰压低明显
2. `weighted_v1`
   - 高峰段修复最明显
   - 但低幅值副作用太大
3. `weighted_v2`
   - 是三者里总体最平衡的一版
   - 总体 `test_mae` 也是三者中最好
   - 但它没有保住 `weighted_v1` 那种最强的高峰回收

因此当前最准确的结论是：

- **若只看总体 MAE，当前局部 best 是 `weighted_v2`**
- **若只看高峰段修复力度，`weighted_v1` 更强**
- **若看当前 official best，仍然还是 `0.1869 mm` 的旧主线重评分**

### 9. 下一步建议更新

当前更值得继续的是：

1. 在 `weighted_v1` 和 `weighted_v2` 之间继续扫一到两档更细的权重
   - 优先试：
     - `1.0,1.75,3.5`
     - `1.0,1.5,3.5`
2. 不再建议继续单纯“人工调一个很大或很小的权重”
   - 现在已经看清 tradeoff 方向
   - 下一步应该更系统地找 Pareto 点
3. 如果还要继续单帧路线，最好同时引入：
   - near-zero 区间约束
   - 或高峰段专门 head / 分桶

## 2026-04-08：`weighted_v2` + 高峰低估惩罚 `underpred_v1`

### 1. 本轮目的

`weighted_v2` 的问题仍然是高峰段有压低，因此本轮尝试只针对：

- `target > 1.0 mm`
- 且 `pred < target`

的样本额外加 loss multiplier。

目的：

- 不去改低幅值段
- 只打高峰段的低估尾巴

### 2. 本轮配置

底座：

- `weighted_v2`
  - target weighting = `1.0, 1.5, 3.0`

新增：

- `enable_high_target_underpredict_penalty = True`
- `underpredict_target_threshold = 1.0`
- `underpredict_penalty_multiplier = 2.0`

新 run：

- [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_underpred_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_underpred_v1)

### 3. 结果

- `best_val_mae = 0.1535 mm`
- `test_mae = 0.2299 mm`
- `test_rmse = 0.3633 mm`
- `test_corr = 0.8019`

相对 `weighted_v2`：

- `test_mae: 0.1988 -> 0.2299`
- `test_corr: 0.8704 -> 0.8019`

重点帧也没有受益：

- `Q10_test1 frame 607`
  - `1.523 -> 1.519`
- `Q10_test2 frame 735`
  - `0.705 -> 0.551`
- `Q5_test3 frame 913`
  - `1.327 -> 1.271`

结论：

- **这条“高峰低估额外惩罚”在当前实现下无效**
- 不仅总体变差，关键高峰帧也没有改善
- 当前不建议继续沿这个具体损失方向追加试验

## 2026-04-08：`weighted_v2` 高分辨率版 `hires_v1`

### 1. 本轮目的

前面几轮现象说明：

- 高峰段问题更像是 crest 细节在单帧 CNN 中被抹平
- 而不一定只是 loss 权重问题

因此本轮保持：

- `weighted_v2` 的 loss 口径

只改输入分辨率：

- `96x192 -> 128x256`
- `batch_size = 16`

### 2. 新 run 与配置

新 run：

- [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1)

关键配置：

- `image_height = 128`
- `image_width = 256`
- `batch_size = 16`
- target weighting 仍为：
  - `1.0, 1.5, 3.0`

关键产物：

- [SL_visual_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1/SL_visual_human_history_curves.png)
- [curves_visual_vs_human_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1/curves_visual_vs_human_only)
- [curves_slosh_vs_visual_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1/curves_slosh_vs_visual_only)

### 3. 总体结果

`hires_v1`：

- `best_epoch = 21`
- `best_val_mae = 0.1810 mm`
- `test_mae = 0.2086 mm`
- `test_rmse = 0.2781 mm`
- `test_corr = 0.8896`
- `test_bias_mean = +0.0303 mm`

相对 `weighted_v2`：

- `test_mae: 0.1988 -> 0.2086`
- `test_rmse: 0.3281 -> 0.2781`
- `test_corr: 0.8704 -> 0.8896`

说明：

- 总体 MAE 没有赢 `weighted_v2`
- 但 RMSE 和 corr 更好
- 这意味着它更像是：
  - 压缩了极端尾部
  - 但付出了一部分低幅值段代价

### 4. 对重点 bag / worst frames 的影响

#### `Q10_test1`

- `weighted_v2`
  - `MAE = 0.1408`
  - `max_abs = 1.8769`
- `hires_v1`
  - `MAE = 0.1809`
  - `max_abs = 1.4353`

说明：

- 平均 MAE 变差
- 但最大误差明显下降

#### `Q10_test2`

- `weighted_v2`
  - `MAE = 0.2083`
  - `max_abs = 1.4952`
- `hires_v1`
  - `MAE = 0.2870`
  - `max_abs = 0.9808`

说明：

- 最大误差显著下降
- 但平均 MAE 明显变差

#### `Q5_test3`

- `weighted_v2`
  - `MAE = 0.2442`
  - `p95 = 0.9357`
  - `max_abs = 1.4734`
- `hires_v1`
  - `MAE = 0.1836`
  - `p95 = 0.4761`
  - `max_abs = 1.1965`

说明：

- 这是 `hires_v1` 受益最明显的一个 bag

典型高峰帧：

- `Q10_test1 frame 607`
  - `weighted_v2: 1.523`
  - `hires_v1: 1.965`
- `Q10_test2 frame 735`
  - `weighted_v2: 0.705`
  - `hires_v1: 1.524`
- `Q5_test3 frame 913`
  - `weighted_v2: 1.327`
  - `hires_v1: 1.900`

说明：

- 在我们最在意的 3 个 worst frames 上，`hires_v1` 全部明显优于 `weighted_v2`

### 5. 分幅值区间结果

相对 `weighted_v2`：

#### `low_<=0.2`

- `weighted_v2`
  - `MAE = 0.1055`
  - `bias = +0.0987`
- `hires_v1`
  - `MAE = 0.1758`
  - `bias = +0.1705`

#### `mid_0.2_0.5`

- `weighted_v2`
  - `MAE = 0.1346`
- `hires_v1`
  - `MAE = 0.1399`

#### `high_>0.5`

- `weighted_v2`
  - `MAE = 0.4852`
  - `bias = -0.4613`
  - `corr = 0.6560`
- `hires_v1`
  - `MAE = 0.3473`
  - `bias = -0.2765`
  - `corr = 0.8000`

说明：

- `hires_v1` 显著改善了高峰段
- 但也显著伤害了低幅值段

### 6. 关于“每一帧误差都压到 0.3 mm 内”

截至当前几版实验，在 held-out test `554` 帧上：

- base `v2`: `|err| > 0.3 mm` 的帧数为 `101`
- `weighted_v2`: `104`
- `hires_v1`: `128`

因此当前可以明确说：

- **在现有 single-frame `mm` 回归口径下，离“每一帧都 < 0.3 mm”还很远**
- 目前更现实的优化目标仍然是：
  - 先降低高峰段系统性低估
  - 再通过结构改动收缩低幅值段副作用

### 7. 当前阶段判断更新

截至目前：

- 若看总体 test MAE：
  - 当前局部 best 仍是 [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2)
  - `test MAE = 0.1988 mm`
- 若看高峰段/极端尾部：
  - [SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1)
  - 更值得关注
- 若看 official best：
  - 仍然是 [SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1)
  - `0.1869 mm`

### 8. 下一步建议更新

当前不建议继续在 single-frame `human_peak_mm` 上盲目小修小补：

1. `underpredict penalty` 这条线暂时收住
2. 如果还要继续 single-frame，最值得的不是再调 loss，而是：
   - 高分辨率输入
   - 更少 pooling 的 backbone
3. 但若目标真的是进一步逼近：
   - 高峰段和低幅值段同时兼顾
   - 甚至尝试逼近“每帧 < 0.3 mm”

更有价值的下一步应优先转向：

- 短时序模型 `TCN / GRU`
- 或 `peak_y_rect` 主监督，再固定投回 `mm`

## 2026-04-08：raw ROI 时序 `GRU + current-frame anchor + weighting` 首轮 `temporal_gru_anchor_weighted_v1`

### 1. 本轮目的

前面单帧线已经说明：

- `weighted_v2` 是当前 retrain 主线里最平衡的单帧版本
- `hires_v1` 明显改善高峰段，但整体 MAE 没赢
- 若想继续逼近“低幅值和高峰段同时兼顾”，仅靠单帧 loss 微调已经不够

因此本轮转向真正时序结构，但遵守一个核心约束：

- 标签仍然是**当前帧**的 `human_peak_mm`

所以时序设计不再做 `temporal average`，而改成：

- `shared CNN encoder`
- `GRU temporal head`
- `current-frame anchor`
- moderate target weighting：`1.0, 1.5, 3.0`

### 2. 工程改动与关键修正

本轮不仅是新 run，也补了时序训练脚本的两类问题：

1. `scripts/SL_train_visual_temporal_human.py` 现在支持：
   - `--temporal-head {mean,gru,tcn}`
   - `--anchor-current-frame`
   - `--enable-target-weighting`
2. 修掉了 raw ROI 时序读取链路的一个根问题：
   - `raw_rectified_roi_path` 不存在于 `debug_session.csv`
   - 只存在于 supervised manifest
   - 因此 raw ROI 时序不能再靠 `debug_session.csv` 组历史帧
   - 现在改为**优先从 manifest 自身按 session/frame_index 建历史索引**

这一步很关键，否则 raw ROI temporal 在工程上其实跑不通。

### 3. 新 run 与训练配置

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1)

训练命令要点：

- `image_column = raw_rectified_roi_path`
- `target_column = human_peak_mm`
- `history_frames = 7`
- `history_step = 1`
- `temporal_head = gru`
- `anchor_current_frame = true`
- target weighting：
  - `<= 0.5 mm -> 1.0`
  - `(0.5, 1.0] mm -> 1.5`
  - `> 1.0 mm -> 3.0`

训练结果：

- `epochs_ran = 18`
- `best_epoch = 12`
- `best_val_mae = 0.1626 mm`

关键产物：

- [SL_visual_temporal_human_summary.json](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/SL_visual_temporal_human_summary.json)
- [SL_visual_temporal_human_history.csv](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/SL_visual_temporal_human_history.csv)
- [SL_visual_temporal_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/SL_visual_temporal_human_history_curves.png)
- [curves_visual_vs_human_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/curves_visual_vs_human_only)
- [curves_slosh_vs_visual_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/curves_slosh_vs_visual_only)
- [curves_visual_human_full](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1/curves_visual_human_full)

### 4. 总体结果

`temporal_gru_anchor_weighted_v1`：

- `test_mae = 0.1731 mm`
- `test_rmse = 0.2502 mm`
- `test_corr = 0.9072`
- `test_bias_mean = -0.0085 mm`
- `|err| > 0.3 mm` 的 test 帧数：`75 / 554`

对比几条关键基线：

- official best `relabel_refresh_eval_v1`
  - `test_mae = 0.1869 mm`
  - `rmse = 0.2843`
  - `corr = 0.9106`
  - `|err| > 0.3 mm = 93`
- `weighted_v2`
  - `test_mae = 0.1988 mm`
  - `rmse = 0.3281`
  - `corr = 0.8704`
  - `|err| > 0.3 mm = 104`
- `hires_v1`
  - `test_mae = 0.2086 mm`
  - `rmse = 0.2781`
  - `corr = 0.8896`
  - `|err| > 0.3 mm = 128`

因此这轮可以明确写结论：

- **这是当前新主线里的最佳结果**
- **它已经超过之前 official best `0.1869 mm`**
- **它也是当前所有实验里 test RMSE 最低的一版**

### 5. 对 test bags 的影响

#### `Q10_test1`

- official best
  - `MAE = 0.1571`
  - `max_abs = 1.8970`
- `weighted_v2`
  - `MAE = 0.1408`
  - `max_abs = 1.8769`
- temporal
  - `MAE = 0.1447`
  - `max_abs = 1.7372`

说明：

- 平均 MAE 与 best single-frame 非常接近
- 最大误差比 official best 和 `weighted_v2` 都更小

#### `Q10_test2`

- official best
  - `MAE = 0.2163`
  - `max_abs = 0.9763`
- `weighted_v2`
  - `MAE = 0.2083`
  - `max_abs = 1.4952`
- temporal
  - `MAE = 0.1702`
  - `max_abs = 0.8202`

说明：

- 这是 temporal 收益最明显的一个 bag
- 平均误差和最大误差都显著改善

#### `Q5_test3`

- official best
  - `MAE = 0.1949`
  - `max_abs = 1.4001`
- `hires_v1`
  - `MAE = 0.1836`
  - `max_abs = 1.1965`
- temporal
  - `MAE = 0.2002`
  - `max_abs = 1.1752`

说明：

- `Q5_test3` 上，平均 MAE 不是当前最佳
- 但最大误差进一步下降
- 也就是 temporal 更像在收缩尾部，而不是单纯压平均值

### 6. 三个重点高峰帧

- `Q10_test1 frame 607`
  - target `3.4`
  - official best: `1.503`
  - `weighted_v2`: `1.523`
  - `hires_v1`: `1.965`
  - temporal: `1.663`
- `Q10_test2 frame 735`
  - target `2.2`
  - official best: `1.224`
  - `weighted_v2`: `0.705`
  - `hires_v1`: `1.524`
  - temporal: `1.678`
- `Q5_test3 frame 913`
  - target `2.8`
  - official best: `1.400`
  - `weighted_v2`: `1.327`
  - `hires_v1`: `1.900`
  - temporal: `2.015`

说明：

- temporal 不是每个极端帧都最好
- 但在 `Q10_test2` 和 `Q5_test3` 的重点高峰帧上，已经明显优于此前 best single-frame

### 7. 分幅值区间结果

#### `low_<=0.2`

- official best: `MAE = 0.1155`
- `weighted_v2`: `0.1055`
- temporal: `0.1143`

说明：

- temporal 没有像 `hires_v1` 那样明显伤害低幅值段
- 低幅值基本维持在 official best 同级别

#### `mid_0.2_0.5`

- official best: `MAE = 0.1289`
- `weighted_v2`: `0.1346`
- temporal: `0.1550`

说明：

- 中间幅值段仍有可继续优化空间

#### `high_>0.5`

- official best: `MAE = 0.4071`
- `weighted_v2`: `0.4852`
- `hires_v1`: `0.3473`
- temporal: `0.3353`

说明：

- **temporal 现在是高峰段最好的版本**
- 这正是它能把总体 MAE 拉过 official best 的主要原因

### 8. 当前判断更新

这轮结果已经把结论改写了：

1. 之前“最小 temporal-average 结构不值得继续”这个判断仍然成立
2. 但这不等于“时序本身没价值”
3. 真正有效的是：
   - `raw ROI`
   - `current-frame anchor`
   - 不做简单平均
   - 同时保留 moderate target weighting

截至当前：

- 新 current best：
  - [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1)
  - `test MAE = 0.1731 mm`
- 它已经超过旧 official best：
  - [relabel_refresh_eval_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_v1/relabel_refresh_eval_v1)
  - `0.1869 mm`

但也仍需保持现实判断：

- 现在还**没有**做到“每一帧都 < 0.3 mm”
- 只是 `|err| > 0.3 mm` 已经从：
  - official best 的 `93`
  - 降到 temporal 的 `75`

### 9. 下一步建议更新

当前最值得继续沿着这条时序线推进，而不是回去盲扫单帧：

1. 在这个 run 基础上优先试：
   - `K=5` vs `K=7`
   - `GRU` vs `TCN`
2. 若继续以“收缩尾部”为目标，可优先看：
   - `Q10_test1 frame 607`
   - `Q5_test3 31~34s` 连续高峰段
3. 若目标转向“进一步压缩 `|err| > 0.3 mm` 的帧数”，下一步不能只盯总体 MAE，还要把：
   - 高峰段 max/p95 abs error
   - bag-wise worst segments
   一起纳入判断

## 2026-04-08：`GRU + anchor` 缩短到 `K=5` 的对照 `temporal_gru_anchor_weighted_k5_v1`

### 1. 本轮目的

上一轮 `K=7` 已经拿到新 best：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1)
- `test MAE = 0.1731 mm`

但它的 `mid_0.2_0.5` 区间仍不算最优，因此本轮只改一个变量：

- `history_frames: 7 -> 5`

目标是验证：

- 更短历史是否能更“当前帧导向”
- 同时保住高峰段收益

### 2. 新 run 与结果

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1)

关键产物：

- [SL_visual_temporal_human_summary.json](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1/SL_visual_temporal_human_summary.json)
- [SL_visual_temporal_human_history.csv](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1/SL_visual_temporal_human_history.csv)
- [SL_visual_temporal_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1/SL_visual_temporal_human_history_curves.png)

训练结果：

- `epochs_ran = 9`
- `best_epoch = 3`
- `best_val_mae = 0.1535 mm`
- `test_mae = 0.2257 mm`
- `test_rmse = 0.3273 mm`
- `test_corr = 0.8497`
- `|err| > 0.3 mm` 的 test 帧数：`116 / 554`

### 3. 相对 `K=7` 的对比

`K=7`：

- `test_mae = 0.1731`
- `rmse = 0.2502`
- `corr = 0.9072`
- `|err| > 0.3 mm = 75`

`K=5`：

- `test_mae = 0.2257`
- `rmse = 0.3273`
- `corr = 0.8497`
- `|err| > 0.3 mm = 116`

三个重点高峰帧也全面退化：

- `Q10_test1 frame 607`
  - `K=7: 1.663`
  - `K=5: 1.620`
- `Q10_test2 frame 735`
  - `K=7: 1.678`
  - `K=5: 0.741`
- `Q5_test3 frame 913`
  - `K=7: 2.015`
  - `K=5: 1.405`

### 4. 当前结论更新

这轮是一个明确的负结果：

- **在当前 `GRU + current-frame anchor + weighting` 口径下，`K=5` 明显不如 `K=7`**
- 更短 history 没有带来更好的泛化，反而丢掉了高峰段收益

更重要的是，它暴露出一个选择风险：

- `K=5` 的 `best_val_mae = 0.1535`
- 甚至表面上还优于 `K=7` 的 `0.1626`
- 但 held-out test 却大幅更差

因此可以明确补一条方法论结论：

- **当前 temporal 配置不能只按 val MAE 选 history 长度**
- 对 `K` 的判断必须同时看 held-out test，尤其是：
  - 高峰段 MAE
  - `|err| > 0.3 mm` 帧数
  - 重点 worst frames

## 2026-04-08：`K=7` 下的结构/加权收敛对照

### 1. 本轮目的

在 `K=7 GRU + anchor + weighting` 拿到新 best 之后，还剩两个未回答的问题：

1. 最优聚合器是不是 `GRU`，还是 `TCN`
2. 当前收益主要来自时序结构，还是来自 target weighting

因此本轮补了 3 条最有信息量的对照：

- `TCN + anchor + weighting`
- `GRU + anchor + unweighted`
- `GRU + anchor + lighter weighting`

### 2. `TCN + anchor + weighting`

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_tcn_anchor_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_tcn_anchor_weighted_v1)

关键结果：

- `epochs_ran = 14`
- `best_epoch = 8`
- `best_val_mae = 0.1681 mm`
- `test_mae = 0.1777 mm`
- `test_rmse = 0.2449 mm`
- `test_corr = 0.9112`
- `bias_mean = +0.0267 mm`
- `|err| > 0.3 mm = 85`

相对当前 best `GRU weighted K7`：

- MAE 略差：`0.1777 > 0.1731`
- RMSE 更好：`0.2449 < 0.2502`
- corr 更好：`0.9112 > 0.9072`
- 但 `|err| > 0.3 mm` 更多：`85 > 75`

分区间看：

- `low_<=0.2`：
  - `0.1367`，明显差于 `GRU weighted` 的 `0.1143`
- `mid_0.2_0.5`：
  - `0.1688`，也差于 `GRU weighted` 的 `0.1550`
- `high_>0.5`：
  - `0.2875`，优于 `GRU weighted` 的 `0.3353`

结论：

- **TCN 更像“高峰段更强、尾部更稳”的版本**
- 但它牺牲了低中幅值，因此没有拿到最优 MAE

### 3. `GRU + anchor + unweighted`

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_unweighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_unweighted_v1)

关键结果：

- `epochs_ran = 9`
- `best_epoch = 3`
- `best_val_mae = 0.1617 mm`
- `test_mae = 0.1819 mm`
- `test_rmse = 0.2955 mm`
- `test_corr = 0.8994`
- `bias_mean = -0.0317 mm`
- `|err| > 0.3 mm = 83`

和 `GRU weighted K7` 对比很清楚：

- low / mid 更好：
  - `low: 0.1074`
  - `mid: 0.1202`
- 但 `high_>0.5` 明显崩掉：
  - `0.4189`，远差于 weighted 的 `0.3353`

重点高峰帧也明显回退：

- `Q10_test1 frame 607`
  - weighted: `1.663`
  - unweighted: `1.433`
- `Q5_test3 frame 913`
  - weighted: `2.015`
  - unweighted: `1.356`

结论：

- **时序结构本身已经能提升很多**
- 但在当前目标上，target weighting 仍然是必要条件，不是可有可无的附属项

### 4. `GRU + anchor + lighter weighting`

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_light_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_light_v1)

权重配置：

- thresholds: `0.5, 1.0`
- weights: `1.0, 1.25, 2.0`

关键结果：

- `best_epoch = 3`
- `best_val_mae = 0.1568 mm`
- `test_mae = 0.1863 mm`
- `test_rmse = 0.2873 mm`
- `test_corr = 0.8958`
- `bias_mean = -0.0030 mm`
- `|err| > 0.3 mm = 83`

分区间结果：

- `low_<=0.2 = 0.1254`
- `mid_0.2_0.5 = 0.1230`
- `high_>0.5 = 0.3909`

这说明：

- 它确实把 `mid` 拉回来了
- 但同时把 `high` 又放掉了
- 最终总体 MAE 仍然输给 `1.0,1.5,3.0`

重点帧同样明显回退：

- `Q10_test1 frame 607`: `1.544`
- `Q10_test2 frame 735`: `1.013`
- `Q5_test3 frame 913`: `1.445`

结论：

- **更轻 weighting 不是更优折中点**
- 当前 moderate weighting 比 light weighting 更接近最优

### 5. 当前阶段收敛判断

截至这一轮，`K=7` 时序主线的关键候选可以归纳为：

- `GRU + anchor + weighting(1.0,1.5,3.0)`
  - `test_mae = 0.1731`
  - `rmse = 0.2502`
  - `corr = 0.9072`
  - `|err| > 0.3 mm = 75`
- `TCN + anchor + weighting(1.0,1.5,3.0)`
  - `test_mae = 0.1777`
  - `rmse = 0.2449`
  - `corr = 0.9112`
  - `|err| > 0.3 mm = 85`
- `GRU + anchor + unweighted`
  - `test_mae = 0.1819`
- `GRU + anchor + light weighting`
  - `test_mae = 0.1863`

因此当前可以把阶段结论收成：

- **若主目标是最低 held-out test MAE，当前收敛版就是：**
  - [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1)
  - 配置：`raw ROI + K=7 + GRU + current-frame anchor + weights 1.0/1.5/3.0`
- **若更看重 RMSE / 尾部稳定性，`TCN` 也值得保留为次优备选**

但从“继续找收敛版”的角度看，当前搜索空间已经明显收窄：

1. 不再回到 `K=5`
2. 不再去掉 weighting
3. 不再把 weighting 调得更轻
4. 当前最有资格作为阶段收敛版的是：
   - `K=7 GRU anchor weighted(1.0,1.5,3.0)`

### 6. 下一步建议更新

如果后续还要继续推进，不建议再在这组超参上做细碎扫描。

更合理的后续方向应转向：

1. 以当前收敛版为主模型，做：
   - 固定权重的重复训练稳定性验证
   - hard cases / worst segments 复核
2. 若业务更在乎大峰值保守性，可保留 `TCN` 作为对照参考
3. 若继续想压 `|err| > 0.3 mm` 的帧数，下一步更可能来自：
   - 标签一致性继续收紧
   - hard-example 定向处理
   - 而不是继续小步扫 `K/head/weights`

## 2026-04-08：current best 配置的 seed 稳定性验证

### 1. 本轮目的

在结构和权重基本收敛之后，最现实的问题变成了：

- 当前 best 是否只是单个 seed 的偶然好结果
- 还是这条配置本身就存在一段可稳定达到 `0.16x mm` 的 seed 窗口

因此本轮固定全部配置不动，只扫 seed：

- `raw ROI`
- `K = 7`
- `GRU`
- `current-frame anchor`
- target weighting `1.0 / 1.5 / 3.0`

### 2. `seed=3` 对照

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed3_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed3_v1)

结果：

- `best_epoch = 3`
- `best_val_mae = 0.1993 mm`
- `test_mae = 0.2174 mm`
- `test_rmse = 0.3238 mm`
- `test_corr = 0.9017`

结论：

- `seed=3` 不只是 test 差，val 也同步明显变差
- 说明这不是“val 选错 checkpoint”的问题
- 而是当前配置对 seed 确实有明显窗口效应

### 3. `seed=11` 对照

新 run：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1)

关键产物：

- [SL_visual_temporal_human_summary.json](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1/SL_visual_temporal_human_summary.json)
- [SL_visual_temporal_human_history_curves.png](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1/SL_visual_temporal_human_history_curves.png)
- [curves_visual_vs_human_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1/curves_visual_vs_human_only)
- [curves_slosh_vs_visual_only](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1/curves_slosh_vs_visual_only)

结果：

- `best_epoch = 13`
- `best_val_mae = 0.1537 mm`
- `test_mae = 0.1567 mm`
- `test_rmse = 0.2494 mm`
- `test_corr = 0.9364`
- `bias_mean = -0.0788 mm`
- `|err| > 0.3 mm = 71 / 554`

相对上一版 best `seed=7`：

- `test_mae: 0.1731 -> 0.1567`
- `rmse: 0.2502 -> 0.2494`
- `corr: 0.9072 -> 0.9364`
- `|err| > 0.3 mm: 75 -> 71`

这轮是一次实质性提升，不是小幅噪声波动。

### 4. 误差结构变化

相对 `seed=7`：

#### `low_<=0.2`

- `seed=7`: `0.1143`
- `seed=11`: `0.0742`

说明：

- 低幅值段显著改善
- 而且 `low` 区间 `|err| > 0.3 mm` 直接降到 `0`

#### `mid_0.2_0.5`

- `seed=7`: `0.1550`
- `seed=11`: `0.1454`

说明：

- 中间幅值也略有改善

#### `high_>0.5`

- `seed=7`: `0.3353`
- `seed=11`: `0.3724`

说明：

- 高峰段反而略差
- 但 overall MAE 仍然更好，说明这轮主要收益来自：
  - low / mid 大幅收紧
  - 同时没有让高峰段崩到不可接受

### 5. bag-wise 变化

#### `Q10_test1`

- `seed=7: MAE = 0.1447`
- `seed=11: 0.1384`

#### `Q10_test2`

- `seed=7: 0.1702`
- `seed=11: 0.1639`

#### `Q5_test3`

- `seed=7: 0.2002`
- `seed=11: 0.1684`

说明：

- 这轮真正的收益并不局限于单个 bag
- 3 个 held-out test bags 都有改善

### 6. 重点高峰帧

这里要保持诚实判断：

- `Q10_test1 frame 607`
  - `seed=7: 1.663`
  - `seed=11: 1.670`
- `Q10_test2 frame 735`
  - `seed=7: 1.678`
  - `seed=11: 1.378`
- `Q5_test3 frame 913`
  - `seed=7: 2.015`
  - `seed=11: 1.832`

说明：

- `seed=11` 不是每个 hardest high-peak frame 都更好
- 它赢在整体分布收紧，而不是赢在所有极端点

### 7. 当前阶段结论更新

seed sweep 之后，当前结论应更新为：

1. 当前 best 配置不是稳定到“任意 seed 都能到 0.16x”
2. 但它也不是纯粹撞大运，因为：
   - `seed=3` 差，且 val 也差
   - `seed=11` 好，且 val 也同步更好
3. 这说明：
   - 当前配置存在明确的好 seed 窗口
   - `val MAE` 对 seed 的优劣是有一定判别力的

截至当前，新的 current best 应更新为：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1)
- `test MAE = 0.1567 mm`

这是目前整条线里最接近“阶段收敛版”的 run。

### 8. 下一步建议更新

如果后续继续推进，我建议：

1. 以 `seed=11` 这版作为当前主 checkpoint
2. 暂停继续扫 `K/head/weights`
3. 后面若再做训练，优先考虑：
   - 少量 seed 复现验证
   - 或围绕 hardest frames / hardest segments 做定向分析

当前阶段不建议再做大范围无约束试验，因为：

- 配置空间已经明显收缩
- 新 best 已经推进到 `0.1567 mm`
- 继续收益大概率会变成边际递减

## 2026-04-08：阶段收口与交接口径

### 1. 阶段收敛版

截至当前，这一轮监督学习主线可以正式收口。

当前阶段收敛版更新为：

- [SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1](/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1)

核心配置：

- `raw ROI`
- `K = 7`
- `GRU temporal head`
- `current-frame anchor`
- target weighting `1.0 / 1.5 / 3.0`
- `seed = 11`

最终 held-out test 指标：

- `MAE = 0.1567 mm`
- `RMSE = 0.2494 mm`
- `Corr = 0.9364`
- `bias_mean = -0.0788 mm`
- `p95 abs error = 0.5562 mm`
- `max abs error = 1.7300 mm`
- `|err| > 0.3 mm = 71 / 554`

### 2. 对外/交接推荐评估口径

后续汇报或交接时，不建议只报 `MAE`。

推荐固定同时报告：

1. `overall MAE`
2. `overall RMSE`
3. `|err| > 0.3 mm` 的帧比例
4. `low / mid / high` 分层 MAE
   - `low <= 0.2 mm`
   - `mid 0.2 ~ 0.5 mm`
   - `high > 0.5 mm`
5. `p95 abs error`
6. `max abs error`
7. `bag-wise MAE / p95 / max / >0.3mm count`
8. 仅在 `target > 0.5 mm` 上统计的相对误差

原因：

- 单看 `MAE` 会把不同幅值区间混在一起
- 现在模型的主要残差仍然集中在高峰尾部
- 是否满足工程阈值，更直接取决于 `|err| > 0.3 mm` 的比例，而不只是均值

### 3. 当前 retained runs

本轮收口后，建议保留的 run 目录是：

- `SL_visual_human_0401_raw_roi_v1`
- `SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2`
- `SL_visual_human_peak_y_rect_0401_raw_roi_relabel_refresh_v2`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed11_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_tcn_anchor_weighted_v1`

保留理由：

- 覆盖历史 official best、最有价值的单帧 baseline、`peak_y_rect` 参考分支、当前 temporal 主线 best，以及 `TCN` 备选。

### 4. 当前删除的 dominated runs

本轮清理后，删除以下明显 dominated 或失败实验目录：

- `SL_visual_human_0401_raw_roi_relabel_refresh_v2`
- `SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v1`
- `SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_hires_v1`
- `SL_visual_human_0401_raw_roi_relabel_refresh_v2_weighted_v2_underpred_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_unweighted_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_k5_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_light_v1`
- `SL_visual_temporal_human_0401_raw_roi_relabel_refresh_v2_gru_anchor_weighted_seed3_v1`

删除理由：

- 它们都已经被更强 run 覆盖
- 继续保留对当前决策价值有限
- 反而会干扰后续交接和定位 current best

### 5. 当前阶段最终判断

如果目标是工程上先收敛一版并用于后续离线评估，这里可以停。

当前最合理的说法是：

- 已经找到一版明确优于旧 official best 和 `/slosh/height` 的主线模型
- 当前 best 已经进入 `0.15x mm` 级
- 继续优化当然还有空间，但大概率已经进入边际收益区

因此后续工作若继续推进，更建议围绕：

- hardest segments 复核
- 评估口径固化
- 控制器对比实验

而不是再做大范围无约束模型搜索。
