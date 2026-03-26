## 2026-03-26 P1 路径与激励对比脚本落地

### 本次目标
- 实现 `scripts/compare_bag_paths_and_excitation_0325.py`
- 用统一口径比较 `0325` 的 `Q0_test1` 与 `Q5_test1/2/3`
- 在讨论“Q 参数抑制是否有效”前，先判定各 bag 是否同任务可比

### 代码变更
- 新增脚本：
  - `scripts/compare_bag_paths_and_excitation_0325.py`
- 脚本职责：
  - 读取 `/odom`、`/cmd_vel`、`/local_path`、`/scout/global_path_smooth`
  - 读取 `/slosh/height`、`/slosh/ax_est`、`/slosh/ay_est`
  - 读取 `verify/0325/*/mpc_realsense_aligned_v2.csv`
  - 生成单 bag 指标、pairwise 可比性判断、轨迹图、激励图、README

### 运行中发现并修复的问题
- 首轮烟雾运行失败，根因是 `zero_align_offset()` 调用了 `statistics.median()`，但脚本缺少 `import statistics`
- 已补上导入后重新执行：
  - `python3 -m py_compile scripts/compare_bag_paths_and_excitation_0325.py`
  - `python3 scripts/compare_bag_paths_and_excitation_0325.py --out-dir /tmp/path_compare_smoke`
- 烟雾验证通过，确认脚本能完整生成：
  - `bag_metrics.csv`
  - `pairwise_compare.csv`
  - `odom_overlay.png`
  - `cmd_vel_compare.png`
  - `excitation_compare.png`
  - `slosh_vs_excitation_compare.png`
  - `README.md`

### 正式产物
- 目录：
  - `/data/a/realsense_validation_v2/verify/0325/path_compare`
- 关键文件：
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/bag_metrics.csv`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/pairwise_compare.csv`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/README.md`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/odom_overlay.png`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/cmd_vel_compare.png`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/excitation_compare.png`
  - `/data/a/realsense_validation_v2/verify/0325/path_compare/slosh_vs_excitation_compare.png`

### 当前结论
- `Q0_test1` 对 `Q5_test1/2/3` 的 `comparable_overall` 全部为 `0`
- 当前这 4 个运动 bag 之间，差异不只来自液体响应，还包括：
  - 轨迹形状差异
  - 任务时长差异
  - 路程差异
  - `speed/omega/ay` 激励量级差异
- 因此现阶段不能直接把 `Q0/Q5` 的 `/slosh/height` 或 RealSense 差异解释成“抑制是否有效”

### 对比中最重要的数字
- `Q0_test1 vs Q5_test1`
  - `duration_ratio = 1.905`
  - `path_shape_mean_distance_m = 2.660`
  - `speed_p90_ratio = 2.424`
  - `ay_p90_ratio = 1.984`
- `Q0_test1 vs Q5_test2`
  - `duration_ratio = 1.148`
  - `path_shape_mean_distance_m = 1.197`
  - `path_length_ratio = 1.755`
- `Q0_test1 vs Q5_test3`
  - `duration_ratio = 1.063`
  - `path_shape_mean_distance_m = 4.664`
  - `omega_p90_ratio = 2.062`
  - `ay_p90_ratio = 1.444`

### 对后续决策的影响
- 现在不应该继续围绕这 4 个 bag 直接讨论“Q=5 是否一定比 Q=0 抑制更好”
- 更短路径是：
  - 先设计更严格的同任务录制约束
  - 或者从已有 bag 里筛一对真正可比的 `Q0/Q5`
  - 再比较 `/slosh/height` 和 RealSense 主液面
