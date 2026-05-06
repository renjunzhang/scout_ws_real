# 2026-05-06 anti_slosh_path_post_processor 增加 costmap collision check

## 1. 触发原因

下一步要把 GeoRef A/B 从 open field 单目标搬到 maze 同起点同终点。当前 `anti_slosh_path_post_processor.py` 没有 costmap collision check：

```text
README §9.6:
  当前 post-processor 没有 costmap / footprint collision check
  maze 只能 smoke test，不能作为安全结论
```

raw global path 由 `mbf_costmap_nav` 出，自带 costmap 安全裕度；GeoRef 通过几何平滑生成，max_drift 实测 0.10–0.16m。在 maze 通道内这一漂移叠加 footprint 半径会切墙，A/B 会因为撞墙失败而非防晃失败，无法作为论文证据。

## 2. 目标与验收

目标：
```text
candidate 切墙时被 reject，selected 永远是 collision-free 的；
raw global path 不应被 self-reject（mbf 出的路径本身在 costmap 安全区）。
```

验收：
```text
1. py_compile 通过。
2. 同一原始 path 下，启动节点不订阅 costmap 时，候选行为与当前一致（向后兼容）。
3. 启动节点订阅 maze costmap 时：
   - candidate_report 中 mild/medium/strong 若切墙，reason 含 collision:idx=k:cost=N。
   - selected 候选不含切墙点。
4. 不影响 open field 已通过的同目标三包结果（在 collision-free 路径上 collision check 不应改变选择）。
```

## 3. 设计

### 3.1 利用 inflation 替代 footprint polygon

仿真 `global_planner_sim.yaml`：
```text
inflation_layer:
  inflation_radius = 0.5
  cost_scaling_factor = 3.0
static_layer:
  lethal_cost_threshold = 65
```

inflation 已经把 robot footprint 的安全距离编码进 cost 值。点查询 + 单一阈值即可：

```text
对候选 path 上每个采样点 (x, y):
  ix = floor((x - origin.x) / resolution)
  iy = floor((y - origin.y) / resolution)
  cost = data[iy * width + ix]
  若 cost < 0 (unknown) 或 cost >= collision_threshold: reject
```

不做 footprint polygon、不做时间维度、不做 costmap update 增量订阅。这与当前已经有 inflation_radius=0.5 的事实一致，简单且足够。

### 3.2 阈值选择

```text
collision_threshold = 50 (默认)
unknown_is_obstacle = true (默认)
```

理由：
```text
inflated_obstacle ≈ 99
inscribed_inflated_obstacle = 99 (footprint 必撞)
cost = 50 大致对应 footprint 边缘到障碍的剩余 inflation 余量
低于该阈值才视为安全可走。
```

阈值通过 launch 参数暴露，maze 严格场景可调到 80 放宽，open field 验证可保持默认。

### 3.3 ROS 接口变更

新增订阅：
```text
~costmap_topic  default=/scout/mbf_costmap_nav/global_costmap/costmap
                nav_msgs/OccupancyGrid (latch / 5Hz update)
```

新增参数：
```text
~gates/collision_threshold  default=50  (0-100 int)
~gates/unknown_is_obstacle  default=true
```

`~gates/enable_collision_check` 已存在，从 placeholder 升级为实际生效。

candidate_report 新增字段：
```text
col=<accepted|idx:N:cost:V|no_costmap>
```

reject_reason 增加：
```text
collision:idx=k:cost=N
no_costmap (当 enable_collision_check=true 但 costmap 未收到时)
```

### 3.4 gate 顺序

按代价升序，便于在 candidate_report 看出第一个失败原因：
```text
1. too_few_points
2. min_seg
3. drift
4. length (upper / lower)
5. level (max_candidate_level)
6. endpoint
7. direction
8. collision   <-- 新增，最贵放最后
```

### 3.5 frame 假设

```text
candidate path frame == map (raw global_path 来自 mbf，frame=map)
costmap frame == map (global_planner_sim.yaml: global_frame: map)
```

不做 TF 变换。如果未来 frame 不一致，再加 transform，不为不存在的问题写代码 (CLAUDE.md §2)。

### 3.6 启动期 graceful 行为

```text
enable_collision_check = true 且 self.latest_costmap is None:
  - 所有非 original 候选 reject (reason="no_costmap")
  - original 不做 collision check (raw 来自 mbf，已安全)
  - selected = original
  - logwarn_throttle 提示等待 costmap
```

避免节点启动顺序问题导致 GeoRef 切墙。

### 3.7 范围之外（不做）

```text
1. 不做 footprint polygon check (inflation 已覆盖)
2. 不做 update topic 增量订阅 (静态 maze 不需要)
3. 不做 TF 变换 (frame 一致)
4. 不做 raw path 自检 (假定 mbf 输出安全)
5. 不重写 candidate generation (外科手术式修改)
```

## 4. 改动清单

```text
src/scout_apps/control/scout_local_planner/scripts/anti_slosh_path_post_processor.py
  - 新增 OccupancyGrid 订阅与缓存
  - evaluate_candidate 增加 collision sub-check
  - publish_candidate_report 增加 col 字段

src/scout_apps/control/scout_local_planner/launch/anti_slosh_path_post_processor.launch
  - 新增 args: costmap_topic / collision_threshold / unknown_is_obstacle
  - enable_collision_check 默认值保持 false (open field 用例不影响)

docs/Claude/修改日志-时间/2026-05-06.md
  - 追加本节实现记录与 py_compile 结果

src/scout_apps/control/scout_local_planner/README-NEW.md
  - §9.2 新增 costmap_topic 订阅
  - §9.6 移除 "no collision check" 边界，改为说明阈值由 inflation 承担
```

## 5. 验证步骤

```text
1. py_compile：
   python3 -m py_compile src/scout_apps/control/scout_local_planner/scripts/anti_slosh_path_post_processor.py

2. 静态检查：
   - 默认 enable_collision_check=false 时行为与之前完全一致
   - candidate_report 仍能解析

3. 仿真 smoke test (用户执行)：
   maze 启动 mbf + post-processor + RViz
   订阅 /anti_slosh_path/debug/mild 与 /anti_slosh_path/debug/strong
   预期 strong/medium 在窄通道被 reject，selected = mild 或 original
```

## 6. 与论文叙事的关系

```text
collision check 加完之后才能写：
  "GeoRef is safety-aware: candidate paths that violate the global costmap
   inflation are rejected before MPC tracking, so the proposed reference
   layer remains within the same safety envelope as the baseline planner."

这同时回应 reference-layer 路线的一个常见审稿问题：
  reference 优化是不是绕过了避障？
  答：候选受 costmap 约束，与 baseline 共享同一安全裕度。
```
