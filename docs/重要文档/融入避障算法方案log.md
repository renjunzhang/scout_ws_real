# 融入避障算法方案日志

## 2026-03-27

### 本轮动作

- 新增并初始化方案文档：
  - `/home/a/scout_ws/docs/重要文档/融入避障算法方案.md`
- 明确方案文档开头固定保留 `## 协作原则`
- 将后续避障相关记录统一收口到本文件

### 本轮补充

- 按“先把方案过完整，再动代码”的要求，补充了以下内容：
  - 当前方案还缺什么
  - 修改目的
  - 非目标
  - 修改后的 MPC 架构
  - 分阶段通过标准
  - 可能造成的问题
  - 文件夹安排
  - 修改后的总体目录理解
- 本轮没有开始搭代码骨架
- 本轮没有修改 `scout_local_planner` 控制代码

### 录包脚本补充

- 修改文件：
  - `/home/a/scout_ws/src/scout_apps/control/scout_local_planner/scripts/record_slosh_experiment.sh`
- 新增了后续避障实现/接口核查所需的最小录包话题：
  - `/scan_front`
  - `/map`
  - `/map_updates`
  - `/scout/mbf_costmap_nav/GlobalPlanner/plan`
  - `/scout/mbf_costmap_nav/global_costmap/costmap`
  - `/scout/mbf_costmap_nav/local_costmap/costmap`
  - `/scout/mbf_costmap_nav/local_costmap/costmap_updates`
  - `/scout/mbf_costmap_nav/local_costmap/footprint`
- 修改目的：
  - 避免后续再次出现“实物 bag 里没有 costmap/scan/global planner 关键话题，无法核实避障输入接口”的问题
- 边界：
  - 这一步只补录包范围
  - 没有开始接入避障控制逻辑

### 录包脚本第二轮补充

- 在上一步基础上，继续补入了“低带宽但后续可能高价值”的 MBF/costmap 相关话题：
  - `/scout/mbf_costmap_nav/GlobalPlanner/potential`
  - `/scout/mbf_costmap_nav/get_path/goal`
  - `/scout/mbf_costmap_nav/get_path/feedback`
  - `/scout/mbf_costmap_nav/get_path/result`
  - `/scout/mbf_costmap_nav/get_path/status`
  - `/scout/mbf_costmap_nav/global_costmap/costmap_updates`
  - `/scout/mbf_costmap_nav/global_costmap/footprint`
  - `/scout/mbf_costmap_nav/global_costmap/parameter_updates`
  - `/scout/mbf_costmap_nav/global_costmap/inflation_layer/parameter_updates`
  - `/scout/mbf_costmap_nav/local_costmap/parameter_updates`
  - `/scout/mbf_costmap_nav/local_costmap/inflation_layer/parameter_updates`
  - `/scout/mbf_costmap_nav/local_costmap/obstacle_layer/parameter_updates`
- 当前判断：
  - 这些话题相比图像和 costmap 主图本身，额外录制成本较低
  - 但对后续排查“路径生成、costmap 参数变化、膨胀层/障碍层语义”可能有明显帮助
- 当前仍然没有补入：
  - `parameter_descriptions`
- 原因：
  - 这类信息通常是静态描述，录制价值低于 `parameter_updates`

### 参考仓库

- 已拉取参考仓库到：
  - `/home/a/scout_ws/src/MPC-D-CBF`
- 当前本地提交：
  - `1b355cca60453af41913d35d84d8d869ca9bfde2`

### 本轮阅读结论

- 参考仓库的核心链路是：
  - `local_map`：点云/局部地图 -> 聚类 -> 椭圆障碍代理
  - `obs_param`：障碍参数跟踪与预测
  - `local_planner.py`：`casadi + ipopt` 下的 MPC + D-CBF 约束
- 当前项目的核心链路是：
  - `PathHandler -> MPCSolver -> OSQP`
  - 状态为 `Frenet + slosh` 增广状态
- 两者差异过大，不能直接整套移植

### 当前决策

1. 参考项目只借鉴“障碍代理 + 约束形式”，不直接迁移其控制框架。
2. v1 避障输入优先接当前已有的 `local_costmap`，不重建 `velodyne/grid_map` 管线。
3. v1 避障求解优先保持 `OSQP + 线性化约束`，不更换为 `casadi/ipopt`。
4. v1 先做静态/准静态障碍，动态障碍预测后置。

### 当前待办

1. 核清当前 `local_costmap` 真值话题、坐标系、更新频率和可用字段。
2. 核清当前 Frenet 误差到局部笛卡尔点的回投公式。
3. 设计 `ObstacleProxyManager` 和约束接入点。
