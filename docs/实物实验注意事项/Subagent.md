# Subagent 仿真实验调用准则

## 1. 适用范围

本文用于约束 Claude / 子 agent 执行 SPMPC 仿真实验、回放实验、离线分析和结果归档时的行为。

核心原则：

```text
子 agent 只能按明确实验卡执行，不自行改变实验口径；
所有仿真结果必须可复现、可追溯、可对比；
任何不确定项优先停下来问主 agent / 用户，而不是自行猜测。
```

## 2. 硬性红线

### 2.1 不动仿真环境，不删除代码

子 agent 不允许：

```text
1. 修改 Gazebo world、地图、模型、起点、终点、障碍物、定位初始化流程等仿真环境；
2. 删除代码、删除实验脚本、删除已有数据；
3. 为了让实验通过而临时改动 launch / yaml / 脚本中的环境参数；
4. 擅自切换 backend、variant、path、alpha_max、速度/加速度限制等实验变量；
5. 擅自 commit、push、reset、checkout、clean 工作树。
```

允许的范围：

```text
1. 按实验卡设置 planner / algorithm / config 参数；
2. 运行已有 launch、suite、analysis 脚本；
3. 创建新的实验输出目录、meta、日志、分析报告；
4. 如果必须改脚本才能记录诊断，先向主 agent / 用户说明必要性并等待确认。
```

### 2.2 文件归档必须集中、清晰、不可覆盖

实验产物优先放在 `/data/a` 下，并按日期、主题、run id 归档。推荐结构：

```text
/data/a/scout_spmpc_experiments/
  raw/
    YYYY-MM-DD/
      <batch_name>/
        <run_id>/
          <run_id>.bag
          <run_id>_meta.yaml
          <run_id>_planner.log
          <run_id>_analysis.md
          <run_id>_generated_path.json
  MANIFEST.md
```

要求：

```text
1. 不把大量 `spmpc*` 目录散落在 `/data/a` 根目录；
2. 不覆盖已有 run 目录，run id 必须带时间戳或唯一后缀；
3. 每个 batch 需要有简短说明：目的、变量、路径、backend、variant；
4. 失败 run 也要归档，不删除、不隐藏；
5. 若产生临时文件，应放在该 run 目录或 batch 目录下。
```

### 2.3 fixed-path 生成口径

除非实验卡另有说明，`stable_goal` / fixed-path 仿真口径固定为：

```text
1. 启动仿真后，以小车当前位姿作为路径起点；
2. 终点使用实验卡指定的固定终点；
3. 由“当前位置起点 + 固定终点”生成固定路径 JSON；
4. 正式 run / replay 使用该 generated path，不在同一批对比中擅自更换路径；
5. 生成的 path JSON 必须归档到 run 目录，并在 meta / analysis 中记录绝对路径。
```

子 agent 不应把“固定路径”误解为固定起点也固定；当前要求是“当前小车位置为起点，固定终点，形成固定路径”。

### 2.4 子 agent 启动命令

后续需要用户或主 agent 启动新的子 agent 会话时，默认命令为：

```bash
claude-gpt-pro
```

如需特定权限模式、工作目录或实验卡，应在启动后第一条 prompt 中明确写出，不让子 agent 自行推断。

### 2.5 仿真启动命令

当前仿真环境的默认启动命令为：

```bash
source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash

SIM_ENV=open USE_RVIZ=true \
SPAWN_X=3.30 SPAWN_Y=0.15 SPAWN_Z=0.1 SPAWN_YAW=-3.08 \
rosrun scout_local_planner launch_sim_nav_stack.sh
```

子 agent 使用该命令启动仿真时，仍必须遵守“启动后等待 30s、一次只跑一个 case、结束后关闭本次仿真并等待 30s”的时间纪律。若启动命令与实验卡冲突，应先报告，不要自行改命令。

运行相关脚本前，必须先阅读脚本说明：

```text
src/scout_apps/control/spmpc_local_planner/scripts/README.md
src/scout_apps/control/spmpc_experiments/scripts/README.md
```

### 2.6 fresh-sim 时间纪律

每个正式仿真 case 必须遵守：

```text
1. 一次只跑一个 case；
2. 启动仿真后等待 30s，让定位 / TF / Gazebo 状态稳定；
3. 从 planner 正式开始跟踪到终点超过 70s，判定该 case 失败；
4. case 结束后关闭本次仿真；
5. 关闭仿真后等待 30s，再开始下一次；
6. 不在同一个 Gazebo/RViz 状态里连续跑多个 formal case。
```

70s 超时必须记录为失败，而不是继续等到偶然到点。

## 3. 调用子 agent 前必须给出的实验卡

主 agent 调用子 agent 前，应明确给出一张实验卡。缺任一关键项时，子 agent 应先询问，不应自行推断。

实验卡至少包含：

```text
1. 实验目的：例如 replay gate / fresh-sim B0 / slosh monitor dry-run；
2. backend：例如 continuous_mpcc_acados / continuous_mpcc_direct_omega_legacy；
3. variant：B0 / B_slosh / B_ours；
4. path source：stable_goal / replay / 指定 PATH_FILE；
5. path file：若 replay，必须给出绝对路径；若 stable_goal，需说明“当前小车位置为起点 + 固定终点生成路径”；
6. fixed terminal goal：若 path_source=stable_goal，必须给出固定终点或引用已有配置；
7. 关键参数：alpha_max、v_max、omega_max、a_max、corridor/obstacle/homotopy flags；
8. run timeout：默认 planner 启动后 70s；
9. 输出根目录：默认 `/data/a/scout_spmpc_experiments/raw/YYYY-MM-DD/<batch_name>`；
10. 成功标准：GOAL_REACHED、progress_s、solver_time、path departure、warm_start_status 等；
11. 允许做的动作与禁止做的动作。
```

推荐实验卡模板：

```text
目的：
backend：
variant：
path_source：
path_file：
fixed_terminal_goal：
关键参数：
输出目录：
成功标准：
失败标准：
禁止事项：
备注：
```

## 4. 仿真前检查清单

子 agent 启动仿真前必须记录：

```text
1. 当前 git branch；
2. 当前 git commit hash；
3. 是否存在 uncommitted changes；
4. backend / variant / mode；
5. 关键 feature flags：slosh、obstacle、homotopy、corridor、corridor_hard_bound；
6. 关键 robot limits：v_max、omega_max、a_max、alpha_max；
7. path_file / generated_path；
8. 输出目录；
9. 是否为 fresh sim；
10. 仿真启动时间与 planner 启动时间。
```

如果发现实际启动日志与实验卡不一致，例如 backend 不对、variant 不对、feature flag 被意外打开，应立即停止并记录为配置失败，不继续跑。

## 5. 录包与日志要求

每个 run 至少应保存：

```text
1. rosbag；
2. planner stdout/stderr log；
3. launch 命令和环境变量；
4. meta.yaml 或 meta.json；
5. 离线分析结果；
6. 若生成路径，保存 generated_path.json；
7. 若启用 RGB / slosh monitor，保存相机/monitor 同步信息。
```

建议 bag 至少包含以下关键信息：

```text
/cmd_vel
/odom
/tf
/tf_static
/spmpc/status
/spmpc/solver_backend
/spmpc/debug/progress_s
/spmpc/debug/warm_start
/spmpc/debug/warm_start_status
/spmpc/slosh_height
/spmpc/debug/slosh_state
/spmpc/slosh_horizon_summary
/terminal/*
/reference/*
/profile_cap/*
/slosh/state
/slosh/debug
/slosh/height
```

如果某些 topic 不存在，应在分析报告中明确写出“未记录 / 不存在”，不要默认当作 0 或正常。

## 6. 成功 / 失败判据

### 6.1 基本成功标准

除非实验卡另有说明，fixed-path / replay 到点 gate 至少要求：

```text
1. GOAL_REACHED > 0；
2. progress_s > 0.95，正式 gate 建议 > 0.97；
3. terminal mode 进入 REACHED；
4. planner 启动后 70s 内完成；
5. solver_time_ms max < 33 ms；
6. cmd_v 不长期塌到接近 0；
7. 不出现明显 path departure；
8. warm_start_status 不是长期异常或靠 fallback 掩盖主路径失败。
```

### 6.2 失败标准

以下任一情况应判定失败：

```text
1. 70s 内未 GOAL_REACHED；
2. progress_s 长时间冻结；
3. path departure 明显扩大；
4. backend / variant / feature flags 与实验卡不一致；
5. 关键 topic 缺失导致无法判断成功性；
6. solver_time 超过实时预算；
7. planner 持续 WAITING / solve failed；
8. 仿真环境被意外改动或 fresh-sim SOP 被破坏。
```

失败 run 不删除，应保留 bag 和日志，并在分析报告中写清楚失败原因。

## 7. 安全与异常处理

子 agent 遇到以下情况必须停下来并报告，不应继续自动尝试：

```text
1. 需要修改仿真环境；
2. 需要删除或覆盖已有数据；
3. backend / variant 与预期不一致；
4. acados OCP bound 与 runtime 参数疑似不一致；
5. path departure 风险明显；
6. zero command / solve fail 安全路径不清楚；
7. rosbag 缺关键 topic；
8. 仿真进程无法干净关闭；
9. 超时后仍有进程残留且不是本次实验启动的进程。
```

进程清理原则：

```text
只关闭本次实验启动的仿真 / planner / rosbag / monitor 进程；
不要 kill 不确定来源的用户进程；
无法确认时先报告，不要强杀。
```

## 8. 分析报告要求

每个 run 结束后，子 agent 必须输出一个简短但完整的分析摘要，至少包括：

```text
1. verdict：SUCCESS / FAIL / CONFIG_FAIL / INCONCLUSIVE；
2. run directory；
3. bag path；
4. backend / variant / key params；
5. progress_s 起止值；
6. GOAL_REACHED count 和 first reached time；
7. terminal mode 统计；
8. solver status 统计；
9. solver_time_ms mean / max；
10. cmd_v / cmd_omega 关键统计；
11. odom displacement / cumulative distance；
12. path departure / projection distance 指标；
13. warm_start_status 统计；
14. slosh 指标，若适用；
15. 异常、缺失 topic、失败原因；
16. 下一步建议。
```

报告必须区分：

```text
1. 实验事实；
2. 离线分析推断；
3. 尚未验证的假设。
```

不能把 RouteB / diagnostic backend 的成功写成 alpha-state 主线成功。

## 9. 对比实验公平性要求

运行 B0 / B_slosh / B_ours 或 baseline 对比时：

```text
1. 只改变实验卡指定的算法变量；
2. path、仿真环境、初始等待、timeout、记录 topic 保持一致；
3. TEB / DWA / SPMPC 的速度、角速度、加速度限制要对齐；
4. 同一批对比中不要混用 fresh-sim 和非 fresh-sim 结果；
5. 不用失败 run 的局部片段冒充完整成功结果。
```

## 10. 子 agent 最终回复格式

子 agent 完成任务后，最终回复应使用以下格式：

```text
结论：SUCCESS / FAIL / CONFIG_FAIL / INCONCLUSIVE

执行内容：
- ...

关键结果：
- progress_s:
- GOAL_REACHED:
- solver_time_ms:
- status counts:
- path departure:

产物：
- run dir:
- bag:
- meta:
- analysis:

异常 / 注意事项：
- ...

建议下一步：
- ...
```

如果未完成，必须说明停在哪一步、为什么停、哪些文件已生成。

## 11. 当前额外注意事项（2026-06-09）

当前正式实物仍是 NO-GO。子 agent 不应把任何仿真实验解释为实物放行，除非主 agent / 用户明确更新 gate。

特别注意：

```text
1. `continuous_mpcc_acados + B0` alpha-state 同路径 replay 尚未到点；
2. warm-start 修正后 replay 仍出现 path departure + progress freeze；
3. RouteB direct-omega 成功只能作为 diagnostic / baseline，不能放行 alpha-state 主线；
4. `alpha_max_override=8.0` 是否真正进入 acados OCP bound 需要先确认；
5. 实物前必须有 path-departure abort 和 hard-zero 安全路径。
```
