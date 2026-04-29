# 2026-04-29 Anti-Slosh diff 整理与提交方案

更新时间：2026-04-29

## 1. 当前状态

工作区当前不是一个干净的单主题 diff。

当前修改混合了：

```text
1. 可保留的离线分析/录包基建
2. 默认关闭的 anti-slosh 实验入口
3. 已验证失败或暂不作为主线的控制机制
4. 实验日志和阶段方案
5. 无关 RViz / 协作规则 / gitignore 漂移
```

因此不建议直接提交为“Anti-Slosh MPC 改进”。

## 2. 当前技术结论

必须在提交信息中明确：

```text
这批代码不是最终有效控制器。
OUTPUT_GUARD 在 P2_s_curve 有效，但 P3_mixed 主段不泛化。
当前可提交价值主要是实验基建、诊断工具和默认关闭的消融入口。
```

已停止作为主线的方向：

- `Q_slosh / Q_slosh_eta_dot / terminal_factor_slosh_*`
- `Q_ay_pred`
- `Q_modal_energy`
- `GOV_AY`
- `PROFILE_SAFE / PROFILE_SELECTIVE / PROFILE_WINDOW / PROFILE_RISK`
- 固定阈值 `OUTPUT_GUARD`

## 3. 建议拆分

### Commit A：离线分析与录包可追溯基建

目的：

```text
保留最有复用价值的资产。
即使控制策略失败，这些工具仍然应该进入仓库。
```

建议包含：

- `src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py`
- `src/scout_apps/control/scout_local_planner/scripts/analyze_slosh_peak_precursors.py`
- `src/scout_apps/control/scout_local_planner/CMakeLists.txt`
- `src/scout_apps/control/scout_local_planner/scripts/README.md`

可包含 `run_sim_fixed_path_bag.sh` 中以下内容：

- `${BAG_PATH}.txt` 参数记录
- `/experiment/config_summary`
- 录制新增诊断 topic
- `PATH_MODE=template_goal`
- `PATH_ID / CONDITION / RUN_ID` 实验命名和配置摘要

风险：

- `run_sim_fixed_path_bag.sh` 里也混有大量具体实验 condition。
- 如果要保持 Commit A 干净，需要用 `git add -p` 只暂存录包追溯相关 hunk。

建议 commit message：

```text
tools: 增加 anti-slosh bag 可追溯记录与峰前诊断
```

### Commit B：默认关闭的 anti-slosh 消融入口

目的：

```text
保留已实现但默认关闭的消融入口，方便未来复盘和对照。
不宣称这些机制有效。
```

建议包含：

- `include/scout_local_planner/types.h`
- `src/cost_function.cpp`
- `include/scout_local_planner/local_planner_ros.h`
- `src/local_planner_ros.cpp`
- `src/path_handler.cpp`
- `config/mpc_params.yaml`
- `config/mpc_params_sim.yaml`
- `launch/slosh_experiment.launch`
- `launch/slosh_experiment_sim.launch`
- `run_sim_fixed_path_bag.sh` 中各 `CONDITION=*` 入口

包含的机制：

- `Q_ay_pred`
- `Q_modal_energy`
- `GOV_AY`
- `PROFILE_*`
- `OUTPUT_GUARD`
- `last_control_.omega` 同步实际发布角速度

必须强调：

```text
所有新机制默认关闭。
提交的是消融入口和诊断通道，不是最终主线控制策略。
```

风险：

- `mpc_params_sim.yaml` 把 `R_domega` 从 `1.0` 改成 `4.0`，这不是默认关闭入口，会改变仿真默认行为。
- 如果要保持低风险，应单独确认是否保留该参数改动；否则从 Commit B 中排除或单独成 Commit E。

建议 commit message：

```text
control: 增加默认关闭的 anti-slosh 消融入口
```

### Commit C：实验结论文档

目的：

```text
把“为什么不继续扫参数”的证据链留下。
```

建议包含：

- `docs/Claude/修改方案-时间-简介/2026-04-27_Anti-Slosh_方向修正.md`
- `docs/Claude/修改方案-时间-简介/2026-04-28_Anti-Slosh_阶段结论与后续方案.md`
- `docs/Claude/修改方案-时间-简介/2026-04-29_Anti-Slosh_diff整理与提交方案.md`
- `docs/Claude/修改日志-时间/2026-04-27.md`
- `docs/Claude/修改日志-时间/2026-04-28.md`
- `docs/Claude/修改日志-时间/2026-04-29.md`
- `docs/Claude/对话交接文档-时间-简介/2026-04-27_Anti-Slosh_MPC改进过程bag清单.md`
- `src/scout_apps/control/scout_local_planner/README.md`

注意：

- `docs/Claude/修改日志-时间/2026-04-24.md` 当前追加了大量 04-26/04-27 内容，时间线混乱。
- 更稳的做法是把 04-26/04-27 的新增内容迁移到对应日期日志，或者保留但在 commit message 中说明“历史日志补记”。

建议 commit message：

```text
docs: 记录 anti-slosh MPC 消融结论与后续方向
```

### Commit D：协作规则迁移

当前状态：

```text
docs/Claude/AGENT规则.md 被删除
docs/Claude/CLAUDE.md 已存在且已跟踪
```

建议：

- 不要混入 MPC 提交。
- 单独确认是否删除 `AGENT规则.md`。
- 如果 `CLAUDE.md` 已经完全替代旧文件，可单独提交删除。

建议 commit message：

```text
docs: 迁移协作规则到 CLAUDE.md
```

### Commit E：仿真参数默认值变更

当前风险项：

```text
config/mpc_params_sim.yaml:
R_domega: 1.0 -> 4.0
```

这会改变仿真默认行为，不是“默认关闭实验入口”。

建议：

- 单独确认是否保留。
- 如果保留，单独提交，说明“仿真参数对齐实物”。
- 如果不保留，后续应只回退这一行，不影响其他实验入口。

建议 commit message：

```text
sim: 对齐仿真 MPC 角速度变化权重
```

## 4. 本批不建议提交的内容

### RViz 文件

文件：

- `src/scout_apps/sensors/nanoscan3_localization/rviz/cartographer_localization.rviz`

原因：

- 与 anti-slosh 主线无关。
- 包含窗口位置、视角、RobotModel 展开状态等 GUI 漂移。

建议：

```text
保持 unstaged，不进本批 commit。
```

### .gitignore

文件：

- `.gitignore`

内容：

```text
.tmp_single_frame_debug_0424
```

判断：

- 可以保留，但与当前 MPC 主线无关。
- 若提交，应单独放到文档/工具清理 commit，不应混入控制器改动。

### AGENT规则.md 删除

文件：

- `docs/Claude/AGENT规则.md`

判断：

- 属于协作规则迁移，不属于 MPC。
- 单独确认后再提交。

## 5. 提交前必须验证

最低验证：

```bash
python3 -m py_compile \
  src/scout_apps/control/scout_local_planner/scripts/extract_slosh_metrics.py \
  src/scout_apps/control/scout_local_planner/scripts/analyze_slosh_peak_precursors.py

bash -n src/scout_apps/control/scout_local_planner/scripts/run_sim_fixed_path_bag.sh

python3 -m xml.etree.ElementTree \
  src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch

python3 -m xml.etree.ElementTree \
  src/scout_apps/control/scout_local_planner/launch/slosh_experiment_sim.launch

git diff --check
```

若要提交控制器代码，还必须跑：

```bash
catkin_make --pkg scout_local_planner
```

## 6. 推荐下一步操作

当前建议：

```text
先不提交。
先按上面的 A/B/C/D/E 分组用 git add -p 拆分。
RViz 保持 unstaged。
R_domega=4.0 单独确认。
AGENT规则.md 删除单独确认。
```

如果要最小化风险，可以只提交：

```text
Commit A：分析/录包基建
Commit C：文档结论
```

暂缓提交：

```text
Commit B：实验性控制器入口
Commit E：仿真默认参数变更
```

理由：

- 当前控制策略尚未形成 P3 泛化成功结论。
- 但分析基建和结论文档已经有复用价值。
