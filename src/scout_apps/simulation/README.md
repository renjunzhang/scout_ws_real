# 仿真源码域（SIM-R8）

这里是**独立于实物源码**的 S-MPCC 仿真实现。它的目标是让仿真控制器、液体模型、ROS
运行图、构建产物和实验工具均不影响
`src/scout_apps/control/` 中的实物控制逻辑。

## 边界

| 项目 | 仿真侧 | 实物侧（不会被仿真使用或改写） |
| --- | --- | --- |
| 控制器 | [`spmpc_sim_local_planner`](spmpc_sim_local_planner/README.md)，节点 `/sim_spmpc_local_planner` | `control/spmpc_local_planner`，节点 `/spmpc_local_planner` |
| 液体相关实现 | [`scout_liquid_plant`](scout_liquid_plant/)，仿真自有 H_proxy/modal/plant 工具 | `control/slosh_models` |
| 离线物理液体评价 | [`scout_dualsphysics_liquid`](scout_dualsphysics_liquid/README.md)，closed-bag 后处理，当前仅 P0 依赖门禁 | 不接入实物控制源码，也不反馈 Gazebo/控制器 |
| 构建与 devel | `/data/a/scout_sim_replacement/r8_controller_ws/{build,devel}` | `/home/a/scout_ws/build`、`/home/a/scout_ws/devel` |
| ROS 诊断 | `/sim_spmpc/*` | `/spmpc/*` |

仿真包是有 provenance 的源码 fork，不是 include-path overlay、软链接或实物库的
wrapper。以后实物控制器更新也**不会自动**进入仿真；若需要同步，必须执行显式
copy-and-review，并重新冻结仿真源码 hash。

## 入口

先构建仿真控制器（该脚本用干净环境并只 whitelist 此仿真包）：

```bash
src/scout_apps/simulation/spmpc_sim_local_planner/scripts/build_sim_controller_workspace.sh
```

环境、H0 smoke、冻结路径 replay、formal gate 和矩阵 runner 的入口均在
[`spmpc_sim_local_planner/scripts/`](spmpc_sim_local_planner/scripts/)；运行前请阅读
该包 README 以及相应 release/freeze 文档。环境启动器只管理它跟踪的 PID，不使用
`killall` 或 `pkill`。

## 仿真环境启动指南（R8 独立入口）

本节参考
[`仿真对比试验启动指南`](../../../docs/实物实验注意事项/对比试验/仿真对比试验/仿真对比试验启动指南.md)
的场地、终点、fresh-case、30 s settle、60 s trajectory timeout 和录包原则；但是
**不要**把该旧指南中的下列共享源码入口用于 R8：

- `/data/a/scout_sim_replacement/scripts/run_strict_fresh_fair_comparison_n3.sh`；
- `/data/a/scout_sim_replacement/scripts/run_strict_fresh_profile_baselines_n1.sh`；
- `src/scout_apps/control/...` 下的 controller、`spmpc_experiments` 或
  `slosh_models`。

它们属于历史 shared-source 运行面，不能满足本目录的源码隔离承诺。R8 只使用本包
的 adapter、runner 和外部仿真 build prefix。

### 启动前检查

在 `/home/a/scout_ws` 执行：

```bash
src/scout_apps/simulation/spmpc_sim_local_planner/scripts/build_sim_controller_workspace.sh
python3 src/scout_apps/simulation/spmpc_sim_local_planner/scripts/smpcc_sim_h0_runtime_adapter.py self-check
```

构建脚本会以干净环境只构建 `spmpc_sim_local_planner` 到
`/data/a/scout_sim_replacement/r8_controller_ws`。它不会写入主工作区的
`build/` 或 `devel/`。启动前不要已有 ROS/Gazebo case；每次 H0 都必须使用两个
不同且空闲的 `127.0.0.1` 端口，且不与其他仿真并发。

固定的 H0 development 场景为开阔场地 P2：出生点 `(-4, 0)`，终点 `(5, 0, 0)`，
`/odom` frame 下的 s-curve（amplitude ratio `0.18`、范围 `0.25–1.20 m`、left、
3 次 smooth）；地图为
`/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream`。
它不是 H1/L1 frozen JSON replay。

### 无 Gazebo 的 dry-run

这一步只验证 H0 生命周期与账本，不启动 ROS/Gazebo，也不产生正式数据：

```bash
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/data/a/scout_sim_replacement/results/r8_h0_dry_run_${STAMP}"
python3 src/scout_apps/simulation/spmpc_sim_local_planner/scripts/smpcc_sim_toolchain.py \
  h0-smoke --output-root "$OUT" \
  --ros-master-uri 127.0.0.1:11890 \
  --gazebo-master-uri 127.0.0.1:11924
```

### H0 live smoke（headless 或可视化）

以下是唯一推荐的单 case 入口。它由 runner 顺序完成 fresh ROS/Gazebo、30 s
settle、recorder 就绪后运动、最多 60 s 等待精确 `GOAL_REACHED`、frozen tail 和
case-local postflight；无需、也不应手动运行 environment launcher 或手动清理进程。

```bash
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/data/a/scout_sim_replacement/results/r8_h0_live_${STAMP}"
ADAPTER=src/scout_apps/simulation/spmpc_sim_local_planner/scripts/smpcc_sim_h0_runtime_adapter.py
RUNNER=src/scout_apps/simulation/spmpc_sim_local_planner/scripts/smpcc_sim_toolchain.py

# 两个端口必须空闲、互不相同；若占用，整体换为另一对 loopback 端口。
ROS_PORT=11890
GAZEBO_PORT=11924

# 默认 headless；需要查看 Gazebo + tracking RViz 时追加 --visualize。
python3 "$ADAPTER" prepare \
  --output-root "$OUT" \
  --spec-output "$OUT/h0_live_spec.json" \
  --ros-master-uri "127.0.0.1:${ROS_PORT}" \
  --gazebo-master-uri "127.0.0.1:${GAZEBO_PORT}" \
  --visualize

# 外层只是防止基础设施挂死；它涵盖启动/settle/tail/postflight，不能替代轨迹的 60 s hard cap。
timeout --preserve-status --kill-after=60s 10m \
  python3 "$RUNNER" run \
    --row "$OUT/h0_runtime_assets/h0_row.json" \
    --spec "$OUT/h0_live_spec.json" \
    --output-root "$OUT"
```

不需要可视化时，删去 `prepare` 命令中的 `--visualize`。可视化窗口由 adapter 自己
追踪和关闭；不要额外执行 `rviz`、`gzclient`、`pkill` 或 `killall`。若可视化模式下
没有图形桌面可用，改用 headless，而不是改动 launch 文件。

成功 case 的数据在
`$OUT/SIM-DEV-H0_SMOKE/b01/p01_Bsmooth/r01/`：检查
`attempt_manifest.json`、`postflight.json`、closed bag、effective-config readback 与
`$OUT/dataset_index.jsonl`。只有 `GOAL_REACHED` 且 postflight PASS 才是成功的
development attempt；60 s 内未到达是 method failure，不能以 retry 覆盖。

此入口固定写入 `formal: false` 与 `DEVELOPMENT_SMOKE_NOT_FORMAL`。它仅用于环境、
可视化和 H0 开发验证，绝不计入 40/64/88。正式 H1/L1/C1/C2/FrozenProfile 矩阵只能
在 immutable R8 freeze/master、随机表、source GO receipt、timing admission 及
plant/firewall 条件齐备后，通过 hash-bound formal adapter 启动；缺任一项会
fail-closed。

## 信号语义

- `/slosh/height` 是 `H_proxy`；
- `/sim_spmpc/slosh_height` 是 `H_modal`；
- 二者都不是独立 liquid plant truth，也不能写入 physical-primary 字段。

只有独立 plant、controller subscriber firewall 和保真验证通过后，仿真才可能给出
仿真 primary 结论；它仍不能替代实物 primary 结论。

## 离线物理液体接入状态

`scout_dualsphysics_liquid` 与 ROS/catkin/Gazebo launch graph 完全分离。它只允许在现有
SIM-R8 case 已完成、bag 已关闭且 postflight/manifest 已固定之后，离线回放 executed
`/odom`。当前实现仅包含固定根安全检查、目录准备、DualSPHysics 固定提交获取和静态
清单审计；不包含 case、solver runner 或高度提取器，也不编译或执行任何上游文件。

外部数据只允许写入 `/data/a/scout_sim_replacement/r8_liquid`。详细 two-pass 设计、
空间水位和 NO-GO 边界见
[`20260805_DualSPHysics物理液体接入SIM-R8方案.md`](../../../docs/实物实验注意事项/对比试验/仿真接入液体/20260805_DualSPHysics物理液体接入SIM-R8方案.md)。

## 当前实验状态

已完成的是 source-separated R8 的 H0 development smoke，明确 `formal: false`，不
计入也不替代正式 40/64/88。正式矩阵仍 fail-closed，至少需要新的 immutable R8
freeze/master/randomization、source GO receipt、timing admission、冻结 H1/L1/C1/C2
与 FixedProfile；若要提出 primary 结论，还必须具备独立 plant/fidelity/firewall
证据。

详细隔离与 H0 记录见：
[`docs/实物实验注意事项/对比试验/仿真对比试验分析/20260804_SIM-R8源码隔离迁移与R7历史边界.md`](../../../docs/实物实验注意事项/对比试验/仿真对比试验分析/20260804_SIM-R8源码隔离迁移与R7历史边界.md)。
