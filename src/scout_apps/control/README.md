# control — 局部规划与控制

Scout Mini 液体运输 anti-slosh 研究的控制层。围绕"两条并行主线 + 一套共享液体物理核 + 一组对比 baseline"组织。

> 详细设计/实验流程见各包内文档与 `docs/`，本 README 只做目录级索引。

---

## 两条主线

两条线**同一抑晃思想**(二阶模态 + slosh 代价)、**不同系统定位**，互不取代、可独立启动。

### Route A — `scout_local_planner`(控制层 SloshPriorityMPC)
- **定位**：纯控制层。Frenet MPC 跟踪固定参考路径，**不改 v_ref**，在代价里加 slosh 项抑晃。
- 状态 `[e_l, e_c, e_θ, v]` (+4 维晃动增广)，控制 `[a, ω]`，OSQP 求解。
- baseline 同框架切 `experiment_group`(C/D/E/F/RPP/BIAGIOTTI/TOPPRA/RUCKIG)。
- 入口：`test_mpc.launch`(实物) / `test_mpc_sim.launch`(仿真) / `slosh_experiment.launch`(抑晃实验)。

### Route B — `spmpc_local_planner`(规控一体 MPC，当前主攻)
- **定位**：规划+控制一体。自己决定速度/进度,带避障(corridor/costmap)。
- **双后端**(运行时 `solver_backend` 切换)：
  - `primitive`：运动基元库择优(确定性、可回退、ablation)。
  - `continuous_mpcc_acados`：连续 MPCC,acados 求解,9 维联合状态 `[px,py,ψ,v,s,η_x,η̇_x,η_y,η̇_y]`,控制 `[a,ω,v_s]`。
- 三层解耦：`core/`(无 ROS) / `ros/`(adapter) / `config/`(YAML 平台/容器/实验)。
- 入口：`spmpc_fixed_path.launch`(`planner_variant:=B0|B_slosh|B_smooth|B_ours`,`solver_backend:=...`)。
- acados 求解器由 `spmpc_local_planner/tools/codegen/acados/generate_spmpc_acados.py` 生成（见安装文档）。

---

## 共享与配套

| 包 | 角色 |
|---|---|
| `slosh_models` | **唯一液体物理核**(`LiquidSloshModel`)：二阶模态 ω_n/ζ/κ、离散 A_d/B_d、高度系数。Route A/B 共用,保证抑晃口径一致。 |
| `spmpc_experiments` | SPMPC 与外部 baseline 的实验 launch / 录包工具。 |
| `baseline_local_planner_runner` | 把 nav_core 插件(TEB/DWA/mpc_local_planner)脱离 move_base 独立跑,作规控一体 baseline。 |

## 对比 baseline / 第三方(vendored)

| 包 | 说明 |
|---|---|
| `teb_local_planner` | TEB 外部 baseline(第三方)。 |
| `mpc_local_planner` | 积分式 MPC 外部 baseline(第三方)。 |
| `control_box_rst` | TEB/mpc_local_planner 的依赖库(第三方)。 |
| `navigation` | ROS navigation 栈(amcl/costmap_2d/... 第三方,供定位/代价地图)。 |

## 其他自研

| 包 | 说明 |
|---|---|
| `scout_omni_local_planner` | 全向底盘局部规划器变体。 |

---

## 快速上手

```bash
cd ~/scout_ws && catkin_make && source devel/setup.bash
```

**Route A 跟踪**(需全局路径 `/scout/global_path`)：
```bash
roslaunch scout_local_planner test_mpc.launch        # 实物
roslaunch scout_local_planner test_mpc_sim.launch    # 仿真
```

**Route B 规控一体**(固定路径)：
```bash
# primitive 后端(无需 acados):
roslaunch spmpc_local_planner spmpc_fixed_path.launch planner_variant:=B0
# continuous_mpcc_acados 后端(需先装 acados + codegen, 见安装文档):
source ~/.bashrc   # 带 ACADOS_SOURCE_DIR / LD_LIBRARY_PATH
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh solver_backend:=continuous_mpcc_acados
```

> continuous 后端没装 acados 时退化为 stub(`/spmpc/status=ACADOS_NOT_IMPLEMENTED`),整包仍可编译。

---

## 关键文档

**Route B(spmpc)**
- 连续 MPCC 升级与模块化方案：`docs/Claude/修改方案-时间-简介/2026-06-03_SPMPC连续MPCC优化升级与模块化方案.md`
- 规控一体新主线方案：`docs/Claude/修改方案-时间-简介/2026-06-01_spmpc_local_planner规控一体新主线方案.md`
- Methods(论文叙事)：`docs/重要文档/Methods/20260602_SPMPC规控一体Methods论文叙事版.md`
- 实物对比实验 SOP：`docs/实物实验注意事项/对比试验/20260603_SPMPC连续MPCC实物对比实验SOP.md`
- acados 安装(实物端,focal 坑)：`docs/实物实验注意事项/代码移植/20260602_实物端代码拉取与子模块注意事项.md` §3.1

**Route A(scout_local_planner)**
- launch 说明：`scout_local_planner/launch/README.md`
- 参数：`scout_local_planner/config/mpc_params.yaml`(实物) / `mpc_params_sim.yaml`(仿真)
- 实物对比实验方案：`docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md`、`docs/重要文档/20260527_SloshPriorityMPC正式对比实验验证方案.md`

**液体视觉真值(两线共用)**
- RGB 离线提取固定流程：`docs/重要文档/红色液体视觉验证固定流程.md`
- 在线液面监控节点：`realsense_liquid_measurement/launch/online_liquid_height.launch`(发 `/liquid/height`)

> 真值口径：RGB max(left,center,right) 液面高度(离线);`/slosh/*`、`/spmpc/slosh_height` 等为模型 proxy,仅辅助。
