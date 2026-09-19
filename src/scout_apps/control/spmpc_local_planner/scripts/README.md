# SPMPC 脚本索引

按用途找脚本：**实物优先看 [real/](real/README.md)，红色液体高度识别看 [rgb/](rgb/README.md)**。

本包使用 ROS1。2026-09-19 整理时，开发分支是 `feat/spmpc-liquid-control`；用户确认实物运行 `origin/diag/lt-dwa-collision-tracking` 的 `8cec9ad5`。分类方便复用，不代表旧实验配置已适配当前两层主线。

## 目录导航

| 目录 | 放什么 | 什么时候用 |
| --- | --- | --- |
| [real/](real/README.md) | 通用实物运行、录包、摘要、C03 消融入口 | 实物实验最常用 |
| [mocap/](mocap/README.md) | 场地地图、固定路径、动捕和执行器辨识 | 检查仿真与实物的运动差异 |
| [rgb/](rgb/README.md) | 相机准备、录制诊断、时间戳检查；跨包识别流程索引 | 红色液体高度测量 |
| [trajectory/](trajectory/README.md) | 全程计划、运输时间搜索、检查点、后缀诊断、新主线录包 | 当前两层主线 |
| [analysis/](analysis/README.md) | bag 分析、画图、验收和共用数值模块 | 实验结束后找原因、做比较 |
| [protocols/](protocols/README.md) | G 系列、I0/O0、早期 smoke、回放和冻结工具 | 复现指定历史实验 |
| [planning/](docs/DEVELOPMENT.md#planning) | 上层规划实现 | 开发模块，由 trajectory 入口调用 |
| [acados/](docs/DEVELOPMENT.md#acados) | 模型、代价、约束和代码生成 | 修改/生成求解器 |
| [lib/](docs/DEVELOPMENT.md#lib) | 实物 wrapper 共用引擎和配置 | 通常由上层脚本调用 |
| [experiments/](docs/DEVELOPMENT.md#experiments) | 新主线录制与配置冻结逻辑 | 由 trajectory 录包入口调用 |
| [tests/](docs/DEVELOPMENT.md#tests) | 单元与协议回归 | 开发检查 |

## 最常用的几个入口

| 目的 | 入口 | 会做什么 |
| --- | --- | --- |
| C03 Full/Smooth 或三组消融 | [run_spmpc_ablation_smoke.sh](real/run_spmpc_ablation_smoke.sh) | 默认校验，`--run` 才采集并运动 |
| 已启动控制器，只录全链 | [record_spmpc_full_rgb_bag.sh](real/record_spmpc_full_rgb_bag.sh) | 只录包；**原始 RGB 默认关闭** |
| 检查 RGB 卡顿 | [diagnose_spmpc_rgb_recording.py](rgb/diagnose_spmpc_rgb_recording.py) | 静态对照接收、RGB 录制、完整录制 |
| 已有 bag，先看整体 | [summarize_spmpc_real_trial.py](real/summarize_spmpc_real_trial.py)、[六图诊断](analysis/plot_spmpc_full_da_diagnostics.py) | 生成摘要和时域图 |
| 核对车体预测/液体改善 | [车体预测](analysis/analyze_robot_state_prediction.py)、[内部液体配对](analysis/analyze_internal_slosh_pair.py) | 内部模型指标需配合独立 RGB 测量 |
| 生成全程防晃计划 | [generate_trajectory_plan.py](trajectory/generate_trajectory_plan.py) | 离线求解或验证，不发运动命令 |

红色液体识别代码仍归 `realsense_liquid_measurement` 传感器包；[RGB 索引](rgb/README.md)已串起调参、标定、录包、离线识别和分析，按[固定流程](../../../../../docs/重要文档/红色液体视觉验证固定流程.md)使用。

## 使用与维护

根目录只保留本索引和分类文件夹，脚本在对应目录中各保留一份。旧根目录入口已移除，调用路径、测试和文档已同步更新；请按上方索引使用新路径。

例如：`bash src/scout_apps/control/spmpc_local_planner/scripts/real/run_spmpc_ablation_smoke.sh --help`。

新增脚本放入对应分类，源码冻结记录本次版本与哈希。详细运行说明和开发索引集中在 [docs/USAGE.md](docs/USAGE.md)、[docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)。
