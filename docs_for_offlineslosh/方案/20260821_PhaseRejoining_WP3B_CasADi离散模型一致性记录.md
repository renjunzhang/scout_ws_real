# Phase-Rejoining WP3B CasADi 离散模型一致性记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`234b73e2`（WP3A 执行增广求解动力学）
- 对应缺口：IMP-03/IMP-04/IMP-05 的 CasADi/generated 离散转移一致性切片
- 结论：WP3B 通过；独立 acados optimizer capsule、在线 augmented context、terminal gate/$\mathcal B^{\mathrm{exec}}$ 和独立 plant 尚未完成；WP2、WP3、B0 和 formal 仍未关闭，状态继续为 G0 NO-GO

## 1. 冻结的 candidate consistency contract

本轮新增纯 CasADi 离散转移，以 WP3A 的 C++ `DelayAugmentedPhaseDynamics` / `ExecutionModel` 为数值真值。生成合同从现有 planner、platform 和 container YAML 确定性解析，当前候选图像为：

```text
dt                         = 0.0333333333 s
d_v / d_omega              = 0.15 / 0.22 s
linear/angular buffer      = 5 / 7
n_f / N_l / N_e            = 7 / 3 / 10
state/control width        = 22 / 3
contract hash              = e198d2a7e0b4d8b2b530e2bfafb33871f3f9f0cb1baab12e6739b6c35191783d
```

22 维状态顺序为：

```text
[x, y, yaw, v, s, omega, eta_x, eta_x_dot, eta_y, eta_y_dot,
 linear pending buffer[5], angular pending buffer[7]]
```

`robot.v/omega` 是两路 actuator output 的权威镜像，C++ 参考模型现在对不一致状态 fail closed。当前 YAML 的两个 $\tau$ 均为 0；CasADi 表达已包含一阶执行器分支，但本记录不把零惯性候选参数写成 Scout formal 标定结论。

## 2. 确定性生成物

`tools/codegen/acados/spmpc_delay_augmented_phase_model.py` 定义单步转移、单步 Jacobian 和 $N_e$ 步 terminal Jacobian；`generate_delay_augmented_phase_transition.py` 生成：

```text
generated/casadi/spmpc_delay_augmented_phase_transition.c
generated/casadi/spmpc_delay_augmented_phase_transition.h
generated/casadi/spmpc_delay_augmented_phase_manifest.h
```

CMake 只在一致性测试中链接该静态 C 转移核；它不在 `SPMPC_PUBLIC_LIBRARIES` 中，也没有替换任何在线 solver backend。Python 测试会在临时目录重新生成三个文件并逐字节比较，防止手工编辑 generated image 或 YAML 漂移后未重生成。

## 3. 数值一致性证据

新增 4 项 C++ 测试：

- manifest 的 buffer cardinality、fractional delay、$n_f$ 和 $N_e$ 与 C++ resolved contract/horizon 一致；
- 固定 seed `20260821`的128 组随机状态/控制单步，C++ 与 CasADi C 核最大允许误差 $2\times10^{-11}$；
- 第一拍 $a/\alpha$ Jacobian 只写入各自 pending buffer，不得提前影响机器人或液体物理状态；
- 10 步 terminal state 与 C++ rollout 一致，terminal Jacobian 与 C++ 中心差分一致。

另新增 1 项 Python 确定性 codegen 测试。完整回归结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 572 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 93 tests, OK

git diff --check
  PASS
```

## 4. 尚未完成的边界

WP3B 只冻结了“候选 CasADi 离散动力学与 C++ 参考转移一致”，不是 formal optimizer 或 B0 放行：

- 在线 acados capsule 仍是旧 10 维、`N=3` 的 development Phase-Rejoin 模型；
- `ExecutionHorizonContext` 尚未由在线 source state/history 构造出完整 expected-publish-epoch 增广初态；
- published-command 速度/加速度硬约束尚未进入 generated OCP；
- terminal 9 维 gate 和 $\mathcal B^{\mathrm{exec}}$ 尚未共同约束 augmented terminal；
- solver capability/contract hash gate、30 Hz deadline 统计和独立执行 plant 尚未完成；
- $\widehat d_c$、$\tau/K$、死区、滑移和工况误差集尚未由 Scout held-out artifact 冻结。

下一切片应在保持在线默认关闭的前提下，以本 CasADi discrete transition 建立独立 `nx=22, nu=3, N=10` acados capsule，加入 published-command 与 rate 约束、严格 capability/hash gate，再接入完整 augmented initial state 和 terminal 约束。
