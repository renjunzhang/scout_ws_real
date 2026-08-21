# Phase-Rejoining WP3A 执行增广求解动力学记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`c4e52c4b`（WP2C 预计发布时间贯通）
- 对应缺口：IMP-03/IMP-04/IMP-05 的纯 C++ solver 参考转移切片
- 结论：WP3A 通过；WP2、WP3、B0 和 formal solver 尚未关闭；formal 仍为 G0 NO-GO

## 1. Solver typed horizon

新增 `ExecutionHorizonContext`，显式绑定：

```text
resolved ExecutionModelContract
expected-publish-epoch initial augmented state
initial progress
n_f = grid execution-front steps
N_l = liquid horizon steps
N_e = n_f + N_l
physical front / grid front / terminal epoch
```

完整 context 已加入 `SolverInput`，但默认 `active=false`。当前在线 acados backend 尚未消费它，因此既有 off/monitor/enforce 行为没有改变。

## 2. Delay-augmented Phase-Rejoin 参考转移

新增无 ROS、无 acados 的 `DelayAugmentedPhaseDynamics`。每一 solver stage 的控制保持：

```text
q = [a, alpha, v_s]
```

但 $a$ 和 $\alpha$ 的语义是上一真实发布速度命令的变化率：

$$
u^{\mathrm{pub}}_{v,k}=u^{\mathrm{pub}}_{v,k-1}+a_k\Delta t,
\qquad
u^{\mathrm{pub}}_{\omega,k}=u^{\mathrm{pub}}_{\omega,k-1}+\alpha_k\Delta t.
$$

新 $u^{\mathrm{pub}}_k$ 随后进入 `ExecutionModel::step()` 的线/角 pending buffer，再依次经过 fractional delay、死区/方向增益/饱和、一阶执行器、机器人和液体传播。`v_s` 同步推进 progress。

这里特意从 pending buffer 的最后一个已发布命令积分，而不是从测得的 `robot.v/omega` 积分，避免延迟存在时把物理执行状态错误当成命令状态。

## 3. 时间和 cardinality 合同

参考动力学由同一个 `ExecutionModel` 给出：

```text
n_f = gridExecutionLeadSteps()
N_e = n_f + N_l
t_physical_front = t_hat_pub + max(d_v, d_omega)
t_grid_front = t_hat_pub + n_f * dt
t_terminal = t_hat_pub + N_e * dt
```

context 的 contract、buffer cardinality、初始 epoch、三个派生 epoch 或任一 horizon 数被篡改时均 fail closed。rollout 只接受严格等于 $N_e$ 的控制序列。

## 4. 自动化证据

新增 6 项针对性测试，覆盖：

- physical/grid/terminal epoch 来自同一 fractional-delay 合同；
- 当前决策从上一真实 `u_pub` 而不是 measured velocity 积分；
- 150 ms 线延迟与 250 ms 角延迟的第一拍不同生效时刻；
- $N_e=n_f+N_l$ 的 state/control/publication cardinality；
- 第一拍线/角 residual 对机器人和液体联合终端的非零灵敏度；
- context mutation、错误 horizon 和非有限控制 fail closed。

完整结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 564 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK

git diff --check
  PASS
```

## 5. 尚未完成的边界

WP3A 是 formal solver 的 C++ 数值真值和 API 骨架，不是 optimizer：

- source state/history 尚未构造 expected-publish-epoch 的完整 pending buffer 初态；
- current online acados capsule 仍是旧 10 维短窗模型；
- CasADi discrete model、generated capsule 和 C++ 随机逐步一致性尚未建立；
- published-command 速度/加速度硬约束尚未进入 formal OCP；
- 9 维 terminal gate 与 $\mathcal B^{\mathrm{exec}}$ 尚未共同约束 augmented terminal；
- 独立执行 plant、参数偏差和故障注入尚未建立；
- $\widehat d_c$ 和执行合同仍未由 Scout held-out artifact/hash 冻结。

下一切片应以本参考转移为唯一数值语义，新增 CasADi discrete model 和 generated solver，并先完成随机单步、第一拍 Jacobian、horizon/index 和终端 epoch 一致性；未通过前不能把现有 `spmpc_phase_rejoin` capsule 改称 formal delay-augmented solver。
