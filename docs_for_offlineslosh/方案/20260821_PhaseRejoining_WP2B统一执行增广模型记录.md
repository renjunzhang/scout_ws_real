# Phase-Rejoining WP2B 统一执行增广模型记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`6506e9b8`（WP2A 预计发布时间合同）
- 对应缺口：IMP-02/IMP-03/IMP-04 的 C++ 参考模型切片
- 结论：WP2B 通过；WP2、IMP-02/03/04 尚未全部关闭；formal 仍为 G0 NO-GO

## 1. 本切片完成的统一合同

新增无 ROS 的执行模型核心类型：

```text
ExecutionModelContract
ExecutionChannelContract
ExecutionAugmentedState
ExecutionChannelState
ExecutionModel
```

每个执行通道统一描述：

```text
physical delay
integer delay steps + fractional remainder
first-order time constant
positive/negative direction gain
deadzone
output saturation
pending published-command buffer
actuator output
```

`ExecutionModel::configure()` 是 delay 栅格分解的唯一实现。每个 stage 先把新的 `u_pub` 压入线、角两路 buffer，再在各自 fractional event 处分段传播，因此较快通道可以在共同前沿之前生效，较慢通道不会被提前。

执行器目标按 `deadzone -> direction gain -> saturation` 映射；一阶惯性使用精确指数更新。每个 fractional segment 的实际线/角输出继续驱动机器人位姿和同一个 `SloshDynamics` 液体模型。

模型提供：

```text
requiredHistorySec()
executionLeadSec()
gridExecutionLeadSteps()
```

runtime 不再自行重复计算共同执行 lead。

## 2. history predictor 兼容接线

`ExecutionStatePredictor` 仍保持原有 published-history-only 角色和状态码合同，但其传播循环已经下沉到 `ExecutionModel::rolloutPublishedHistory()`。原有能力保持：

- source state age 先于共同前沿传播；
- 线、角通道按不同物理延迟采样；
- empty/partial/complete/stale history 语义不变；
- `require_complete_history` 仍可 fail closed；
- `off/monitor/shadow/fixed_closed_loop/fixed_robot_only` 的应用边界不变；
- 默认增益为 1、死区为 0、饱和为宽边界，因此当前配置行为不被非线性模型改写。

这条接线只消除执行公式的双真源。它没有让求解前尚不存在的本周期新命令进入 history rollout，也没有改变现有 acados OCP 的状态维度。

## 3. 自动化证据

新增测试覆盖：

- physical delay 的整步和 fractional remainder 分解；
- `n=0`、一整步以上及不同线/角延迟下的新决策第一拍因果性；
- 同一 stage 内两个不同 fractional event 的稳定排序和分段；
- 一阶惯性、方向增益、死区和饱和；
- 非法 schema、delay、state、buffer 和非有限命令；
- predictor 的旧 constant/golden、state-age、partial-history 和 closed-loop 行为；
- predictor 的分数延迟跨通道 history causality。

完整结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 540 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK

git diff --check
  PASS
```

## 4. 尚未完成的边界

WP2B 是共享参考模型，不是完整执行因果闭环：

- `expected_publish_stamp` 尚未驱动 source state 到预计发布时间的对齐；
- 本周期新 `u_pub` 虽可由参考模型压入 buffer，但尚未作为 solver 决策量进入 formal OCP；
- C++ 参考模型与 CasADi/generated solver 的随机逐步一致性尚未建立；
- 载荷、地面、电量、速度有效域和辨识误差集合尚未加入并冻结；
- `contract_hash` 只有字段接口，尚未由 typed artifact/preflight 计算和强制校验；
- 现有三组正向仿真仍是零延迟理想执行 proxy，没有验证本模型下的端到端防晃效果；
- ROS receipt 仍不是 driver/CAN/底盘 ACK。

下一切片应让同一个 `expected_publish_stamp` 驱动状态对齐和 solver input，然后在独立 formal solver 生成物中加入 execution augmented state、双通道 buffer 和完整 $N_e=n_f+N_\ell$ horizon。未完成前不得关闭 B0，也不得放行实物 `enforce`。
