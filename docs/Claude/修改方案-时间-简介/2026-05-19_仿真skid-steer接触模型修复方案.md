# 2026-05-19 仿真 skid-steer 接触模型修复方案

## 1. 背景与触发问题

当前 Gazebo 仿真存在三个层次的问题:

1. **静态/低速**:Scout Mini 原地自转时车体抖动,旋转中心系统性偏移(不是绕几何中心)。
2. **动态轨迹**:实物上能稳定跟踪的轨迹,仿真里 MPC 跟不上,sim-real gap 明显。
3. **目标层**:后续要在该仿真上做 RL 训练 anti-slosh 策略,当前模型质量不足以支撑。

`docs/重要文档/仿真笔记.md` 第 7.4 节把这一现象归因于 "Gazebo 四轮 skid-steer 接触模型固有限制",建议不修。该结论在"只把仿真当 MPC 演示"的前提下成立,但**不适用于 sim-real 迁移和 RL 训练**,需要重新评估。

## 2. 启动链与加载文件

```text
launch_sim_nav_stack.sh
 → scout_mini_true_empty_bridge.launch
   → scout_mini_true_empty_abs_imu.launch
     → spawn_scout_mini_abs_imu.launch
       → scout_mini_stock_abs_imu.launch
         → /data/a/official_scout_ws/src/scout_description/urdf/mini_abs.xacro
             ├── scout_mini_wheel_{1,2,3,4}_abs.xacro
             └── scout_mini.gazebo
```

控制器 YAML: `/data/a/official_scout_ws/src/scout_gazebo_sim/config/scout_v2_control.yaml`
URDF extras 注入点(`mini_abs.xacro:117`): `/home/a/scout_ws/src/scout_ros/scout_description/urdf/official_scout_mini_imu_extras.urdf.xacro`

## 3. 官方模型实际缺陷清单(按贡献度)

### A. 摩擦/接触参数被整块注释掉

`scout_mini.gazebo:21-29` 的全局摩擦块是注释状态,`scout_mini_wheel.gazebo` 也只有 `<transmission>` 没有任何轮子 `<gazebo reference="..._wheel_link">` 接触块。

后果:四个轮子全部走 Gazebo 默认 `mu1=mu2=1` 各向同性摩擦,没有 `fdir1`、没有重设 `kp/kd`。**头号原因**。

### B. 右侧两个轮子人为抬高 1 mm

`mini_abs.xacro:111,114`:

```xml
<origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset+0.001}" .../>
<origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset+0.001}" .../>
```

左侧没有 `+0.001`。四轮不共面 → 静止时右侧轮可能间歇离地 → 法向力左右不对称 → **自转中心系统性偏移的直接原因**。

### C. skid-steer 插件 torque=1000 N·m 且无加速度限制

`scout_mini.gazebo:122`: `<torque>1000</torque>`,实物单轮电机扭矩 20–30 N·m,放大 30–50 倍。
`libgazebo_ros_skid_steer_drive` 无 `wheelAcceleration` 字段时瞬时打满力矩 → 接触瞬时大穿透 → 反弹抖动。
**同时**导致 sim 阶跃响应比实物快得多,是"实物轨迹 sim 跟不上"的核心原因之一。

### D. base_link 惯量低估 ~2 倍

`mini_abs.xacro:28`:`ixx=iyy=0.1354, izz=0.2667`,mass=10。
按 0.62×0.585×0.235 thin-box 估算应为 `ixx≈0.326, izz≈0.601`。车体偏"轻飘",瞬时摩擦力更容易推动 → 加剧漂。

### E. wheel collision 原点偏移 0.03 m

`scout_mini_wheel_*_abs.xacro:18`: `<origin xyz="0 0 0.03" .../>`。
collision cylinder 与 visual 不同心,接触点偏离实际轮缘。

### F. base_link collision 是 0.4×0.4×0.05 扁 box

只覆盖车体一小块,不影响自转(高度上不和地接触),但对碰撞/recovery 不真实。优先级最低。

## 4. 修复路线(按性价比)

| 序号 | 修法 | 改在哪 | 工作量 | 预期收益 |
|----|----|----|----|----|
| A | 在 `official_scout_mini_imu_extras.urdf.xacro` 追加四个轮子的 `<gazebo reference="..._wheel_link">` 块(`mu1/mu2/fdir1/kp/kd/minDepth/maxContacts`) | bridge 侧,不动官方 | ~30 行 xacro | 抖动降 70%+ |
| C | 限制 `<torque>` 到 30 N·m 级 + 加 `<wheelAcceleration>`;或在控制层加速度 ramp 滤波 | `scout_mini.gazebo`(需 fork)或上层 cmd_vel 滤波 | 几行 | sim 阶跃响应接近实物 + 减抖 |
| B | 修右侧轮 `+0.001` 偏置,统一回 `wheel_vertical_offset` | `mini_abs.xacro`(需 fork) | 两个数字 | 自转中心方向漂消失 |
| D | 重算 base_link 惯量,改 `ixx/iyy/izz` | `mini_abs.xacro`(需 fork) | 一行 | 抖动再降一档 |

修法 A 是唯一可以在 bridge 侧 xacro extras 完成的(因为 Gazebo 会合并同名 `<gazebo reference>` 块)。其余必须改 URDF 本体。

**Fork 策略**:把 `/data/a/official_scout_ws/src/scout_description` 复制到本仓库 `src/scout_ros/scout_description_fixed/`(或同名 overlay),launch 链改指向 fork。既能版本化 patch、又不污染官方 ws。

## 5. 执行顺序与验证

按 **A → C → B → D** 依次推进,每步独立验证:

1. **A**:写 patch 到 `official_scout_mini_imu_extras.urdf.xacro`。验证:相同 spawn 位姿下纯角速度 `cmd_vel(0, 0.2)` 30s,对比 `/gazebo/model_states` 中 `pose.position` 的最大偏移和 `twist.angular.z` 的 RMS 抖动。**目标**:位置偏移峰值降 50%+,IMU `angular_velocity.z` 一阶差分谱在 5–20 Hz 的能量降 70%+。
2. **C**:torque + acceleration 限制。验证:阶跃 `cmd_vel(0→0.5, 0)` 下,`/odom.twist.linear.x` 10–90% rise time 是否接近实物 bag(实物经验 ~150–300 ms)。
3. **B**:消除右轮 +0.001。验证:`cmd_vel(0, 0.2)` 持续 30s,`/gazebo/model_states` 中位置的方向性漂移是否消失(漂为零均值噪声而非单边偏置)。
4. **D**:重算惯量。验证:同 A 抖动指标再降。

**统一回归测试**:固定 S 弯 open-loop(写死 `(v, ω)` 序列)对比 Gazebo world pose 轨迹和实物录的同序列轨迹,RMSE 应进入可接受范围(目标 < 5 cm / 10 m)。

## 6. 不在本方案内的事

- 不改 wheel axis 符号(已确认正常,见笔记 7.4)。
- 不改 ODE 全局 solver 参数(`max_step_size/iters/sor`)。代价/收益比低,且会拉低 RTF,污染 anti-slosh 时间尺度。如果 A+B+C+D 后仍有显著残余抖动,再单独立项。
- 不引入 `libgazebo_ros_planar_move` kinematic 插件替代物理仿真。anti-slosh 必须保留底盘动力学(加速度是 slosh 的源头),不能用 kinematic 模型绕开。
- 不评估 Isaac Sim / MuJoCo 迁移。属于"RL 训练阶段"决策,本方案先把 Gazebo 修到能用,再做平台选型评估。

## 7. 后续衍生工作(占位,不在本次执行)

- 实物 system identification:录一段标准阶跃 + 标准 S 弯 bag,提取实物 cmd→speed 一阶传函,作为 C/D 步参数标定依据。
- 修复完成后,作为 RL 训练的 sim 基线,评估是否还需切到 Isaac Sim。
