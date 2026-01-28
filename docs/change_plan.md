# Scout Mini MPC 局部规划器开发计划

**创建日期**：2026年1月28日  
**最后更新**：2026年1月28日  
**目标**：实现带液体晃动抑制的 MPC 局部规划器

---

## 〇、设计决策（2026-01-28 讨论确定）

### 1. MPC 类型选择

| 方案 | 特点 | 选择 |
|------|------|------|
| **标准 MPC** | 笛卡尔误差 `[x-x_ref, y-y_ref]` | ❌ 对曲线路径效果差 |
| **Frenet MPC** | 沿路径分解 `[e_l, e_c, e_θ]` | ✅ **选用** |
| **完整 MPCC** | 弧长 s 也是优化变量，时间最优 | ❌ 过于激进，不利于液体抑制 |

**选择理由**：液体晃动抑制需要平稳运动，不追求时间最优。Frenet MPC 足够且更简单。

### 2. 误差定义

```
e_l = 纵向误差（沿路径切向，类似 MPCC 的 lag error）
e_c = 横向误差（垂直路径，类似 MPCC 的 contour error）
e_θ = 航向误差（机器人航向与路径切向的夹角）
```

**注意**：这不是完整 MPCC，路径进度由 PathHandler 管理，不是 MPC 优化变量。

### 3. 路径表示方式

| 方案 | 优点 | 缺点 | 选择 |
|------|------|------|------|
| 离散点 + 最近点 | 简单 | 曲率不连续 | ❌ |
| 全局 B样条 | 平滑 | 计算量大 | ❌ |
| **离散点 + 局部三次样条** | 曲率连续，计算量小 | - | ✅ **选用** |

**实现方式**：
```
1. 找最近点 idx
2. 取 [idx-2, idx+N+2] 窗口
3. 局部三次样条拟合（只拟合窗口）
4. 计算 s, κ(s), θ(s)
5. 输出参考点序列 + 曲率
```

### 4. 控制输入选择

| 方案 | 状态维度 | 晃动输入 | 平滑性 | 选择 |
|------|----------|----------|--------|------|
| **速度 (v, ω)** | 3 维 | 需要差分 | 速度可能跳变 | ❌ |
| **加速度 (a, α)** | 5 维 | 直接使用 | 速度平滑 | ✅ **选用** |

**选择理由**：
- 晃动模型需要 `ax, ay`，用加速度输入可直接使用 `ax = a`
- 加速度作为控制量，速度作为状态，天然保证速度连续

### 5. 设计总结

```
┌─────────────────────────────────────────────────────────────┐
│  Frenet MPC + 加速度输入 + 局部三次样条                     │
├─────────────────────────────────────────────────────────────┤
│  第 1 步（5维）                                             │
│  状态: x = [e_l, e_c, e_θ, v, ω]ᵀ                          │
│  控制: u = [a, α]ᵀ                                          │
│  路径: 离散点 + 局部三次样条 → κ(s)                         │
├─────────────────────────────────────────────────────────────┤
│  第 2 步（9维）                                             │
│  状态: x = [e_l, e_c, e_θ, v, ω, η_x, η̇_x, η_y, η̇_y]ᵀ     │
│  控制: u = [a, α]ᵀ                                          │
│  晃动输入:                                                   │
│    ax = a      ← 直接使用控制量                             │
│    ay = v * ω  ← 从状态计算                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 一、开发阶段

### 第 1 步：MPC 路径跟踪（不含晃动）
- **状态**：`[e_l, e_c, e_θ, v, ω]`（5维）
- **控制**：`[a, α]`（加速度输入）
- **路径**：离散点 + 局部三次样条
- **目标**：验证能否稳定跟踪全局路径
- **预计时间**：1 周
- **关键**：预留晃动约束接口

### 第 2 步：添加液体晃动
- **状态扩展**：`[e_l, e_c, e_θ, v, ω, η_x, η̇_x, η_y, η̇_y]`（9维）
- **晃动输入**：`ax = a`（直接用控制量），`ay = v * ω`
- **目标**：添加晃动软约束，验证抑制效果
- **预计时间**：1 周

### 第 3 步：参数调优 + 实车测试
- **预计时间**：1-2 周

---

## 二、整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         独立节点模式架构                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────┐                                                     │
│  │   /goal        │ (RViz / 上层任务)                                   │
│  └───────┬────────┘                                                     │
│          ↓                                                              │
│  ┌────────────────┐         ┌─────────────────────────────────────────┐ │
│  │ Global Planner │         │         scout_local_planner             │ │
│  │ (move_base 或  │ Path    │         (独立 ROS 节点)                  │ │
│  │  自定义节点)   │────────→│                                         │ │
│  │                │         │  ┌─────────────────────────────────┐   │ │
│  │ 发布:          │         │  │      PathHandler                │   │ │
│  │ /global_path   │         │  │  • TF 变换 (map→base_link)      │   │ │
│  └────────────────┘         │  │  • 局部三次样条 → κ(s), θ(s)    │   │ │
│                             │  │  • Frenet 误差计算              │   │ │
│  ┌────────────────┐         │  └──────────────┬──────────────────┘   │ │
│  │   /odom        │────────→│                 ↓                       │ │
│  │  (v, ω 状态)   │         │  ┌─────────────────────────────────┐   │ │
│  └────────────────┘         │  │      MPC Controller             │   │ │
│                             │  │  • Frenet 误差动力学            │   │ │
│  ┌────────────────┐         │  │  • 液体晃动动力学（第2步）      │   │ │
│  │   /tf          │────────→│  │  • OSQP 求解                   │   │ │
│  │ (map→base_link)│         │  └──────────────┬──────────────────┘   │ │
│  └────────────────┘         │                 ↓                       │ │
│                             │           /cmd_vel [v, ω]               │ │
│                             └─────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.1 集成模式：独立节点

**选择理由**：
- 不依赖 move_base 框架，更灵活
- 可独立测试 MPC 控制效果
- 方便后期替换全局规划器

#### 节点架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        独立节点模式                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐         ┌──────────────────────────────┐  │
│  │ 全局规划节点      │         │   scout_local_planner        │  │
│  │ (global_planner) │         │   (独立 ROS 节点)            │  │
│  │                  │         │                              │  │
│  │  订阅:           │         │  订阅:                       │  │
│  │  - /goal         │  Path   │  - /global_path              │  │
│  │                  │────────→│  - /odom                     │  │
│  │  发布:           │         │  - /tf                       │  │
│  │  - /global_path  │         │                              │  │
│  └──────────────────┘         │  发布:                       │  │
│                               │  - /cmd_vel                  │  │
│                               │  - /local_path (可视化)      │  │
│                               │  - /mpc_status               │  │
│                               └──────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| Sub | `global_path` | `nav_msgs/Path` | 全局路径（frame_id=map） |
| Sub | `odom` | `nav_msgs/Odometry` | 里程计（获取 v, ω） |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | 速度命令 |
| Pub | `local_path` | `nav_msgs/Path` | MPC 预测轨迹（调试用） |
| Pub | `mpc_status` | `std_msgs/String` | 节点状态（调试用） |

#### 节点状态机

```
                    ┌─────────────┐
                    │   IDLE      │ ← 启动状态 / 到达目标后
                    └──────┬──────┘
                           │ 收到 global_path
                           ↓
                    ┌─────────────┐
         ┌─────────→│  TRACKING   │←─────────┐
         │          └──────┬──────┘          │
         │                 │                 │
         │    路径更新     │                 │ 恢复正常
         │                 ↓                 │
         │          ┌─────────────┐          │
         │          │  到达目标?  │──否──────┤
         │          └──────┬──────┘          │
         │                 │ 是              │
         │                 ↓                 │
         │          ┌─────────────┐          │
         │          │   REACHED   │          │
         │          └──────┬──────┘          │
         │                 │ 新路径           │
         │                 └─────────────────┘
         │
         │          ┌─────────────┐
         └──────────│   ERROR     │ ← 路径丢失 / 超时
                    └─────────────┘
```

**状态定义**：
- `IDLE`：等待全局路径
- `TRACKING`：正在跟踪路径
- `REACHED`：到达目标点
- `ERROR`：异常状态（路径丢失、求解失败等）

#### 路径更新策略

```
1. 收到新 global_path 时：
   - 如果当前状态是 IDLE/REACHED/ERROR → 切换到 TRACKING
   - 如果当前状态是 TRACKING → 平滑切换到新路径

2. 路径有效性检查：
   - 路径点数 >= 2
   - 路径时间戳不超时（可配置，默认 5s）
   - 路径起点在机器人可达范围内

3. 到达判定：
   - 距离目标点 < goal_tolerance（默认 0.1m）
   - 航向误差 < yaw_tolerance（默认 0.1rad）
```

#### 全局规划器选择（第 1 步测试用）

**推荐**：直接使用 move_base 的 global_planner，remap 输出话题：

```xml
<!-- test_mpc.launch -->
<launch>
  <!-- 全局规划（使用 move_base 的 global_planner） -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <!-- 只启用全局规划 -->
    <param name="base_local_planner" value=""/>  <!-- 禁用内置局部规划 -->
    
    <!-- remap 全局路径输出 -->
    <remap from="~NavfnROS/plan" to="/global_path"/>
    <!-- 或者如果用 GlobalPlanner -->
    <remap from="~GlobalPlanner/plan" to="/global_path"/>
  </node>
  
  <!-- MPC 局部规划（独立节点） -->
  <node pkg="scout_local_planner" type="local_planner_node" name="local_planner" output="screen">
    <rosparam file="$(find scout_local_planner)/config/mpc_params.yaml"/>
    <remap from="global_path" to="/global_path"/>
    <remap from="cmd_vel" to="/scout/cmd_vel"/>
  </node>
</launch>
```

**备选**：后续可替换为自定义全局规划器。

#### TF 与坐标系约定

```
map → odom → base_link

全局路径：frame_id = "map"
局部控制：在 base_link 坐标系计算误差
```

#### PathHandler 处理流程

```
输入：/global_path (nav_msgs/Path, frame_id=map)

步骤：
1. TF 变换：将路径点从 map 变换到 base_link
2. 找最近点：确定当前在路径上的位置 idx
3. 截取窗口：取 [idx-2, idx+N+2] 范围的点
4. 局部样条：对窗口点进行三次样条拟合
5. 计算输出：
   - 参考点序列 (x_ref, y_ref)
   - 切向角 θ_path(s)
   - 曲率 κ(s)
   - Frenet 误差 (e_l, e_c, e_θ)
```

---

## 三、状态空间设计

### 第 1 步（5 维）

```
x = [e_l, e_c, e_θ, v, ω]ᵀ

e_l    = 纵向误差（沿路径切向，类似 lag error）
e_c    = 横向误差（垂直路径，类似 contour error）
e_θ    = 航向误差（机器人航向 θ 与路径切向 θ_path(s) 的差值）
v      = 线速度（base_link 坐标系）
ω      = 角速度（base_link 坐标系）
```

### 第 2 步扩展（9 维）

```
x = [e_l, e_c, e_θ, v, ω, η_x, η̇_x, η_y, η̇_y]ᵀ

η_x, η̇_x = X方向液体晃动模态位移和速度（base_link 坐标系）
η_y, η̇_y = Y方向液体晃动模态位移和速度（base_link 坐标系）

注：符号与 liquid_slosh_model.cpp 保持一致
```

### 控制向量（2 维）

```
u = [a, α]ᵀ = [线加速度, 角加速度]ᵀ

注：所有加速度均在 base_link 坐标系下定义
```

---

## 四、MPC 数学模型

### 4.1 Frenet 误差动力学（核心！）

**重要**：动力学方程只使用当前状态 `v, ω`，不包含 `v_ref`。`v_ref` 只出现在代价函数中。

#### 严格写法（推荐）

```
// 路径弧长推进率
ṡ = v * cos(e_θ) / (1 - κ(s) * e_c)

// Frenet 误差动力学
e_l_dot = v * cos(e_θ) - ṡ           // 纵向误差变化（可简化为 0，见下文）
e_c_dot = v * sin(e_θ)               // 横向误差变化
e_θ_dot = ω - κ(s) * ṡ               // 航向误差变化

// 速度动力学
v_dot = a
ω_dot = α
```

#### 小角度简化写法（实现时可用）

当 `e_θ` 较小时（cos(e_θ) ≈ 1, sin(e_θ) ≈ e_θ）：

```
e_l_dot ≈ v - v_path                 // v_path = ṡ，路径推进速度
e_c_dot ≈ v * e_θ                    // 横向误差
e_θ_dot ≈ ω - κ(s) * v               // 航向误差

v_dot = a
ω_dot = α
```

#### 离散化（欧拉法）

```
e_l[k+1] = e_l[k] + dt * (v[k] - v_path[k])
e_c[k+1] = e_c[k] + dt * v[k] * e_θ[k]
e_θ[k+1] = e_θ[k] + dt * (ω[k] - κ[k] * v[k])

v[k+1] = v[k] + dt * a[k]
ω[k+1] = ω[k] + dt * α[k]
```

**说明**：
- `v_path[k]` = 路径推进速度，由 PathHandler 提供（可设为期望速度 v_des）
- `κ[k]` = 路径曲率，从局部三次样条计算得到
- `v_ref` **不出现在动力学中**，只出现在代价函数

### 4.2 液体晃动动力学（第 2 步添加）

```
// 晃动状态更新
[η_x, η̇_x]ᵀ[k+1] = A_slosh * [η_x, η̇_x]ᵀ[k] + B_slosh * ax[k]
[η_y, η̇_y]ᵀ[k+1] = A_slosh * [η_y, η̇_y]ᵀ[k] + B_slosh * ay[k]

// 加速度计算（base_link 坐标系）
ax[k] = a[k]           // 切向加速度 = 控制量（线加速度）
ay[k] = v[k] * ω[k]    // 法向加速度 = 向心加速度

注：A_slosh, B_slosh 由 LiquidSloshModel 的 ZOH 离散化得到
```

### 4.3 代价函数

**重要**：`v_ref` 只在代价函数中出现，不在动力学中。

```
J = Σₖ {
    // ====== 路径跟踪（Frenet 误差）======
    Q_l * e_l[k]² +           // 纵向误差
    Q_c * e_c[k]² +           // 横向误差（主要！）
    Q_θ * e_θ[k]² +           // 航向误差
    
    // ====== 速度跟踪（v_ref 只在这里！）======
    Q_v * (v[k] - v_ref[k])² +   // 速度误差
    
    // ====== 液体晃动抑制（第 2 步）======
    Q_slosh * h_slosh[k]² +   // 液面高度
    
    // ====== 控制平滑 ======
    R_a * a[k]² +             // 加速度惩罚
    R_α * α[k]² +             // 角加速度惩罚
    
    // ====== 控制变化率 ======
    R_da * (a[k] - a[k-1])² + // 加速度变化
    R_dα * (α[k] - α[k-1])²   // 角加速度变化
}
```

### 4.4 约束条件

```
// 速度约束（状态约束）
-v_max  ≤ v[k]  ≤ v_max       // 线速度
-ω_max  ≤ ω[k]  ≤ ω_max       // 角速度

// 加速度约束（控制约束）
-a_max  ≤ a[k]  ≤ a_max       // 线加速度
-α_max  ≤ α[k]  ≤ α_max       // 角加速度

// 液面高度软约束（第 2 步）
h_slosh[k] ≤ h_max + s[k]     // s[k] 是松弛变量
J += ρ * s[k]²                // 惩罚越界
```

---

## 五、PathHandler 职责（重要！）

PathHandler 负责提供 MPC 所需的路径信息：

```
输入：全局路径（离散点序列）+ 机器人当前位姿

处理：
1. 找到路径上最近点，确定当前弧长 s
2. 对 [s, s + N*dt*v_des] 区间进行局部三次样条拟合
3. 计算未来 N 步的参考点

输出（每个参考点包含）：
├── x_ref, y_ref      // 路径点位置
├── θ_path(s)         // 路径切向角（用于计算 e_θ = θ_robot - θ_path）
├── κ(s)              // 路径曲率（用于 Frenet 动力学）
├── v_path            // 路径推进速度（可设为 v_des 或根据曲率调整）
└── s                 // 弧长参数

Frenet 误差计算：
├── e_l = 投影到切向的误差
├── e_c = 投影到法向的误差
└── e_θ = θ_robot - θ_path(s)
```

**关键**：`κ(s)` 和 `θ_path(s)` 必须从局部三次样条计算，否则 MPC 无法正确预测。

**注意**：这不是 MPCC！
- 弧长 `s` 由 PathHandler 管理，**不是 MPC 的优化变量**
- MPC 只优化控制量 `[a, α]`，路径进度由外部追踪
- 这样设计更简单，且适合液体晃动抑制（不追求时间最优）

---

## 五-A、控制输出说明（重要！）

```
┌─────────────────────────────────────────────────────────────────┐
│  MPC 控制流程                                                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  控制量：u = [a, α]（加速度）                                    │
│      ↓                                                          │
│  状态更新：                                                      │
│      v[k+1] = v[k] + dt * a[k]                                  │
│      ω[k+1] = ω[k] + dt * α[k]                                  │
│      ↓                                                          │
│  输出 cmd_vel：                                                  │
│      cmd_vel.linear.x  = v[k]   ← 从状态读取，不是控制量！       │
│      cmd_vel.angular.z = ω[k]   ← 从状态读取，不是控制量！       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘

说明：
- MPC 内部优化的是加速度 [a, α]
- 但发送给机器人的 cmd_vel 是速度 [v, ω]
- v, ω 是 MPC 的状态变量，每步更新
- 实际发送的是 MPC 预测的下一步状态值 v[1], ω[1]
  （或当前步 v[0], ω[0]，取决于实现）
```

---

## 六、代码架构

```
control/
├── slosh_models/                          # 独立库（第 2 步直接用）
│   ├── include/slosh_models/
│   │   ├── liquid_slosh_model.h           # 已有
│   │   └── slosh_constraint.h             # 新建：晃动约束接口
│   └── src/
│       ├── liquid_slosh_model.cpp         # 已有
│       └── slosh_constraint.cpp           # 新建
│
└── scout_local_planner/
    ├── include/scout_local_planner/
    │   ├── types.h                        # 类型定义（状态、控制、参数）
    │   ├── path_handler.h                 # 路径处理
    │   ├── dynamics_model.h               # 动力学模型（可扩展）
    │   ├── cost_function.h                # 代价函数（可扩展）
    │   ├── constraint_manager.h           # 约束管理器（可插拔）
    │   ├── mpc_solver.h                   # MPC 求解器
    │   └── local_planner_ros.h            # ROS 接口
    │
    ├── src/
    │   ├── path_handler.cpp
    │   ├── dynamics_model.cpp
    │   ├── cost_function.cpp
    │   ├── constraint_manager.cpp
    │   ├── mpc_solver.cpp
    │   └── local_planner_ros.cpp
    │
    ├── config/
    │   ├── mpc_params.yaml
    │   └── vehicle_params.yaml
    │
    └── launch/
        └── test_mpc.launch
```

---

## 六、核心设计原则

```
✅ 模块解耦：晃动模型独立，通过接口注入
✅ 状态可扩展：预留晃动状态位置
✅ 约束可插拔：使用策略模式添加约束
✅ 参数外置：所有参数通过 YAML 配置
✅ 模型可切换：支持差速/滑移转向与全向模型并存
```

---

## 六点五、后期适配全向底盘说明（补充）

当前底盘为差速/滑移转向（skid-steer），使用 `u = [a, α]`。  
为后期适配麦轮/全向底盘，需预留以下扩展点：

1) **控制维度可切换**
   - 差速/滑移转向：`u = [a, α]`（2维）
   - 全向：`u = [a_x, a_y, α]` 或 `u = [v_x, v_y, ω]`（3维）

2) **动力学模型可切换**
   - `DiffDriveModel`（当前）
   - `OmniModel`（后期新增）

3) **误差定义可切换**
   - 差速：Frenet 误差 `[e_l, e_c, e_θ]`
   - 全向：可选 Frenet 或笛卡尔误差 `[e_x, e_y, e_θ]`

4) **液体晃动输入保持统一**
   - 差速：`ax = a`，`ay = v * ω`
   - 全向：`ax, ay` 直接由控制量给出

5) **滑移转向修正（可选）**
   - 添加等效转向系数 `k_slip`，用于修正实际角速度：`ω_eff = k_slip * ω_cmd`

以上扩展不影响当前三步计划，只需在 `DynamicsModelBase` 和 `MPCSolver` 中保留可变维度接口。

## 七、核心接口设计

### 1. 状态索引定义 (`types.h`)

```cpp
struct StateIndex {
    // 基础状态（第 1 步）
    static constexpr int E_L = 0;      // 纵向误差
    static constexpr int E_C = 1;      // 横向误差
    static constexpr int E_THETA = 2;  // 航向误差
    static constexpr int V = 3;        // 线速度
    static constexpr int OMEGA = 4;    // 角速度
    
    // 晃动状态（第 2 步添加）
    // static constexpr int ETA_X = 5;
    // static constexpr int ETA_X_DOT = 6;
    // static constexpr int ETA_Y = 7;
    // static constexpr int ETA_Y_DOT = 8;
    
    static constexpr int BASE_DIM = 5;      // 第 1 步
    static constexpr int SLOSH_DIM = 0;     // 第 2 步改为 4
    static constexpr int TOTAL_DIM = BASE_DIM + SLOSH_DIM;
};
```

### 2. 动力学模型基类 (`dynamics_model.h`)

```cpp
class DynamicsModelBase {
public:
    virtual ~DynamicsModelBase() = default;
    
    // 状态预测：x[k+1] = f(x[k], u[k], ref[k])
    virtual StateVector predict(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt) const = 0;
    
    // 获取线性化矩阵（用于 QP 求解）
    virtual void linearize(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B
    ) const = 0;
};

// 第 1 步：基础差速模型（5 维）
class DiffDriveModel : public DynamicsModelBase { ... };

// 第 2 步：差速模型 + 晃动（9 维）
// class DiffDriveSloshModel : public DynamicsModelBase { ... };
```

### 3. 约束基类 (`constraint_manager.h`)

```cpp
class ConstraintBase {
public:
    virtual std::string name() const = 0;
    virtual int numConstraints() const = 0;
    
    virtual Eigen::VectorXd evaluate(
        const StateVector& x,
        const ControlVector& u) const = 0;
    
    virtual Eigen::VectorXd lowerBound() const = 0;
    virtual Eigen::VectorXd upperBound() const = 0;
};

// 第 2 步添加：晃动约束
// class SloshConstraint : public ConstraintBase { ... };
```

### 4. MPC 求解器 (`mpc_solver.h`)

```cpp
class MPCSolver {
public:
    // 设置动力学模型（支持切换）
    void setDynamicsModel(std::shared_ptr<DynamicsModelBase> model);
    
    // 添加/移除约束（可插拔）
    void addConstraint(std::shared_ptr<ConstraintBase> constraint);
    void removeConstraint(const std::string& name);
    
    // 添加/移除代价项
    void addCostTerm(std::shared_ptr<CostTermBase> term);
    
    // 第 2 步接口（预留）
    // void enableSloshSuppression(const SloshParams& params);
    
    // 核心求解
    Solution solve(
        const FrenetState& current_error,
        double current_v,
        double current_omega,
        const std::vector<ReferencePoint>& reference_path
    );
};
```

---

## 八、第 2 步扩展示例

```cpp
// 1. 修改 types.h 中的状态维度
static constexpr int SLOSH_DIM = 4;  // 改为 4
static constexpr int TOTAL_DIM = BASE_DIM + SLOSH_DIM;  // = 9

// 2. 创建新的动力学模型
auto slosh_model = std::make_shared<DiffDriveSloshModel>(slosh_params);
mpc_solver.setDynamicsModel(slosh_model);

// 3. 添加晃动约束
auto slosh_constraint = std::make_shared<SloshConstraint>(h_max, weight);
mpc_solver.addConstraint(slosh_constraint);

// 4. 添加晃动代价
auto slosh_cost = std::make_shared<SloshCost>(Q_slosh);
mpc_solver.addCostTerm(slosh_cost);
```

---

## 九、配置文件 (`mpc_params.yaml`)

```yaml
mpc:
  # 预测参数
  N: 20                    # 预测步长
  dt: 0.05                 # 时间步长 (s)
  
  # 状态权重
  Q_el: 1.0                # 纵向误差
  Q_ec: 10.0               # 横向误差（主要关注）
  Q_etheta: 5.0            # 航向误差
  Q_v: 1.0                 # 速度误差
  
  # 控制权重
  R_a: 1.0                 # 加速度
  R_alpha: 1.0             # 角加速度
  
  # 控制变化率权重
  R_da: 0.1                # 加速度变化
  R_dalpha: 0.1            # 角加速度变化
  
  # 晃动权重（第 2 步启用）
  Q_slosh: 0.0             # 设为 0 表示不启用
  slosh_height_max: 0.05   # m

vehicle:
  v_max: 1.0               # m/s
  v_min: -0.3              # m/s（倒车）
  omega_max: 1.0           # rad/s
  a_max: 0.5               # m/s²
  alpha_max: 1.0           # rad/s²
  
  wheelbase: 0.0           # 差速轮设为 0
  track_width: 0.456       # Scout Mini 轮距

path_handler:
  lookahead_distance: 1.0  # 前视距离
  goal_tolerance: 0.1      # 到达目标容差
```

---

## 十、代码修改清单

### 0. 全局规划对接（最小可用）

**目标**：先保证能稳定产出 `/scout/global_path`（`nav_msgs/Path`，`frame_id=map`），供局部 MPC 使用。  
**建议**：先用 `move_base` 自带全局规划器（`navfn`/`global_planner`），`scout_global_planner` 先只做配置与 remap。

```
navigation/
└── scout_global_planner/
    ├── config/
    │   └── global_planner.yaml          # [已有/补充] 全局规划参数
    ├── launch/
    │   └── move_base_global.launch      # [新建] move_base + remap -> /scout/global_path
    ├── CMakeLists.txt                   # [已有]
    └── package.xml                      # [已有]
```

> 说明：如果尚未接入 `move_base`，也可先用离线路径或录制路径作为 `/scout/global_path` 的输入。

#### 操作清单（按执行顺序）
1. 启动 SLAM/定位与 TF：确保 `map -> odom -> base_link` 完整
2. 启动地图服务：确认 `/map` 已发布
3. 启动全局规划：`roslaunch scout_global_planner move_base_global.launch`
4. 在 RViz 将 “2D Nav Goal” 话题改为 `/scout/goal`
5. 验证输出：`/scout/global_path` 能持续发布

### 第 1 步需要创建/修改的文件（局部 MPC 核心）

```
scout_local_planner/
├── include/scout_local_planner/
│   ├── types.h                    # [新建] 类型定义
│   ├── cubic_spline.h             # [新建] 三次样条插值
│   ├── path_handler.h             # [新建] 路径处理 + Frenet 转换
│   ├── dynamics_model.h           # [新建] 动力学模型接口
│   ├── diff_drive_model.h         # [新建] 差速模型实现
│   ├── cost_function.h            # [新建] 代价函数
│   ├── constraint_manager.h       # [新建] 约束管理器
│   ├── mpc_solver.h               # [新建] MPC 求解器
│   └── local_planner_ros.h        # [新建] ROS 接口
│
├── src/
│   ├── cubic_spline.cpp           # [新建]
│   ├── path_handler.cpp           # [新建]
│   ├── diff_drive_model.cpp       # [新建]
│   ├── cost_function.cpp          # [新建]
│   ├── constraint_manager.cpp     # [新建]
│   ├── mpc_solver.cpp             # [新建]
│   └── local_planner_ros.cpp      # [新建]
│
├── config/
│   └── mpc_params.yaml            # [新建] 参数配置
│
├── launch/
│   └── test_mpc.launch            # [新建] 测试启动
│
├── CMakeLists.txt                 # [新建]
└── package.xml                    # [新建]
```

### 第 2 步需要添加的文件（晃动扩展）

```
slosh_models/
├── include/slosh_models/
│   └── slosh_constraint.h         # [新建] 晃动约束

scout_local_planner/
├── include/scout_local_planner/
│   └── diff_drive_slosh_model.h   # [新建] 带晃动的动力学模型
│
├── src/
│   └── diff_drive_slosh_model.cpp # [新建]
```

---

## 十一、开发任务清单

| 序号 | 任务 | 文件 | 优先级 | 状态 | 依赖 |
|------|------|------|--------|------|------|
| 1 | 类型定义 | `types.h` | P0 | ⬜ 待开发 | - |
| 2 | 三次样条 | `cubic_spline.h/cpp` | P0 | ⬜ 待开发 | - |
| 3 | 路径处理 | `path_handler.h/cpp` | P0 | ⬜ 待开发 | 1, 2 |
| 4 | 差速动力学 | `diff_drive_model.h/cpp` | P0 | ⬜ 待开发 | 1 |
| 5 | 代价函数 | `cost_function.h/cpp` | P1 | ⬜ 待开发 | 1 |
| 6 | 约束管理器 | `constraint_manager.h/cpp` | P1 | ⬜ 待开发 | 1 |
| 7 | MPC 求解器 | `mpc_solver.h/cpp` | P0 | ⬜ 待开发 | 1, 4, 5, 6 |
| 8 | ROS 接口 | `local_planner_ros.h/cpp` | P2 | ⬜ 待开发 | 3, 7 |
| 9 | 配置文件 | `mpc_params.yaml` | P1 | ⬜ 待开发 | - |
| 10 | CMake 配置 | `CMakeLists.txt, package.xml` | P2 | ⬜ 待开发 | - |
| 11 | 测试启动 | `test_mpc.launch` | P2 | ⬜ 待开发 | 8 |
| 12 | 晃动约束 | `slosh_constraint.h/cpp` | P3 | ⬜ 第2步 | - |
| 13 | 晃动动力学 | `diff_drive_slosh_model.h/cpp` | P3 | ⬜ 第2步 | 4, 12 |

---

## 十二、依赖项

### 必须安装

```bash
# OSQP 求解器
sudo apt-get install libosqp-dev

# Eigen（通常已安装）
sudo apt-get install libeigen3-dev

# ROS 导航栈
sudo apt-get install ros-noetic-navigation
```

### 可选

```bash
# 用于可视化调试
sudo apt-get install ros-noetic-plotjuggler-ros
```

---

## 十三、待确认参数

| 参数 | 建议值 | 实际值 | 备注 |
|------|--------|--------|------|
| Scout Mini 最大线速度 | 1.0 m/s | ? | |
| Scout Mini 最大角速度 | 1.0 rad/s | ? | |
| 最大加速度 | 0.5 m/s² | ? | |
| 最大角加速度 | 1.0 rad/s² | ? | |
| MPC 预测步长 N | 20 | ? | |
| MPC 时间步长 dt | 0.05 s | ? | |
| 液体容器半径 | - | ? | 第 2 步需要 |
| 液体静态高度 | - | ? | 第 2 步需要 |

---

## 十四、开发建议顺序

```
第 1 天：types.h + cubic_spline.h/cpp
    ↓
第 2 天：path_handler.h/cpp（路径处理 + Frenet 转换）
    ↓
第 3 天：diff_drive_model.h/cpp（差速运动学）
    ↓
第 4 天：cost_function.h/cpp + constraint_manager.h/cpp
    ↓
第 5 天：mpc_solver.h/cpp（核心求解器）
    ↓
第 6 天：local_planner_ros.h/cpp + 配置文件
    ↓
第 7 天：测试 + 调参
```

---

**最后更新**：2026年1月28日
