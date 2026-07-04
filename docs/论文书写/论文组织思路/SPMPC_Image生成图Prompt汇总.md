# SPMPC 可用 Image 生成的论文示意图 Prompt 汇总

本文档汇总适合用 image2.0 先生成视觉草图/底图的 SPMPC 论文图。后续正式论文中的 SVG、矢量重绘、文字标签和公式标注可再手动完成。

## 通用生成原则

适合 image2.0 生成的是：

- 研究缺口图；
- 方法框架图；
- 晃液模型示意图；
- 滚动时域预测示意图；
- 实验场景概念示意图。

不建议 image2.0 直接生成最终版的是：

- 消融实验曲线；
- 基线对比数据图；
- 真实轨迹图；
- 液面高度曲线；
- Pareto 曲线；
- 求解时间统计图；
- 实物实验照片。

这些必须来自真实实验、仿真数据或实物拍摄。

## 通用负面要求

每次生成时都可以附加以下约束：

```text
No fake experimental data, no numerical plots, no realistic paper text, no equations, no watermarks, no logos, no decorative background, no cluttered labels. Use minimal placeholder labels only. Keep the layout clean and suitable for later manual annotation.
```

中文含义：不要生成假的实验数据，不要生成具体数值曲线，不要生成大段文字，不要生成公式，不要水印，不要标志，不要复杂背景；只保留少量占位标签，方便后续手动画 SVG 和加文字。

---

# Fig. 1 研究缺口与方法定位图

## 用途

建议放在引言，用于说明本文不是普通防晃 MPC，也不是普通局部规划，而是把晃液动态状态引入在线局部规划层。

## 希望表达的内容

三栏对比：

1. 普通移动机器人局部规划器：能在线输出可执行底盘命令，但不传播液体状态；
2. 已有防晃方法：考虑液体模型，但多位于离线轨迹、速度剖面、输入整形、跟踪控制或特殊平台层；
3. SPMPC：把低阶晃液模态状态、路径进度和底盘状态放入滚动时域 MPCC 局部规划器。

## Prompt

```text
A clean academic vector-style diagram for a robotics paper, white background, three-column comparison layout.

Left panel: a wheeled mobile robot carrying an open liquid container follows a reference path. Show a conventional online local planner that receives a path and robot state, then outputs executable velocity commands to the mobile base. The liquid container is shown but not connected to the planner, indicating that liquid state is not modeled.

Middle panel: anti-sloshing methods are shown as separate higher-level or lower-level modules: offline trajectory generation, speed profiling, input shaping, and trajectory tracking control. Show a liquid model icon connected to these modules, but not as a standard online local planner interface.

Right panel: the proposed SPMPC online local planner integrates robot state, path progress, and low-order liquid slosh modal state into a receding-horizon MPCC optimizer. Show arrows from robot state, reference path, and liquid modal state into one optimizer block, then output linear and angular velocity commands to the mobile base.

Use thin lines, blue and gray color palette, simple block diagram style, clean arrows, minimal placeholder labels only, no detailed text, no equations, no fake data, no logos, no watermark.
```

---

# Fig. 2 SPMPC 总体框架图

## 用途

建议放在方法章开头，对应当前方法结构图占位。它应当说明输入、增强状态、OCP 和输出命令之间的关系。

## 希望表达的内容

输入：

- 里程计/当前机器人状态；
- 参考路径；
- 上一帧控制；
- 低阶晃液状态初值。

中间：

- 增强状态模型；
- 低阶晃液动力学；
- MPCC OCP；
- 路径跟踪、路径进度、控制平滑性、预测晃液响应。

输出：

- 第一帧控制；
- `/cmd_vel`；
- 滚动时域循环。

## Prompt

```text
A clean technical block diagram for a robotics control paper, white background, IEEE-style two-column paper figure.

On the left, show four input blocks: mobile robot odometry, reference path, previous command, and low-order liquid slosh state. Use simple icons: robot pose, curved path, command arrow, and open container with a small wave.

In the center, show an augmented dynamics model block combining mobile robot state, path progress, and low-order slosh modal state. Show the mobile base model and the liquid modal oscillator model as two connected sub-blocks inside the augmented model.

On the right-center, show a receding-horizon MPCC optimal control problem block. Inside it, use four small cost icons: path tracking, progress reward, control smoothness, and predicted slosh response. Do not include equations.

On the far right, show that only the first optimized control action is sent to the robot as linear and angular velocity command, then a curved feedback arrow returns to the next planning cycle.

Use clean arrows, blue accent color, gray blocks, minimal placeholder labels, no long text, no formulas, no fake data, no watermark.
```

---

# Fig. 3 低阶晃液模型与增强状态示意图

## 用途

建议放在方法章晃液动力学模型附近。它帮助读者理解为什么要把 `eta` 和 `eta_dot` 放进状态。

## 希望表达的内容

- 移动底盘携带开口容器；
- 容器中液面晃动；
- 前向加速度激励；
- 转弯产生横向激励；
- 低阶模态状态表示液体动态记忆；
- 底盘状态 + 路径进度 + 液体模态状态 = 增强状态。

## Prompt

```text
A clean vector illustration for a robotics and control paper, white background, showing a wheeled mobile robot carrying an open liquid container.

Create a combined side-view and top-view schematic.

In the side view, show the open container mounted on the mobile robot, with a blue sloshing free surface. Overlay a simplified second-order modal oscillator representation beside the container, using a mass-spring-damper style icon to represent low-order liquid slosh dynamics.

In the top view, show the robot moving along a curved path. Add a forward acceleration arrow and a lateral turning-induced excitation arrow. Show linear velocity and angular velocity conceptually using simple arrows, but avoid detailed formulas.

Visually emphasize that the liquid has dynamic memory by showing the slosh wave state continuing over time, not just instantaneous acceleration.

Use academic vector style, thin lines, blue liquid, gray robot body, simple arrows, minimal placeholder labels only, no equations, no realistic background, no fake data, no watermark.
```

---

# Fig. 4 滚动时域预测与第一帧执行示意图

## 用途

建议放在方法章滚动时域执行部分，说明 SPMPC 不是离线轨迹生成，而是在线重复优化。

## 希望表达的内容

- 当前时刻；
- 预测时域；
- 未来若干步机器人轨迹；
- 未来晃液状态传播；
- 只执行第一帧控制；
- 下一周期重新优化。

## Prompt

```text
A clean academic diagram illustrating receding horizon control for a mobile robot carrying an open liquid container, white background, IEEE-style figure.

Show a horizontal time axis. Mark the current time at the left and a prediction horizon extending to the right. Along the horizon, show a sequence of small robot icons following a predicted curved trajectory. On each robot, show a small open container with a changing blue liquid wave, indicating predicted slosh state propagation.

Highlight the first control action with a bold arrow and show that only this first action is applied. Then show the horizon shifting forward and the optimization repeating at the next time step.

Use a clean blue and gray palette, thin lines, simple arrows, minimal placeholder labels, no equations, no detailed text, no fake data plots, no watermark.
```

---

# Fig. 5 MPCC 路径进度、轮廓误差和滞后误差示意图

## 用途

建议放在方法章 MPCC 问题构造部分。

## 注意

这张图对几何准确性要求较高，最终更推荐手动画 SVG、TikZ 或 Python 矢量图。image2.0 可以先生成构图参考，但不要直接作为最终图。

## 希望表达的内容

- 参考路径曲线；
- 当前机器人位置；
- 路径投影点；
- 切向方向；
- 法向方向；
- 轮廓误差；
- 滞后误差；
- 路径进度。

## Prompt

```text
A clean geometric schematic for a model predictive contouring control paper, white background.

Show a smooth reference path curve and a mobile robot point near the path. Show the closest or progress-related reference point on the path. Draw a tangent direction arrow along the path and a normal direction arrow perpendicular to the path. Show two error components from the robot position to the path reference frame: one lateral contour error and one longitudinal lag error. Show path progress along the curve with a small arrow.

Use precise-looking geometry, thin lines, blue reference path, gray robot point, red and orange error arrows, minimal placeholder labels only. Avoid equations, avoid long text, avoid decorative elements, no fake data, no watermark.
```

---

# Fig. 6 实验场景概念示意图

## 用途

建议放在实验章开头，用于说明实验任务环境和路径类型。正式结果最好用真实仿真截图或实物照片替换，但这张可以作为概念示意图。

## 希望表达的内容

- 室内平整环境；
- 移动底盘携带开口液体容器；
- 起点、终点和参考路径；
- 直线路径、转弯路径、S 弯路径或窄通道路径；
- 强调局部规划和液体晃动风险。

## Prompt

```text
A clean conceptual experiment scenario illustration for a robotics paper, white background, top-down indoor environment.

Show a wheeled mobile robot carrying an open liquid container moving along a reference path from a start point to a goal point. The environment is an indoor flat-floor setting with simple laboratory-like benches or corridor boundaries, but keep it abstract and not cluttered. Show several representative path shapes: a straight segment, a right-angle turn, and an S-shaped curve, as faint alternative paths or small inset panels.

Use blue path lines, gray floor boundaries, simple robot icon, open container with blue liquid wave. Emphasize that acceleration, braking, and turning can cause liquid sloshing. Use minimal placeholder labels only, no detailed text, no fake measurements, no realistic photo style, no watermark.
```

---

# Fig. 7 图形摘要/论文主视觉草图（可选）

## 用途

可用于汇报 PPT、论文首页草图或后续图形摘要，不一定放入正式 RAL 正文。

## 希望表达的内容

一句话概括：

> 带开口液体容器的移动底盘，在在线局部规划中同时考虑路径跟踪、底盘可执行性和液体动态记忆。

## Prompt

```text
A clean graphical abstract for a robotics control paper, white background.

Show a mobile robot carrying an open liquid container moving along a curved path. On one side, show a conventional local planner icon that only considers robot motion. On the other side, show the proposed slosh-aware predictive local planner that includes robot state, path progress, and liquid slosh modal state. Show a receding horizon trajectory ahead of the robot and a small predicted liquid wave evolution inside the container.

The visual message should be: liquid sloshing is a dynamic state prediction problem, not merely a trajectory smoothing problem.

Use elegant academic vector illustration, blue liquid, gray robot, clean arrows, minimal placeholder labels, no long text, no equations, no fake data, no logos, no watermark.
```

---

# 推荐优先生成顺序

建议马上优先生成以下三张：

1. **Fig. 1 研究缺口与方法定位图**  
   用于引言，帮助老师和读者快速理解 novelty。

2. **Fig. 2 SPMPC 总体框架图**  
   用于方法章，是当前最重要的结构图。

3. **Fig. 3 低阶晃液模型与增强状态示意图**  
   用于解释晃液动态记忆和增强状态。

如果还有时间，再生成：

4. **Fig. 4 滚动时域预测图**；
5. **Fig. 6 实验场景概念图**。

---

# 后续手动画 SVG 时建议保留的图形元素

无论 image2.0 生成结果如何，后续正式矢量图建议保留这些元素：

- 移动底盘图标；
- 开口容器和蓝色液面；
- 参考路径；
- 当前状态与预测时域；
- 低阶晃液模态状态；
- MPCC/OCP 优化器方框；
- `/cmd_vel` 或速度命令输出；
- 滚动时域反馈箭头。

文字、公式和变量标签建议后期手动加，避免图片生成模型产生乱码。