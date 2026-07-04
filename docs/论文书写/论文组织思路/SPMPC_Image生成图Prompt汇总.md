# SPMPC 可用 Image 生成的论文示意图 Prompt 汇总

本文档已按当前 `main_twocolumn` 投稿化初稿对齐。当前正文中的主要图位是：

- **Fig. 1：任务与方法定位示意图**，放在引言；
- **Fig. 2：增强状态与 OCP 结构图**，放在方法章；
- **Fig. 3：实验平台与评价流程图**，放在实验章；
- **Fig. 4：内部消融结果图**，放在实验章，但最终必须由真实数据生成。

image2.0 适合生成 Fig. 1、Fig. 2、Fig. 3 的视觉草图/底图；Fig. 4 只能生成“版式草图”，不能生成最终结果图。后续正式论文中的 SVG、矢量重绘、中文/英文标签和公式标注可再手动完成。

---

## 0. 通用生成原则

适合 image2.0 生成的是：

- 研究缺口与方法定位图；
- SPMPC 增强状态与 OCP 框架图；
- 低阶晃液模型示意元素；
- 滚动时域预测示意元素；
- 实验平台与评价流程示意图；
- 图形摘要或汇报用主视觉草图。

不建议 image2.0 直接生成最终版的是：

- 内部消融实验曲线；
- 基线对比数据图；
- 真实轨迹图；
- 液面高度曲线；
- Pareto 曲线；
- 求解时间统计图；
- 实物实验照片。

这些必须来自真实实验、仿真数据或实物拍摄。

每次生成时都建议附加以下负面要求：

```text
No fake experimental data, no numerical plots, no realistic paper text, no equations, no watermarks, no logos, no decorative background, no cluttered labels. Use minimal placeholder labels only. Keep the layout clean and suitable for later manual annotation.
```

中文含义：不要生成假的实验数据，不要生成具体数值曲线，不要生成大段文字，不要生成公式，不要水印，不要标志，不要复杂背景；只保留少量占位标签，方便后续手动画 SVG 和加文字。

---

# Fig. 1 任务与方法定位示意图

## 正文位置

引言，当前占位图对应：

```text
Fig. 1：任务与方法定位示意图
普通局部规划 / 防晃方法 / SPMPC 在线局部规划
```

## 用途

用于说明本文不是普通防晃 MPC，也不是普通局部规划，而是把晃液动态状态引入标准轮式移动底盘在线局部规划层。

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

## 后续手动画时建议保留的标签

- Conventional local planner
- Anti-sloshing methods
- SPMPC online local planner
- Robot state
- Reference path
- Slosh modal state
- Velocity command

中文正文图中可改成：普通局部规划器、防晃方法、SPMPC 在线局部规划器。

---

# Fig. 2 SPMPC 增强状态与 OCP 结构图

## 正文位置

方法章，当前占位图对应：

```text
SPMPC 增强状态与 OCP 结构图
机器人状态 + 路径进度 + 晃液状态 → 增强动力学与 MPCC OCP → 第一帧底盘命令
```

## 用途

这是当前方法章最重要的图，用于说明输入、增强状态、低阶晃液模型、MPCC OCP 和底盘命令输出之间的关系。

## 希望表达的内容

输入：

- 里程计/当前机器人状态；
- 参考路径；
- 上一帧控制；
- 低阶晃液状态初值。

中间：

- 机器人状态；
- 路径进度；
- 低阶晃液模态状态；
- 增强动力学；
- MPCC OCP；
- 路径误差、路径进度、控制平滑性、预测晃液响应。

输出：

- 第一帧优化控制；
- 线速度/角速度命令；
- 滚动时域反馈循环。

## Prompt

```text
A clean technical block diagram for a robotics control paper, white background, IEEE-style two-column paper figure.

On the left, show four input blocks: mobile robot odometry, reference path, previous command, and low-order liquid slosh state. Use simple icons: robot pose, curved path, command arrow, and open container with a small wave.

In the center, show an augmented dynamics model block combining mobile robot state, path progress, and low-order slosh modal state. Inside this model, show a mobile base model and a simplified liquid modal oscillator model connected together. The liquid model should be represented by a small open container with a blue sloshing wave and a simple mass-spring-damper or oscillator icon.

On the right-center, show a receding-horizon MPCC optimal control problem block. Inside it, use four small cost icons: path tracking, progress reward, control smoothness, and predicted slosh response. Do not include equations.

On the far right, show that only the first optimized control action is sent to the robot as linear and angular velocity command, then a curved feedback arrow returns to the next planning cycle.

Use clean arrows, blue accent color, gray blocks, minimal placeholder labels, no long text, no formulas, no fake data, no watermark.
```

## 可选增强元素

如果 image2.0 画面空间足够，可以加入：

- 小时间轴，表示 receding horizon；
- 小波形图标，表示预测晃液状态沿时域传播；
- `/cmd_vel` 图标，表示标准底盘接口。

---

# Fig. 3 实验平台与外部液面评价流程图

## 正文位置

实验章，当前占位图对应：

```text
Fig. 3：实验平台与评价流程占位
底盘、开口容器、参考路径、RGB/外部液面观测、日志记录
```

## 用途

用于说明实验系统和评价流程，而不是展示结果。它应体现：真实液面结论来自外部观测，内部 `/spmpc/slosh_height` 只是模型代理量。

## 希望表达的内容

- 移动底盘携带开口液体容器；
- 参考路径、起点和终点；
- 外部 RGB 相机或液面观测模块；
- 规划器日志，包括轨迹、命令、求解时间、内部晃液代理量；
- 评价指标计算模块，包括真实液面指标、任务完成、路径跟踪、控制平滑性、实时性。

## Prompt

```text
A clean conceptual experiment setup and evaluation pipeline illustration for a robotics paper, white background, IEEE-style figure.

Show a wheeled mobile robot carrying an open liquid container moving along a reference path from a start point to a goal point in a simple indoor flat-floor environment. Keep the environment abstract and clean, with simple laboratory-like benches or corridor boundaries if needed.

Show an external RGB camera or external liquid-level observation module looking at the open container. Make clear that the real liquid surface is evaluated by external observation, not by the planner itself.

On the side, show a simple data logging and evaluation pipeline: robot trajectory, velocity commands, solver time, internal model-predicted slosh proxy, and external liquid-surface metric. Represent these as clean blocks and arrows, not as numerical plots.

Use blue path lines, gray floor boundaries, simple robot icon, open container with blue liquid wave, clean arrows, minimal placeholder labels only, no detailed text, no fake measurements, no realistic photo style, no watermark.
```

## 后续手动画时建议保留的标签

- Reference path
- Mobile base with open container
- External liquid observation
- Planner logs
- Real liquid metric
- Internal slosh proxy

中文正文图中可改成：参考路径、开口容器、外部液面观测、日志记录、真实液面指标、内部代理量。

---

# Fig. 4 内部消融结果图

## 正文位置

实验章，当前占位图对应：

```text
Fig. 4：内部消融结果占位
B0 / B_smooth / B_slosh / B_ours 的轨迹、速度、液面响应和求解时间
```

## 重要说明

Fig. 4 是论文核心证据图，最终必须来自真实仿真/实物数据，不能用 image2.0 生成最终结果。image2.0 只能用于生成“版式草图”，帮助决定图的布局，例如 2×2 子图排布、颜色风格和图例位置。

## 建议最终内容

最终 Fig. 4 应由真实数据生成，建议包含：

- 四种方法的轨迹或路径误差；
- 线速度/角速度或加速度曲线；
- 真实液面指标曲线；
- 内部预测晃液状态或 `/spmpc/slosh_height`；
- 求解时间或任务完成统计。

## 仅用于版式草图的 Prompt

```text
A clean layout mockup for an experimental results figure in a robotics paper, white background, IEEE-style multi-panel figure.

Create a 2 by 2 panel layout only, without real numerical data. Panel A should be reserved for trajectory comparison of four methods. Panel B should be reserved for velocity or control smoothness curves. Panel C should be reserved for real liquid-surface response. Panel D should be reserved for solver time or summary statistics.

Use placeholder axes without numbers, simple colored line placeholders for four methods, and minimal placeholder labels only. Do not generate real data, do not generate numerical values, do not show conclusions, do not fabricate experimental results. Use blue, orange, green, and purple line colors, clean academic style, no watermark.
```

## 不能做的事

不要让 image2.0 生成：

- “SPMPC 明显最好”的假曲线；
- 具体百分比降低；
- 具体数值表；
- 真实液面曲线；
- 实物照片替代图。

---

# 可选图 A：低阶晃液模型与增强状态示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号，但它可以作为 Fig. 2 的局部元素，或者在后续扩展为方法章的补充图。

## 用途

帮助读者理解为什么要把 `eta` 和 `eta_dot` 放进状态。

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

# 可选图 B：滚动时域预测与第一帧执行示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号，但它可以作为 Fig. 2 的局部元素，或者用于汇报 PPT。

## 用途

说明 SPMPC 不是离线轨迹生成，而是在线重复优化，只执行第一帧控制。

## Prompt

```text
A clean academic diagram illustrating receding horizon control for a mobile robot carrying an open liquid container, white background, IEEE-style figure.

Show a horizontal time axis. Mark the current time at the left and a prediction horizon extending to the right. Along the horizon, show a sequence of small robot icons following a predicted curved trajectory. On each robot, show a small open container with a changing blue liquid wave, indicating predicted slosh state propagation.

Highlight the first control action with a bold arrow and show that only this first action is applied. Then show the horizon shifting forward and the optimization repeating at the next time step.

Use a clean blue and gray palette, thin lines, simple arrows, minimal placeholder labels, no equations, no detailed text, no fake data plots, no watermark.
```

---

# 可选图 C：MPCC 路径进度、轮廓误差和滞后误差示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号。若篇幅紧张，可以不放；若方法章读者需要几何解释，可作为补充图。

## 注意

这张图对几何准确性要求较高，最终更推荐手动画 SVG、TikZ 或 Python 矢量图。image2.0 可以先生成构图参考，但不要直接作为最终图。

## Prompt

```text
A clean geometric schematic for a model predictive contouring control paper, white background.

Show a smooth reference path curve and a mobile robot point near the path. Show the closest or progress-related reference point on the path. Draw a tangent direction arrow along the path and a normal direction arrow perpendicular to the path. Show two error components from the robot position to the path reference frame: one lateral contour error and one longitudinal lag error. Show path progress along the curve with a small arrow.

Use precise-looking geometry, thin lines, blue reference path, gray robot point, red and orange error arrows, minimal placeholder labels only. Avoid equations, avoid long text, avoid decorative elements, no fake data, no watermark.
```

---

# 可选图 D：图形摘要 / 汇报主视觉草图

## 当前状态

不建议放入 RAL 正文主线，适合汇报 PPT、论文首页草图或图形摘要。

## Prompt

```text
A clean graphical abstract for a robotics control paper, white background.

Show a mobile robot carrying an open liquid container moving along a curved path. On one side, show a conventional local planner icon that only considers robot motion. On the other side, show the proposed slosh-aware predictive local planner that includes robot state, path progress, and liquid slosh modal state. Show a receding horizon trajectory ahead of the robot and a small predicted liquid wave evolution inside the container.

The visual message should be: liquid sloshing is a dynamic state prediction problem, not merely a trajectory smoothing problem.

Use elegant academic vector illustration, blue liquid, gray robot, clean arrows, minimal placeholder labels, no long text, no equations, no fake data, no logos, no watermark.
```

---

# 推荐优先生成顺序

当前正文最需要优先生成：

1. **Fig. 1 任务与方法定位示意图**  
   用于引言，帮助读者快速理解本文的层级缺口和 novelty。

2. **Fig. 2 SPMPC 增强状态与 OCP 结构图**  
   用于方法章，是当前最重要的结构图。

3. **Fig. 3 实验平台与外部液面评价流程图**  
   用于实验章，说明真实液面评价和内部代理量的区别。

不建议马上用 image2.0 生成 Fig. 4 的最终图。Fig. 4 应等待真实实验数据，用 Python/MATLAB/LaTeX 绘图生成。

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
- 外部 RGB/液面观测模块；
- 内部 slosh proxy 与真实液面指标的区分；
- 滚动时域反馈箭头。

文字、公式和变量标签建议后期手动加，避免图片生成模型产生乱码。
