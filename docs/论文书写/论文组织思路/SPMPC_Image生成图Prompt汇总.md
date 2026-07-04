# SPMPC 面向 RAL 审美的 Image 2.0 论文图 Prompt 汇总

本文档用于生成当前 `main_twocolumn` 投稿化初稿中的论文示意图。目标不是生成 SVG，而是用 Image 2.0 生成符合 RAL / IEEE 双栏论文审美的高质量科研插图底图或成图素材。

当前正文中的主要图位是：

- **Fig. 1：任务与方法定位示意图**，放在引言；
- **Fig. 2：增强状态与 OCP 结构图**，放在方法章；
- **Fig. 3：实验平台与评价流程图**，放在实验章；
- **Fig. 4：内部消融结果分析图占位**，放在实验章；当前保留占位，最终必须由真实数据生成。

Image 2.0 优先用于生成 Fig. 1、Fig. 2、Fig. 3 的科研风格底图。Fig. 4 只能生成“结果图版式占位”，不能生成最终实验曲线、数值或结论。最终实验结果必须由真实仿真、实物或外部液面观测数据绘制。

---

## 0. RAL / IEEE 论文图通用审美要求

### 0.1 总体风格

生成图像时应强调以下关键词：

```text
IEEE RA-L paper style, clean scientific diagram, minimal academic illustration, white background, flat vector-like raster illustration, thin consistent strokes, restrained blue-gray color palette, high readability in a two-column robotics paper, no decorative elements.
```

这里的 “vector-like raster illustration” 表示图像看起来像矢量图，但输出可以是 PNG/JPG，不要求 SVG。

### 0.2 适合 RAL 双栏论文的构图

建议优先生成两类尺寸感：

1. **单栏图**：适合放在一栏内，画面要非常简洁，避免超过 5–7 个主要模块；
2. **跨栏图**：适合 Fig. 1 或 Fig. 2 这类方法总览图，横向 16:9 或 2:1 构图，后续可根据版面决定是否用 `figure*`。

如果不确定，就使用下面这句：

```text
Design it as a compact figure that remains readable when reduced to one column width in an IEEE RA-L two-column paper.
```

### 0.3 颜色建议

推荐固定一套颜色：

- 主色：深蓝 / 科研蓝，用于路径、主流程箭头、SPMPC 模块；
- 辅助色：浅蓝，用于液体和晃液状态；
- 灰色：用于普通模块、基线方法、平台边界；
- 橙色：只用于突出“液体激励 / 关键差异 / warning”，不要大面积使用。

建议 prompt 中写：

```text
Use a restrained palette: deep academic blue, light cyan for liquid, neutral gray for baseline modules, and a small amount of muted orange for emphasis.
```

### 0.4 字体和文字

Image 2.0 容易生成乱码或不稳定小字，所以正文图建议：

- 图中只保留极少量英文短标签；
- 不要让模型生成公式；
- 不要生成长句；
- 不要生成中文正文；
- 变量、公式、详细标签后续在 LaTeX 或图片编辑软件里补。

推荐统一加入：

```text
Use only short clean English placeholder labels, no paragraphs, no mathematical equations, no tiny text. Leave enough whitespace so final labels can be added manually later.
```

### 0.5 通用负面 prompt

每次生成都建议附加：

```text
No fake experimental data, no numerical values, no fabricated plots, no performance claims, no equations, no dense text, no Chinese characters, no watermark, no logo, no decorative background, no photorealistic rendering, no 3D glossy style, no cartoon style, no clutter, no excessive shadows, no gradient-heavy background, no handwritten text, no pseudo-random labels.
```

中文含义：不要生成假实验数据、假数值、假曲线、假结论；不要生成公式和大段文字；不要中文乱码；不要水印、标志、复杂背景、3D 炫光、卡通风、手写字或杂乱小标签。

---

# Fig. 1 任务与方法定位示意图

## 正文位置

引言。当前正文图名：

```text
Fig. 1：任务与方法定位示意图
普通局部规划 / 防晃方法 / SPMPC 在线局部规划
```

## 图的任务

这张图负责在引言中建立论文缺口：普通局部规划器和已有防晃方法分别解决了相关问题，但没有在“标准移动底盘在线局部规划层”把液体动态状态作为预测状态使用。

## 推荐构图

推荐三栏横向构图：

1. 左栏：普通局部规划器；
2. 中栏：已有防晃方法；
3. 右栏：SPMPC 在线局部规划器。

右栏应该视觉上最清晰、最聚焦，但不要夸张成宣传海报。

## 主 Prompt：RAL 风格三栏定位图

```text
Create a clean scientific diagram for an IEEE RA-L robotics paper, white background, compact three-panel horizontal layout, flat vector-like raster illustration, high readability when reduced to one-column or double-column width.

Panel 1, labeled “Local Planner”: show a simple wheeled mobile robot carrying an open liquid container and following a reference path. Show inputs as “Path” and “Robot State”, and output as “Velocity Command”. The liquid container is visible but not connected to the planner, indicating that the liquid state is not modeled.

Panel 2, labeled “Anti-sloshing Methods”: show anti-sloshing modules such as speed profile, input shaping, offline trajectory, or tracking controller around a liquid model icon. Make it clear that these methods use liquid knowledge but are placed outside the standard online local planning layer.

Panel 3, labeled “SPMPC”: show robot state, path progress, and low-order slosh state entering one receding-horizon MPCC local planner block. Show the output as a standard mobile-base velocity command. Use a slightly stronger academic blue outline for this panel.

Use a restrained palette: deep academic blue, light cyan for liquid, neutral gray for baseline modules, and a small amount of muted orange for emphasis. Use thin consistent strokes, clean arrows, rounded rectangles, balanced spacing, and large readable labels.

Use only short clean English placeholder labels, no paragraphs, no mathematical equations, no tiny text. Leave enough whitespace for final labels to be adjusted manually. No fake experimental data, no numerical values, no watermark, no logos, no photorealistic rendering, no cartoon style, no 3D glossy style, no clutter.
```

## 备选 Prompt：更像论文综述图的版本

```text
Design an elegant RA-L style overview figure showing the research gap for open-liquid transportation by mobile robots. Use a clean white background and a three-column taxonomy layout.

Column A: conventional online local planning, with path tracking and velocity command, but no liquid state. Column B: anti-sloshing planning and control, with liquid model but offline planning, speed profiling, input shaping, or tracking control. Column C: proposed SPMPC, integrating robot state, path progress, and slosh modal state into an online MPCC local planner.

Make the diagram look like a polished figure from an IEEE Robotics and Automation Letters paper: minimal, precise, flat, no decorative illustration, no dense text, no equations, no fake data. Use blue-gray colors, light cyan liquid, thin arrows, and clean module boxes. The rightmost SPMPC column should be visually emphasized but still academic and restrained.
```

## 需要避免

不要画成真实照片、不要画成炫酷机器人海报、不要把化学实验室画得很复杂。Fig. 1 的重点是方法定位，不是场景宣传。

---

# Fig. 2 SPMPC 增强状态与 OCP 结构图

## 正文位置

方法章。当前正文图名：

```text
SPMPC 增强状态与 OCP 结构图
机器人状态 + 路径进度 + 晃液状态 → 增强动力学与 MPCC OCP → 第一帧底盘命令
```

## 图的任务

这是方法章最重要的图。它要让读者一眼看出：SPMPC 不是单独的液体估计器，也不是离线轨迹生成器，而是把低阶晃液状态并入移动底盘在线 MPCC 局部规划 OCP。

## 推荐构图

推荐左到右流程：

```text
Inputs → Augmented Dynamics → MPCC OCP → First command → Receding-horizon loop
```

其中 Inputs 包括：

- Robot state；
- Reference path / path progress；
- Previous command；
- Slosh modal state。

MPCC OCP 内部包含四类目标：

- Path tracking；
- Progress；
- Control smoothness；
- Predicted slosh response。

## 主 Prompt：RAL 方法结构图

```text
Create a polished technical block diagram for an IEEE RA-L robotics paper, white background, flat vector-like raster style, clean left-to-right control architecture, readable in a two-column paper.

On the left, show four input blocks with simple icons: “Robot State”, “Reference Path”, “Previous Command”, and “Slosh State”. Use a small mobile robot icon, a curved path icon, a command arrow icon, and a small open container with a cyan sloshing wave.

In the center-left, show an “Augmented Dynamics” block. Inside it, visually combine a mobile base model, path progress, and a low-order slosh modal model. The slosh model should be represented by a minimal oscillator or mass-spring-damper icon connected to an open liquid container, not by complex equations.

In the center-right, show an “MPCC OCP” block. Inside this block, show four compact objective icons: path tracking, progress reward, control smoothness, and predicted slosh response. Use small icons, not long text.

On the right, show “First Command” going to a mobile base as “/cmd_vel”, and a clean curved feedback arrow returning to the next planning cycle, indicating receding-horizon execution.

Use deep academic blue for the main SPMPC flow, light cyan for liquid, neutral gray for auxiliary modules, and a tiny amount of muted orange for the slosh excitation arrow. Keep strokes thin and consistent. Use generous whitespace, aligned boxes, simple arrows, and balanced proportions.

Use only short English labels. No equations, no dense text, no fake data, no numerical plots, no watermark, no logo, no photorealistic robot, no cartoon style, no 3D glossy blocks, no decorative background.
```

## 备选 Prompt：更强调“动态记忆”的版本

```text
Draw a clean RA-L style method diagram explaining that liquid sloshing is a dynamic state prediction problem. White background, academic blue-gray palette, vector-like raster illustration.

Show a receding-horizon MPCC local planner for a wheeled mobile robot carrying an open liquid container. The planner receives robot state, path progress, previous command, and slosh modal state. The slosh modal state is shown as a small memory state with a cyan wave and an oscillator icon. The augmented state flows into an optimal control problem with costs for tracking, progress, smoothness, and predicted slosh response. Only the first optimized command is sent to the robot, then the loop repeats.

Make it look like a precise control-system figure in IEEE Robotics and Automation Letters: aligned blocks, thin arrows, minimal labels, no equations, no paragraphs, no fake data, no decorative 3D effects, no clutter.
```

## 需要避免

不要把 Fig. 2 画成复杂软件架构图。模块数量过多会导致双栏缩小后不可读。它只需要表达“增强状态 + OCP + 第一帧执行”。

---

# Fig. 3 实验平台与外部液面评价流程图

## 正文位置

实验章。当前正文图名：

```text
Fig. 3：实验平台与评价流程
底盘、开口容器、参考路径、RGB/外部液面观测、日志记录
```

## 图的任务

这张图负责说明实验和评价边界：真实液面结论来自外部观测；内部 `/spmpc/slosh_height` 只是模型代理量或诊断量。

## 推荐构图

推荐分成左右两部分：

1. 左侧：移动底盘、开口容器、参考路径、外部相机；
2. 右侧：数据记录和评价指标流程。

注意这张图不是实验结果图，不要出现真实曲线和具体数值。

## 主 Prompt：RAL 实验流程图

```text
Create a clean experimental setup and evaluation pipeline figure for an IEEE RA-L robotics paper, white background, flat vector-like raster illustration, minimal and precise.

Left side: show a simple wheeled mobile robot carrying an open liquid container moving along a smooth reference path from start to goal on a flat indoor floor. Use a very simple indoor lab or corridor outline, not a detailed scene. The liquid should be light cyan with a small sloshing surface.

Show an external RGB camera or external liquid-level observation module looking at the container. The camera should clearly be outside the planner loop, indicating independent liquid-surface evaluation.

Right side: show a clean data logging and evaluation pipeline with four blocks: “Planner Logs”, “Internal Slosh Proxy”, “External Liquid Observation”, and “Evaluation Metrics”. The final metrics include liquid response, task completion, path tracking, smoothness, and solver time. Use icons and short labels only.

Visually distinguish “Internal Slosh Proxy” from “External Liquid Observation”. Use gray for internal logs, blue for planner flow, and cyan for true liquid observation. Make the external observation path visually important but not flashy.

IEEE RA-L style: thin consistent strokes, aligned blocks, restrained blue-gray palette, large readable labels, no clutter, no decorative background, no photorealistic rendering, no 3D glossy style, no fake data, no numerical plots, no performance claims, no equations, no watermark.
```

## 备选 Prompt：更强调“内部代理量 vs 真实观测”的版本

```text
Design a minimal RA-L style figure showing how open-liquid transport experiments are evaluated. White background, flat scientific illustration, blue-gray palette.

A mobile robot with an open liquid container follows a reference path. The planner produces velocity commands and internal logs, including an internal slosh proxy. Separately, an external RGB camera observes the liquid surface. Both streams go to an evaluation module, but the diagram clearly indicates that real liquid conclusions come from external observation, not from the internal model proxy.

Use clean arrows, large readable labels, subtle cyan liquid, simple robot icon, simple camera icon, and compact metric boxes. No fake plots, no numbers, no dense text, no equations, no decorative lab details, no watermark, no logos.
```

## 需要避免

不要画成真实实验照片；不要出现具体液面曲线；不要让内部代理量看起来就是“真实液面高度”。

---

# Fig. 4 内部消融结果分析图占位

## 正文位置

实验章。当前正文图名：

```text
Fig. 4：内部消融结果分析图占位
B0 / B_smooth / B_slosh / B_ours 的轨迹、速度、液面响应和求解时间；真实结果图后续由实验数据生成
```

## 重要说明

Fig. 4 是论文核心证据图，但当前实验没有做完，所以正文中保留结果占位是合理的。Image 2.0 只能帮助生成版式占位或多面板布局参考，不能生成最终实验曲线、具体数值或“SPMPC 更好”的假结论。

## 推荐最终结构

最终 Fig. 4 可以采用 2×2 或 2×3 多面板：

- Panel A：轨迹或路径误差；
- Panel B：速度 / 加速度 / 控制平滑性；
- Panel C：真实液面响应，例如 RGB 最大 LCR 或液面高度；
- Panel D：求解时间、任务完成率或统计摘要。

## 仅用于占位版式的 Prompt

```text
Create a clean placeholder layout for an experimental ablation results figure in an IEEE RA-L robotics paper. White background, flat vector-like raster style, multi-panel scientific layout.

Create a 2 by 2 panel figure with four empty result panels. Panel A is reserved for trajectory and path error comparison. Panel B is reserved for velocity, acceleration, or control smoothness. Panel C is reserved for real liquid-surface response from external observation. Panel D is reserved for solver time and task completion summary.

Use empty axes or faint gray axis frames only. Use very light placeholder curves without numerical values, or use abstract dashed placeholders that do not imply any method is better. Add a small legend area with four method names: B0, B_smooth, B_slosh, B_ours. Do not show real values, percentages, performance claims, or conclusions.

Make it look like a polished RA-L result figure template: clean panel boundaries, consistent spacing, readable panel letters, restrained colors, no clutter, no fake experimental data, no numerical tick labels, no fabricated trends, no watermark, no logo.
```

## 更安全的占位 Prompt：不画曲线，只画版式框

```text
Create a minimal IEEE RA-L style multi-panel result figure template, white background, no data. Four panels arranged in a 2 by 2 grid: trajectory comparison, control profile, external liquid response, solver-time summary. Use only gray empty axes, panel labels A-D, and small method legend placeholders for B0, B_smooth, B_slosh, and B_ours. Do not draw curves, do not draw numerical values, do not imply performance trends. Clean academic layout, thin strokes, high readability, no watermark.
```

## 不能做的事

不要让 Image 2.0 生成：

- “SPMPC 明显最好”的假曲线；
- 具体百分比降低；
- 具体数值表；
- 真实液面曲线；
- 实物照片替代图；
- 看起来像已完成实验的统计结论。

---

# 可选图 A：低阶晃液模型与增强状态示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号，但它可以作为 Fig. 2 的局部元素，或者后续扩展为方法章补充图。

## Prompt

```text
Create a clean RA-L style scientific illustration of a low-order slosh model for a mobile robot carrying an open liquid container. White background, flat vector-like raster style, restrained blue-gray palette.

Show a wheeled mobile base carrying a simple open container with light cyan liquid. Beside the container, show a minimal second-order modal oscillator icon, such as a mass-spring-damper or pendulum-like symbol, representing low-order liquid slosh dynamics. Show a forward acceleration arrow and a turning-induced lateral excitation arrow. Indicate that the liquid state has memory by showing two or three faint wave states over time.

Keep the figure minimal, precise, and readable in an IEEE RA-L paper. Use thin strokes, clean arrows, very short English labels, no equations, no dense text, no fake data, no photorealistic rendering, no cartoon style, no watermark.
```

---

# 可选图 B：滚动时域预测与第一帧执行示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号，但它可以作为 Fig. 2 的局部元素，或者用于汇报材料。

## Prompt

```text
Create a clean RA-L style receding-horizon control diagram for a mobile robot carrying an open liquid container. White background, flat vector-like raster style, blue-gray palette.

Show a horizontal prediction horizon with several future robot poses along a curved path. Each future pose carries a small open container with a slightly different cyan liquid wave, indicating predicted slosh state propagation. Highlight only the first control action as applied to the robot. Then show the horizon shifting forward and the optimization repeating.

Use thin arrows, minimal labels, large readable elements, no equations, no dense text, no fake plots, no numerical values, no watermark, no decorative background.
```

---

# 可选图 C：MPCC 路径进度、轮廓误差和滞后误差示意图

## 当前状态

当前投稿化初稿没有单独为这张图留正式图号。若方法章读者需要几何解释，可作为补充图。

## Prompt

```text
Create a precise geometric schematic for model predictive contouring control in an IEEE RA-L paper. White background, flat vector-like raster style, minimal geometry.

Show a smooth reference path curve and a mobile robot point near the path. Show a progress point on the path, a tangent direction, and a normal direction. Show two error components: lateral contour error and longitudinal lag error. Show path progress with a small arrow along the curve.

Use blue for the reference path, gray for the robot point and path frame, muted orange and red for the two error arrows. Keep labels very short and readable. No equations, no dense text, no fake data, no decorative elements, no watermark.
```

---

# 可选图 D：图形摘要 / 汇报主视觉草图

## 当前状态

不建议放入 RAL 正文主线，适合汇报 PPT、论文首页草图或图形摘要。

## Prompt

```text
Create an elegant graphical abstract for a robotics control paper, but keep it restrained and academic like an IEEE RA-L figure. White background, flat vector-like raster illustration, blue-gray palette.

Show a mobile robot carrying an open liquid container moving along a curved path. On one side, show conventional local planning that only considers robot motion. On the other side, show the proposed slosh-aware predictive local planner that includes robot state, path progress, and liquid slosh modal state. Show a short receding horizon trajectory ahead of the robot and a small predicted liquid wave evolution inside the container.

The visual message is: liquid sloshing is a dynamic state prediction problem, not merely a trajectory smoothing problem. Use minimal labels, clean arrows, no equations, no fake data, no decorative background, no 3D glossy style, no watermark.
```

---

# 推荐优先生成顺序

当前最值得用 Image 2.0 生成的是：

1. **Fig. 2：SPMPC 增强状态与 OCP 结构图**  
   方法章最关键，现有 TikZ 图偏机械，应该优先替换成更像 RAL 的科研示意图。

2. **Fig. 1：任务与方法定位示意图**  
   引言中的 novelty 图，决定读者第一印象。

3. **Fig. 3：实验平台与外部液面评价流程图**  
   用于强调真实液面评价和内部代理量的边界。

4. **Fig. 4：内部消融结果分析图占位**  
   只生成版式占位，不生成最终结果。

---

# 生成后筛选标准

生成图片后建议按以下标准筛选：

- 缩小到一栏宽度后仍能看清主结构；
- 模块不超过 7 个主要视觉单元；
- 没有明显 AI 乱字、伪公式或乱码；
- 没有假数据、假曲线或暗示性结论；
- 颜色克制，不像宣传海报；
- 白底、线条清楚、箭头方向明确；
- 机器人、容器和液面元素足够简洁；
- 图中标签可以后期替换，不影响主体结构。

如果一张图结构好但文字差，可以保留为底图，后续裁切掉文字或覆盖标签；如果结构混乱，即使颜色漂亮也不要用于论文正文。
