# SPMPC 面向 RAL 审美的 Image 2.0 论文图 Prompt 汇总

本文档用于为 `docs/论文书写/草稿/spmpc_paper_cn` 中的中文 SPMPC 初稿生成论文图底图或版式参考。目标不是让 Image 2.0 一次性生成可直接投稿的最终图，而是生成符合 IEEE RA-L / ICRA 论文审美的**干净科研图底稿**：结构、留白、箭头、图标和色彩先到位，正式文字、公式、变量名和实验曲线后续由 LaTeX、SVG、矢量软件或真实数据脚本覆盖。

当前正文主要图位：

- **Fig. 1：任务与方法定位示意图**，放在引言；
- **Fig. 2：SPMPC 增强状态与 OCP 结构图**，放在方法章；
- **Fig. 3：实验平台与评价流程图**，放在实验章；
- **Fig. 4：内部消融结果分析图占位**，放在实验章；当前只保留占位，最终必须由真实数据生成。

建议生成文件先放入论文图目录：

```text
docs/论文书写/草稿/spmpc_paper_cn/figures/
```

建议候选文件名：

```text
fig1_system_overview_image2.png
fig2_method_structure_image2.png
fig3_experiment_pipeline_image2.png
fig4_ablation_layout_placeholder_image2.png
```

> **当前建议：优先尝试 Fig. 2，但优先生成“无文字/少文字底图”。** Image 模型生成小字、公式和变量名的稳定性通常不够好；如果结构不错但文字不好，应把它当作底图，后续手工覆盖标签。若结构本身混乱、模块过多、像宣传海报或出现假数据，则不要进入论文仓库。

---

## 0. 全局原则

### 0.1 论文图定位

SPMPC 的论文主线是：

```text
Liquid sloshing is not merely a trajectory smoothing problem, but a dynamic-memory state prediction problem.
SPMPC embeds a low-order slosh state into an online MPCC local-planning layer for a standard mobile base.
```

图像必须服务这个主线，而不是画成复杂软件架构图、实验照片、炫酷机器人海报或结果宣传图。

### 0.2 Image 2.0 的使用边界

Image 2.0 适合生成：

- 简洁的科研示意图底图；
- 机器人、容器、液体、相机、路径、模块框、箭头等视觉元素；
- RA-L 风格的留白、版式、图标和颜色参考；
- Fig. 4 的空版式占位框。

Image 2.0 **不适合**生成：

- 最终实验曲线；
- 具体数值、百分比或性能排名；
- 真实液面曲线；
- Ferrari-style 模型—外部液面一致性曲线；
- 复杂公式、小号变量名和长句；
- 需要严格对齐的最终可投稿矢量图。

新增的模型—外部液面一致性评价只能在真实外部液面数据可用后绘制。不能用 Image 2.0 生成假曲线、假对齐趋势、假 RMSE / correlation / bias 结果。

### 0.3 通用审美 prompt

每次生成建议加入：

```text
IEEE Robotics and Automation Letters style, clean scientific diagram, minimal academic illustration, white background, flat vector-like raster illustration, thin consistent strokes, restrained blue-gray color palette, high readability in a two-column robotics paper, balanced whitespace, no decorative elements.
```

配色建议：

```text
Use a restrained palette: deep academic blue for the proposed SPMPC flow, light cyan for liquid and slosh state, neutral gray for auxiliary modules and baselines, and a very small amount of muted orange only for slosh excitation or key contrast.
```

### 0.4 文字策略

优先采用两阶段制图：

1. Image 2.0 生成**无文字或少文字底图**；
2. 后续用 LaTeX / SVG / Illustrator / Inkscape 覆盖正式标签。

通用文字约束：

```text
Prefer a no-text diagram. If text is necessary, use only very short English placeholder labels with large font. No paragraphs, no equations, no tiny text, no Chinese characters, no pseudo-random labels. Leave clear empty label zones so final labels can be added manually later.
```

### 0.5 通用负面 prompt

每次生成都建议附加：

```text
No fake experimental data, no numerical values, no fabricated plots, no performance claims, no equations, no dense text, no Chinese characters, no watermark, no logo, no decorative background, no photorealistic rendering, no glossy 3D style, no cartoon style, no clutter, no excessive shadows, no gradient-heavy background, no handwritten text, no pseudo-random labels.
```

---

# Fig. 1 任务与方法定位示意图

## 正文位置

引言。当前正文图名：

```text
任务与方法定位示意图：普通局部规划 / 防晃方法 / SPMPC 在线局部规划
```

## 图的任务

Fig. 1 用于建立论文缺口：普通移动机器人局部规划器处在正确的在线部署层，但通常不传播液体状态；已有防晃方法利用液体模型，但多数位于离线轨迹、速度剖面、输入整形、跟踪控制或特殊平台层；SPMPC 的定位是在标准轮式移动底盘在线局部规划层引入低阶晃液状态。

## 推荐构图

三栏横向构图：

1. 左栏：Conventional Local Planner；
2. 中栏：Anti-sloshing Methods；
3. 右栏：SPMPC Online Local Planner。

右栏可以用更清晰的蓝色边框强调，但不要像宣传海报。

## 优先 Prompt：无文字/少文字底图

```text
Create a clean IEEE RA-L style research-gap overview diagram, white background, flat vector-like raster illustration, compact three-panel horizontal layout, readable in a two-column robotics paper.

Panel A: a conventional local planner for a wheeled mobile robot carrying an open liquid container. Show a reference path, robot state, and velocity-command arrow. The liquid container is visible, but the liquid state is not connected to the planner.

Panel B: anti-sloshing planning and control methods. Show a liquid model icon connected to offline trajectory, speed profile, input shaping, or tracking-control blocks. Make this panel visually different from an online local planner.

Panel C: the proposed SPMPC online local planner. Show robot state, path progress, and a low-order slosh state entering one receding-horizon MPCC local-planner block, with a standard mobile-base velocity command as output. This panel should be visually clearest and outlined in academic blue.

Use thin arrows, rounded rectangles, simple icons, balanced spacing, deep academic blue, neutral gray, and light cyan liquid. Prefer no text; if labels are used, keep them as very short placeholders only. Leave whitespace for final labels to be added manually.

No fake experimental data, no plots, no numerical values, no equations, no dense text, no Chinese characters, no photorealistic robot, no cartoon style, no glossy 3D style, no decorative background, no watermark, no logo.
```

## 备选 Prompt：带少量英文标签

```text
Design an elegant RA-L style taxonomy figure showing three method categories for open-liquid transport by mobile robots: Local Planner, Anti-sloshing Methods, and SPMPC. Use a clean white background and a restrained blue-gray palette.

Local Planner: online path tracking and velocity command, but no liquid-state feedback. Anti-sloshing Methods: liquid model with offline trajectory, speed profile, input shaping, or tracking controller. SPMPC: robot state, path progress, and slosh modal state integrated into one online MPCC local planner.

Use simple module boxes, thin arrows, light cyan liquid icons, and large readable labels only. No equations, no paragraphs, no fake data, no numerical plots, no decorative details.
```

## 避免

不要画成真实照片、复杂化学实验室、炫酷机器人海报或“SPMPC 全面胜出”的宣传图。Fig. 1 的重点是**层级定位**。

---

# Fig. 2 SPMPC 增强状态与 OCP 结构图

## 正文位置

方法章。当前正文图名：

```text
SPMPC 增强状态与 OCP 结构图：机器人状态 + 路径进度 + 晃液状态 → 增强动力学与 MPCC OCP → 第一帧底盘命令
```

## 图的任务

Fig. 2 是方法章核心图。它要表达：SPMPC 不是单独的液体估计器，也不是离线轨迹生成器，而是将低阶晃液状态作为增强状态的一部分并入在线 MPCC 局部规划 OCP；只执行滚动时域优化的第一帧底盘命令。

## 推荐构图

左到右流程：

```text
Inputs → Augmented State / Dynamics → MPCC OCP → First Command → Receding-horizon Loop
```

核心元素：

- 输入：robot state、reference path / path progress、previous command、slosh modal state；
- 增强状态/动力学：mobile-base model + path progress + low-order slosh memory；
- OCP 目标：tracking、progress、smoothness、predicted slosh response；
- 输出：first command / cmd_vel；
- 反馈：receding-horizon loop。

## 优先 Prompt：无文字底图版

```text
Create a clean no-text technical block diagram for an IEEE RA-L robotics paper, white background, flat vector-like raster style, left-to-right receding-horizon control architecture, highly readable in a two-column paper.

Use five visual regions from left to right:
1) input icons: mobile robot state, reference path, previous command arrow, and open container with cyan sloshing liquid;
2) augmented-state / augmented-dynamics region: visually merge a mobile-base model, path-progress curve, and a low-order slosh-memory icon such as a mass-spring-damper or modal oscillator connected to a container;
3) MPCC optimal-control region: show a prediction horizon strip and four small objective icons for path tracking, progress, smooth control, and predicted slosh response;
4) first-command region: highlight only the first control action going to a mobile base;
5) receding-horizon feedback loop returning to the next planning cycle.

Use aligned rounded rectangles, thin consistent arrows, generous whitespace, restrained academic blue for the main SPMPC flow, light cyan for liquid, neutral gray for auxiliary elements, and a tiny muted-orange arrow only for slosh excitation or dynamic-memory coupling.

Prefer no text at all. Leave clean empty space above or inside each region for final LaTeX/SVG labels to be added manually. No equations, no numerical plots, no fake data, no tiny text, no Chinese characters, no pseudo-random labels, no photorealistic robot, no cartoon style, no glossy 3D blocks, no decorative background, no watermark.
```

## 备选 Prompt：少文字结构图

```text
Create a polished technical architecture diagram for an IEEE RA-L robotics paper, white background, flat vector-like raster style, clean left-to-right flow.

On the left, show four compact input blocks labeled only with short placeholders: Robot, Path, Command, Slosh. Use simple icons: a wheeled mobile robot, a curved path, an arrow command, and an open container with a cyan sloshing wave.

In the middle, show an Augmented Dynamics block that combines a mobile-base model, path progress, and a low-order slosh modal model. Represent the slosh model by a minimal oscillator or mass-spring-damper connected to an open liquid container, not by equations.

Next, show an MPCC OCP block with a short prediction horizon and four small objective icons: Tracking, Progress, Smoothness, Slosh. Avoid long labels.

On the right, show First Command going to a mobile base, then a clean curved feedback arrow returning to the next planning cycle.

Use academic blue for the main flow, light cyan for liquid, neutral gray for secondary modules, and a small muted-orange slosh-excitation cue. Use large readable placeholder labels only. No equations, no dense text, no fake data, no numerical plots, no watermark, no logo, no photorealistic rendering, no cartoon style, no 3D glossy style, no clutter.
```

## 避免

- 不要把 Fig. 2 画成 ROS 软件架构图；
- 不要出现 10 个以上模块；
- 不要画复杂三维机器人或炫光背景；
- 不要把 `/spmpc/slosh_height` 画成真实液面测量；
- 不要生成公式、乱码、小字或假实验曲线。

---

# Fig. 3 实验平台与外部液面评价流程图

## 正文位置

实验章。当前正文图名：

```text
实验平台与评价流程：底盘、开口容器、参考路径、RGB/外部液面观测、日志记录、评价指标
```

## 图的任务

Fig. 3 用于说明实验和评价边界：真实液面结论来自外部观测；内部 `/spmpc/slosh_height` 只是模型预测或诊断代理量。五章重构后，实验章新增模型—外部液面一致性评价，因此 Fig. 3 可以显示 Internal Slosh Proxy 与 External Liquid Observation 共同进入 Fidelity / Metrics 模块，但不能画成已经有真实曲线或具体数值。

## 推荐构图

左右结构：

1. 左侧：移动底盘、开口容器、参考路径、外部相机或液位观测模块；
2. 右侧：Planner Logs、Internal Slosh Proxy、External Liquid Observation、Evaluation Metrics / Fidelity Check。

## 优先 Prompt：无假数据实验流程图

```text
Create a clean experimental setup and evaluation-pipeline figure for an IEEE RA-L robotics paper, white background, flat vector-like raster illustration, minimal and precise.

Left side: show a simple wheeled mobile robot carrying an open liquid container along a smooth reference path from start to goal on a flat indoor floor. Keep the environment abstract and minimal, like a lab or corridor outline, not a detailed scene. The liquid is light cyan with a small sloshing surface.

Show an external RGB camera or external liquid-level observation module looking at the container. The camera must be outside the planner loop, indicating independent liquid-surface evaluation.

Right side: show a clean logging and evaluation pipeline. Use separate streams for planner logs, internal slosh proxy, and external liquid observation. The internal proxy stream should be visually distinct from the external observation stream. Both can feed an evaluation or fidelity-check module, which then outputs categories such as liquid response, model-observation fidelity, task completion, path tracking, smoothness, and solver time.

Use blue for planner flow, gray for internal logs/proxy, and cyan for external liquid observation. Keep labels very short or omit labels and leave space for manual labels. Use thin strokes, aligned boxes, and clear arrows.

No fake plots, no numerical values, no experimental curves, no performance claims, no equations, no dense text, no Chinese characters, no photorealistic rendering, no decorative lab details, no watermark, no logo.
```

## 备选 Prompt：强调内部代理量 vs 真实观测

```text
Design a minimal RA-L style figure showing how open-liquid transport experiments are evaluated. White background, flat scientific illustration, blue-gray palette.

A mobile robot with an open liquid container follows a reference path. The planner produces velocity commands and internal logs, including an internal slosh proxy. Separately, an external RGB camera observes the liquid surface. Both streams go to an evaluation module, but the diagram clearly indicates that real liquid conclusions come from external observation, not from the internal model proxy.

Use clean arrows, large readable placeholder labels, subtle cyan liquid, a simple robot icon, a simple camera icon, and compact metric boxes. No fake plots, no numbers, no dense text, no equations, no decorative lab details, no watermark, no logos.
```

## 避免

不要出现具体液面曲线、具体一致性数值、真实结果趋势或“模型已经验证准确”的暗示。Fig. 3 是**评价流程图**，不是实验结果图。

---

# Fig. 4 内部消融结果分析图占位

## 正文位置

实验章。当前正文图名：

```text
内部消融结果分析图占位：B0 / B_smooth / B_slosh / B_ours 的轨迹、控制、真实液面响应和求解时间
```

## 重要说明

Fig. 4 是最终证据图，但当前实验还没有完成。因此，Image 2.0 只能生成**版式占位**，不能生成曲线、柱状图、数值、百分比或“SPMPC 更好”的趋势。最终 Fig. 4 必须由真实仿真、实物或外部液面观测数据绘制。

## 推荐最终结构

最终可采用 2×2 或 2×3 多面板：

- Panel A：轨迹或路径误差；
- Panel B：速度 / 加速度 / 控制平滑性；
- Panel C：真实液面响应，例如 RGB 最大 LCR 或液面高度；
- Panel D：求解时间、任务完成率或统计摘要。

## 安全 Prompt：只画空版式框

```text
Create a minimal IEEE RA-L style multi-panel result-figure template, white background, no data. Four panels arranged in a clean 2 by 2 grid. The panels are reserved for trajectory comparison, control profile, external liquid response, and solver-time / task summary.

Use only light gray empty panel frames, subtle axis boxes without tick labels, panel letters A-D, and a small legend area reserved for four method names. Do not draw curves, do not draw bars, do not draw scatter points, do not draw numerical values, and do not imply any performance trend.

Clean academic layout, thin strokes, balanced spacing, high readability in a two-column paper, no fake experimental data, no fabricated plots, no numerical tick labels, no watermark, no logo.
```

## 可选 Prompt：带占位图例但无数据

```text
Create a clean placeholder layout for an experimental ablation results figure in an IEEE RA-L robotics paper. White background, flat vector-like raster style, multi-panel scientific layout.

Create a 2 by 2 panel figure with four empty result panels. Panel A is reserved for trajectory and path error comparison. Panel B is reserved for velocity, acceleration, or control smoothness. Panel C is reserved for real liquid-surface response from external observation. Panel D is reserved for solver time and task completion summary.

Include only a small legend placeholder with method names B0, B_smooth, B_slosh, and B_ours. Use empty gray axes or blank frames only. Do not show curves, bars, values, percentages, performance claims, or conclusions.
```

## 不能做的事

不要让 Image 2.0 生成：

- “SPMPC 明显最好”的假曲线；
- 具体百分比降低；
- 具体数值表；
- 真实液面曲线；
- 模型—外部液面一致性曲线；
- 实物照片替代结果图；
- 看起来像已完成实验的统计结论。

---

# 可选图 A：低阶晃液模型与动态记忆示意图

## 当前状态

当前正文没有为该图单独保留正式图号；它适合作为 Fig. 2 的局部元素，或用于汇报材料。

## Prompt

```text
Create a clean RA-L style scientific illustration of a low-order slosh model for a mobile robot carrying an open liquid container. White background, flat vector-like raster style, restrained blue-gray palette.

Show a wheeled mobile base carrying a simple open container with light cyan liquid. Beside the container, show a minimal second-order modal oscillator icon, such as a mass-spring-damper or pendulum-like symbol, representing low-order liquid slosh dynamics. Show a forward acceleration arrow and a lateral excitation arrow. Indicate dynamic memory by showing two or three faint successive wave states over time.

Keep the figure minimal, precise, and readable in an IEEE RA-L paper. Prefer no text; if labels are used, keep them very short. No equations, no dense text, no fake data, no photorealistic rendering, no cartoon style, no watermark.
```

---

# 可选图 B：滚动时域预测与第一帧执行示意图

## 当前状态

当前正文没有单独为该图留正式图号；它可以作为 Fig. 2 的局部元素，或用于汇报材料。

## Prompt

```text
Create a clean RA-L style receding-horizon control diagram for a mobile robot carrying an open liquid container. White background, flat vector-like raster style, blue-gray palette.

Show a horizontal prediction horizon with several future robot poses along a curved path. Each future pose carries a small open container with a slightly different cyan liquid wave, indicating predicted slosh-state propagation. Highlight only the first control action as applied to the robot. Then show the horizon shifting forward and the optimization repeating.

Use thin arrows, minimal placeholder labels, large readable elements, no equations, no dense text, no fake plots, no numerical values, no watermark, no decorative background.
```

---

# 可选图 C：MPCC 路径进度、轮廓误差和滞后误差示意图

## 当前状态

当前正文没有为该图单独保留正式图号。若方法章需要几何解释，可作为补充图。

## Prompt

```text
Create a precise geometric schematic for model predictive contouring control in an IEEE RA-L paper. White background, flat vector-like raster style, minimal geometry.

Show a smooth reference path curve and a mobile robot point near the path. Show a progress point on the path, a tangent direction, and a normal direction. Show two error components: lateral contour error and longitudinal lag error. Show path progress with a small arrow along the curve.

Use blue for the reference path, gray for the robot point and path frame, muted orange and red for the two error arrows. Keep labels very short and readable. No equations, no dense text, no fake data, no decorative elements, no watermark.
```

---

# 可选图 D：图形摘要 / 汇报主视觉草图

## 当前状态

不建议放入 RA-L 正文主线；适合汇报 PPT、论文首页草图或图形摘要。

## Prompt

```text
Create an elegant graphical abstract for a robotics control paper, but keep it restrained and academic like an IEEE RA-L figure. White background, flat vector-like raster illustration, blue-gray palette.

Show a mobile robot carrying an open liquid container moving along a curved path. On one side, show conventional local planning that only considers robot motion. On the other side, show the proposed slosh-aware predictive local planner that includes robot state, path progress, and liquid slosh modal state. Show a short receding-horizon trajectory ahead of the robot and a small predicted liquid-wave evolution inside the container.

The visual message is: liquid sloshing is a dynamic state prediction problem, not merely a trajectory smoothing problem. Use minimal labels, clean arrows, no equations, no fake data, no decorative background, no 3D glossy style, no watermark.
```

---

# 推荐优先生成顺序

1. **Fig. 2：SPMPC 增强状态与 OCP 结构图**  
   方法章核心图，建议先用“无文字底图版”prompt 反复筛结构，再手工覆盖标签。

2. **Fig. 1：任务与方法定位示意图**  
   引言 novelty 图，决定读者第一印象；重点是层级差异，不是结果优越性。

3. **Fig. 3：实验平台与外部液面评价流程图**  
   用于强调真实液面评价和内部代理量边界。

4. **Fig. 4：内部消融结果分析图占位**  
   只生成空版式占位，不生成最终结果。

---

# 生成后筛选标准

生成图片后按以下标准筛选：

- 缩小到 IEEE 双栏单栏宽度后仍能看清主结构；
- 主视觉模块不超过 5–7 个；
- 没有明显 AI 乱字、伪公式、乱码或多余标签；
- 没有假数据、假曲线、假柱状图或暗示性结论；
- 颜色克制，像论文图而不是宣传海报；
- 白底、线条清楚、箭头方向明确；
- 机器人、容器和液面元素足够简洁；
- 图中标签可以后期替换，不影响主体结构；
- 对 Fig. 3 和 Fig. 4，内部代理量与真实外部液面观测的边界不能被画混。

如果一张图结构好但文字差，可以保留为底图并覆盖文字；如果结构混乱、视觉过满或出现假结果，即使颜色漂亮也不要用于论文正文。
