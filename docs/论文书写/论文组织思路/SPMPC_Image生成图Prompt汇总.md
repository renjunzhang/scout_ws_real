# SPMPC 面向 RAL 审美的 Image 2.0 论文图 Prompt 汇总

本文档用于为 `docs/论文书写/草稿/spmpc_paper_cn` 中的中文 SPMPC 初稿生成论文图底图或版式参考。目标不是让 Image 2.0 一次性生成可直接投稿的最终图，而是生成符合 IEEE RA-L / ICRA 论文审美的**干净科研图底稿**：结构、留白、箭头、图标和色彩先到位，正式文字、公式、变量名和实验曲线后续由 LaTeX、SVG、矢量软件或真实数据脚本覆盖。

当前正文主要图位：

- **Fig. 1：任务与方法定位示意图**，放在引言；
- **Fig. 2：SPMPC 增强状态与 OCP 结构图**，放在方法章；
- **Fig. 3：实验平台与评价流程图**，放在实验章；
- **Fig. 4：内部消融变体与验证关系图**，放在实验章；当前是设计图，不是结果图。

当前实验章结构已经调整为：

```text
A. 实验设置与评价指标
B. 仿真实验：内部消融与外部基线对比
C. 实物实验：真实液面响应与模型一致性
D. 在线求解性能与执行诊断
```

当前方法章已经区分内部模型代理量的两种口径：

```text
H_model^modal : 仅由模态位移给出的动态晃液代理量
H_model^full  : H_model^modal + 可选转弯抛物面代理项
H_model^eval  : 实验中实际拿去和 H_vis 对比的口径，可取 modal 或 full
```

这些量以及 `/spmpc/slosh_height` 都只能理解为**模型内部预测或诊断代理量**，不能等同真实液面。真实液面结论必须由 RGB、液位传感或其它外部观测 `H_vis` 支持。

建议生成文件先放入论文图目录：

```text
docs/论文书写/草稿/spmpc_paper_cn/figures/
```

建议候选文件名：

```text
fig1_system_overview_image2.png
fig2_method_structure_image2.png
fig3_experiment_pipeline_image2.png
fig4_ablation_design_image2.png
```

> **当前建议：不要把 Image 2.0 输出直接作为论文正文图。** 已试生成的 `1.png`--`4.png` 更适合作为版式灵感或反例，不适合直接插入 RA-L/ICRA 风格论文。当前正文图优先采用 TikZ、SVG、draw.io/Figma 手工矢量图或真实数据脚本生成；Image 2.0 只作为早期构图探索工具。若生成图出现图标感过强、AI 小字、结构含混、像信息图或结果暗示，应直接放弃，不进入论文仓库。

---

## 0. 全局原则

### 0.0 对当前试生成图片的评估结论

已检查 `/home/zrj/Downloads/科研工作/论文配图/1.png`--`4.png`。这些图**不建议直接插入论文**，主要问题是：

- 整体更像 AI 信息图或汇报插画，而不是可投稿的 RA-L/ICRA 矢量论文图；
- 小字、符号、图标语义不稳定，缩到双栏单栏宽度后可读性和严谨性不足；
- Fig. 1--3 类流程图的模块关系不如当前 TikZ 图清楚，容易把 planner log、internal proxy 和 external observation 的边界画混；
- Fig. 4 虽然接近消融矩阵，但文字和图标过多，作为正文图仍应优先用 TikZ/SVG 重画；
- 这些图无法保证 `H_model^modal`、`H_model^full`、`H_model^eval`、`/spmpc/slosh_height` 与真实外部观测 `H_vis` 的边界表达足够严谨。

因此，当前策略调整为：

```text
正文图：优先 TikZ / SVG / draw.io / Figma / 数据脚本生成。
Image 2.0：只用于构图灵感，不作为最终图源。
实验结果图：只能由真实数据脚本生成。
```

### 0.1 新版收敛 Prompt：只生成论文图底稿

下面这组 prompt 是当前推荐版本。目标不是让 Image 2.0 直接生成最终论文图，而是生成**可被临摹、矢量化或后期覆盖文字的论文图底稿**。如果输出仍然像 AI 信息图、PPT 插画、图标包或含有假数据，应直接放弃。

通用前缀：

```text
Create a publication-style technical diagram for an IEEE Robotics and Automation Letters paper. The figure should look like a clean TikZ or SVG schematic, not like an infographic, not like a poster, and not like a presentation slide.

Use a white background, flat vector-like line art, thin consistent strokes, very limited colors, and large empty regions where final labels will be added manually later. Use only simple geometric shapes, rounded rectangles, thin arrows, minimal robot/container/camera icons, and clear layout hierarchy.

Prefer almost no text. If text is unavoidable, use only short placeholder labels in large readable English, but leave enough space for manual LaTeX labels. Do not generate formulas, paragraphs, small labels, Chinese characters, numerical values, plots, curves, tick marks, or performance claims.

Style: restrained academic blue for the proposed SPMPC flow, neutral gray for auxiliary modules, light cyan only for liquid or external visual observation, and a tiny muted orange only for excitation or contrast.
```

通用负面 prompt：

```text
No fake experimental data, no fabricated plots, no line charts, no bar charts, no numerical values, no percentages, no performance trend, no equations, no tiny text, no dense annotations, no Chinese characters, no pseudo-random labels, no watermark, no logo, no decorative background, no photorealistic rendering, no glossy 3D style, no cartoon style, no icon-pack style, no excessive shadows, no gradients, no clutter, no poster-like composition.
```

#### Fig. 2 推荐新版 Prompt：SPMPC 方法结构图

```text
Create a no-text technical block diagram skeleton for the core method of a robotics paper. The diagram should look like a clean TikZ/SVG schematic drawn by an engineer.

Layout: a left-to-right receding-horizon local-planning architecture with five clearly separated regions.

Region 1, far left: four simple input symbols arranged vertically: mobile-base state, reference path, previous command, and low-order slosh state. Use only minimal geometric icons, not detailed illustrations.

Region 2, center-left: one large rounded rectangle representing augmented dynamics. Inside it, show three very simple visual hints: a mobile-base model, a path-progress curve, and a low-order slosh-memory oscillator connected to a small open container. Do not use formulas.

Region 3, center: one large rounded rectangle representing an MPCC optimal-control problem. Show a short prediction horizon as small repeated dots or rectangles, and four tiny objective icons arranged below it: path tracking, path progress, smooth control, and predicted slosh response. No labels, no equations.

Region 4, center-right: a narrow highlighted block representing the first applied control action only.

Region 5, far right: a minimal mobile-base output icon. Add a clean feedback arrow from the output back to the augmented-dynamics region, indicating receding-horizon replanning.

Use aligned boxes, straight orthogonal arrows, large whitespace, thin gray strokes, and a single academic-blue flow path. Use light cyan only inside the liquid container. The figure should remain readable at single-column width in a two-column IEEE paper.

Do not include ROS topics, software nodes, complex robot drawings, formulas, mathematical symbols, small text, fake data, plots, or decorative elements.
```

追加负面约束：

```text
Avoid infographic style, avoid icon-heavy layout, avoid dashboard style, avoid flowchart clutter, avoid more than 6 main modules, avoid photorealistic robots, avoid 3D blocks, avoid labels that look autogenerated.
```

#### Fig. 3 推荐新版 Prompt：实验平台与评价流程图

```text
Create a clean no-text experimental evaluation pipeline diagram for an IEEE RA-L robotics paper. The diagram should look like a minimal TikZ/SVG schematic, not an infographic.

Use a two-lane layout with clear separation between internal model information and external liquid observation.

Left side: a simple wheeled mobile robot carrying an open container with light cyan liquid, following a smooth reference path on a plain white background. Add a minimal external camera icon observing the container from outside the robot/planner loop.

Middle: split the information into two visually distinct horizontal streams.

Upper stream: internal planner/model stream. Use neutral gray boxes and arrows to represent planner logs, commands, solver time, and internal model proxy. This stream must stay inside the model/logging side.

Lower stream: external observation stream. Use light cyan boxes and arrows to represent RGB or liquid-level observation of the real liquid surface. This stream should originate from the external camera/sensor.

Right side: both streams enter a clean evaluation block. From the evaluation block, four small output boxes go to the right: real liquid response, model-observation fidelity, task/path metrics, and solver/runtime metrics. Use only empty boxes or very short placeholder labels if necessary.

Important visual rule: the internal proxy stream and the external observation stream must not be confused. The external stream is the only one visually connected to real liquid-surface conclusions.

White background, thin strokes, academic blue for planner flow, gray for internal logs, cyan for external liquid observation, balanced whitespace, no data plots.
```

追加负面约束：

```text
No experimental curves, no fake H_model vs H_vis plot, no RMSE values, no correlation values, no numerical metrics, no performance claims, no equations, no small text, no Chinese characters, no photorealistic lab scene, no decorative camera details, no dashboard style.
```

#### Fig. 1 推荐新版 Prompt：任务与方法定位图

```text
Create a clean three-panel research-gap schematic for an IEEE RA-L robotics paper. The style should resemble a simple TikZ/SVG figure with minimal icons and large whitespace.

Use three vertical panels arranged horizontally from left to right.

Panel 1: conventional mobile-robot local planning. Show a minimal mobile robot carrying an open liquid container, a reference path, and a velocity-command arrow. The planner considers robot/path information only. The liquid is visible but visually disconnected from the planner.

Panel 2: existing anti-sloshing methods. Show a simple liquid model icon connected to offline trajectory, speed profile, input shaping, or tracking-control blocks. Make it visually clear that this is not the same as an online local planner.

Panel 3: proposed SPMPC online local planner. Show robot state, path progress, and a low-order slosh-state icon entering a single receding-horizon MPCC local-planner block, with the first mobile-base command as output. Use a subtle academic-blue outline around this panel.

Keep the panels symmetric and simple. Use no more than 3–4 visual elements per panel. Prefer no text; leave blank label areas above each panel for manual labels. Use gray for existing methods, academic blue for SPMPC, and light cyan for liquid.
```

追加负面约束：

```text
No result comparison, no winner/loser visual metaphor, no fake performance, no plots, no equations, no long labels, no icon clutter, no photorealistic robots, no laboratory scene, no decorative background, no poster style.
```

#### Fig. 4 推荐新版 Prompt：内部消融设计图

```text
Create a clean ablation-design matrix for a robotics paper. The figure should look like a vector diagram or TikZ schematic, not an infographic.

Use a simple 2 by 2 matrix with four empty rounded cells. The horizontal axis represents absence or presence of a slosh-state model. The vertical axis represents weak or strong control smoothness. Do not draw data.

Place a very small placeholder area in each cell for method labels that will be added manually later. Use subtle blue emphasis only on the lower-right cell, representing the full method. Use thin gray grid lines and large whitespace.

Add simple comparison arrows between cells:
- vertical arrow on the left column for smoothness effect;
- horizontal arrow on the top row for slosh-state effect;
- horizontal arrow on the bottom row for full method versus smooth-only;
- vertical arrow on the right column for smoothness with slosh state.

Keep the diagram minimal. Do not include robot illustrations inside every cell unless they are extremely simple and small. No plots, no curves, no bars, no numbers, no percentages, no performance trend.
```

追加负面约束：

```text
No experimental results, no fake curves, no fake ranking, no bar charts, no scatter points, no detailed robot icons, no dense legend, no long text, no equations, no Chinese characters, no decorative background.
```

#### 如果只生成一张图，优先使用 Fig. 2 收敛版

```text
Create a clean no-text TikZ-style schematic for an IEEE RA-L robotics paper.

Draw a left-to-right receding-horizon MPCC local-planning architecture for a wheeled mobile robot carrying an open liquid container. Use only simple rounded rectangles, thin arrows, and minimal line icons.

From left to right:
1. four compact input icons: robot state, reference path, previous command, low-order slosh state;
2. one augmented-dynamics block that visually combines mobile-base dynamics, path progress, and slosh modal memory;
3. one MPCC OCP block with a short prediction horizon and four small objective icons: tracking, progress, smoothness, predicted slosh;
4. one highlighted first-command block;
5. one mobile-base output icon;
6. a feedback arrow indicating the next receding-horizon cycle.

Use white background, thin gray strokes, academic blue for the main SPMPC flow, light cyan only for liquid, and generous whitespace. Leave empty label zones for manual LaTeX labels.

No equations, no small text, no Chinese characters, no ROS graph, no fake plots, no numerical values, no photorealistic rendering, no glossy 3D style, no cartoon style, no decorative elements.
```

### 0.2 论文图定位

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
- Fig. 4 的消融设计图底图。

Image 2.0 **不适合**生成：

- 最终实验曲线；
- 具体数值、百分比或性能排名；
- 真实液面曲线；
- `H_model` 与 `H_vis` 的一致性曲线；
- Ferrari-style 模型--外部液面一致性结果；
- RMSE、correlation、phase lag、bias 等具体统计结果；
- 复杂公式、小号变量名和长句；
- 需要严格对齐的最终可投稿矢量图。

新增的模型--外部液面一致性评价只能在真实外部液面数据可用后绘制。不能用 Image 2.0 生成假曲线、假对齐趋势、假 RMSE / correlation / bias 结果。

### 0.3 内部代理量与真实液面边界

图中如需表达液面评价，必须区分两类量：

```text
Internal model proxy:
  H_model^modal
  H_model^full
  /spmpc/slosh_height

External liquid observation:
  H_vis from RGB, liquid-level sensor, or other external observation
```

通用规则：

- `/spmpc/slosh_height` 不能画成传感器直接测到的真实液面；
- `H_model^modal` 和 `H_model^full` 不能画成真实自由液面高度；
- `H_model^eval` 只是实验中选择的模型代理量口径；
- 真实液面结论只能来自外部观测 `H_vis`；
- 模型--外部一致性只用于诊断解释，不替代真实液面结论，也不构成形式化防溢出保证。

### 0.4 通用审美 prompt

每次生成建议加入：

```text
IEEE Robotics and Automation Letters style, clean scientific diagram, minimal academic illustration, white background, flat vector-like raster illustration, thin consistent strokes, restrained blue-gray color palette, high readability in a two-column robotics paper, balanced whitespace, no decorative elements.
```

配色建议：

```text
Use a restrained palette: deep academic blue for the proposed SPMPC flow, light cyan for liquid and slosh state, neutral gray for auxiliary modules and baselines, and a very small amount of muted orange only for slosh excitation or key contrast.
```

### 0.5 文字策略

优先采用两阶段制图：

1. Image 2.0 生成**无文字或少文字底图**；
2. 后续用 LaTeX / SVG / Illustrator / Inkscape 覆盖正式标签。

通用文字约束：

```text
Prefer a no-text diagram. If text is necessary, use only very short English placeholder labels with large font. No paragraphs, no equations, no tiny text, no Chinese characters, no pseudo-random labels. Leave clear empty label zones so final labels can be added manually later.
```

### 0.6 通用负面 prompt

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

不要把 `H_model^modal`、`H_model^full` 或 `/spmpc/slosh_height` 画成传感器测量。它们可作为“diagnostic output”或“model proxy”在 Fig. 3 中出现，但不是 Fig. 2 的核心。

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

Fig. 3 用于说明实验和评价边界：真实液面结论来自外部观测；内部 `/spmpc/slosh_height`、`H_model^modal`、`H_model^full` 只是模型预测或诊断代理量。当前实验章采用“设置与指标 → 仿真实验 → 实物实验 → 在线求解性能”的结构，因此 Fig. 3 重点是展示评价流，而不是展示结果。

## 推荐构图

左右结构：

1. 左侧：移动底盘、开口容器、参考路径、外部相机或液位观测模块；
2. 右侧：Planner Logs、Internal Model Proxy、External Liquid Observation、Evaluation Metrics / Fidelity Check。

内部模型代理量与外部液面观测必须分成两条视觉上不同的流：

```text
Internal proxy stream:
  model states, H_model^modal, H_model^full, /spmpc/slosh_height

External observation stream:
  RGB / liquid-level sensor, H_vis
```

如果担心 Image 2.0 生成公式或乱码，则图中只写非常短的英文占位词，例如 `Proxy`、`External`、`Metrics`，正式符号后续手工覆盖。

## 优先 Prompt：无假数据实验流程图

```text
Create a clean experimental setup and evaluation-pipeline figure for an IEEE RA-L robotics paper, white background, flat vector-like raster illustration, minimal and precise.

Left side: show a simple wheeled mobile robot carrying an open liquid container along a smooth reference path from start to goal on a flat indoor floor. Keep the environment abstract and minimal, like a lab or corridor outline, not a detailed scene. The liquid is light cyan with a small sloshing surface.

Show an external RGB camera or external liquid-level observation module looking at the container. The camera must be outside the planner loop, indicating independent liquid-surface evaluation.

Right side: show a clean logging and evaluation pipeline. Use separate streams for planner logs, internal model proxy, and external liquid observation. The internal proxy stream should be visually distinct from the external observation stream. The external observation stream is the only source for real liquid-surface conclusions. Both streams can feed a fidelity-check or evaluation module, which outputs categories such as liquid response, model-observation fidelity, task completion, path tracking, smoothness, and solver time.

Use blue for planner flow, gray for internal logs/proxy, and cyan for external liquid observation. Keep labels very short or omit labels and leave space for manual labels. Use thin strokes, aligned boxes, and clear arrows.

No fake plots, no numerical values, no experimental curves, no performance claims, no equations, no dense text, no Chinese characters, no photorealistic rendering, no decorative lab details, no watermark, no logo.
```

## 备选 Prompt：强调内部代理量 vs 真实观测

```text
Design a minimal RA-L style figure showing how open-liquid transport experiments are evaluated. White background, flat scientific illustration, blue-gray palette.

A mobile robot with an open liquid container follows a reference path. The planner produces velocity commands and internal logs, including internal model-proxy outputs. Separately, an external RGB camera observes the liquid surface. Both streams go to an evaluation module, but the diagram clearly indicates that real liquid conclusions come from external observation, not from the internal model proxy.

Use clean arrows, large readable placeholder labels, subtle cyan liquid, a simple robot icon, a simple camera icon, and compact metric boxes. No fake plots, no numbers, no dense text, no equations, no decorative lab details, no watermark, no logos.
```

## 避免

不要出现具体液面曲线、具体一致性数值、真实结果趋势或“模型已经验证准确”的暗示。Fig. 3 是**评价流程图**，不是实验结果图。尤其不要把内部 proxy stream 画成外部传感器。

---

# Fig. 4 内部消融变体与验证关系图

## 正文位置

实验章。当前正文图名：

```text
内部消融变体与验证关系：B0 / B_smooth / B_slosh / B_ours 的二维拆解
```

## 图的任务

Fig. 4 当前是**消融设计图**，不是实验结果图。它说明四个变体如何拆分两件事：

```text
是否传播晃液状态
是否增强控制平滑性
```

它的作用是帮助读者理解后续仿真实验如何排除 “SPMPC 只是 smooth-only” 这一替代解释。该图不应包含实验曲线、数值、统计结果或性能趋势。

## 推荐构图

二维矩阵：

```text
                 No slosh state      Slosh state
Weak smoothness       B0              B_slosh
Strong smoothness     B_smooth        B_ours
```

箭头或注释可以表示四组比较关系：

- `B_smooth vs B0`：只增强平滑性有什么收益；
- `B_slosh vs B0`：显式晃液状态预测是否有独立收益；
- `B_ours vs B_smooth`：完整方法是否超越 smooth-only；
- `B_ours vs B_slosh`：平滑项是否仍有助于可执行命令。

## 优先 Prompt：消融设计图，无数据

```text
Create a clean IEEE RA-L style ablation-design diagram, white background, flat vector-like raster style, no experimental data.

Use a compact 2 by 2 matrix layout. The columns represent absence or presence of a slosh-state model. The rows represent weak or strong control smoothness. Put four method cells in the matrix: B0, B_smooth, B_slosh, and B_ours. Highlight B_ours with a subtle academic-blue outline or light blue fill.

Add thin comparison arrows or visual connectors that indicate four comparison questions: smoothness benefit, slosh-state benefit, whether the full method exceeds smooth-only, and whether smoothness still helps executable commands. Keep all text very short and readable, or leave blank spaces for labels to be added manually later.

Clean academic layout, thin strokes, restrained blue-gray palette, no curves, no bars, no scatter points, no numerical values, no percentages, no performance claims, no fake experimental data, no fabricated plots, no equations, no decorative background, no watermark, no logo.
```

## 备选 Prompt：无文字矩阵底图

```text
Create a no-text scientific ablation matrix for a robotics paper. White background, clean 2 by 2 grid, flat vector-like style, thin gray lines, subtle academic-blue emphasis on the lower-right cell.

The matrix should visually represent two binary factors: slosh-state model absent/present and control smoothness weak/strong. Show four empty rounded cells with small reserved label areas, and simple arrows showing horizontal, vertical, and diagonal comparisons. Leave enough whitespace for final labels B0, B_smooth, B_slosh, B_ours to be added manually.

No data, no plots, no numbers, no equations, no Chinese characters, no fake results, no decorative elements.
```

## 不能做的事

不要让 Image 2.0 生成：

- “SPMPC 明显最好”的假曲线；
- 具体百分比降低；
- 具体数值表；
- 真实液面曲线；
- 模型--外部液面一致性曲线；
- 实物照片替代结果图；
- 看起来像已完成实验的统计结论。

---

# 后续结果图说明：只能由真实数据生成

当前正文还没有放入最终实验结果图。后续如果加入结果图，建议是：

```text
Fig. 5 仿真消融与外部基线结果
Fig. 6 实物 H_model^eval vs H_vis 曲线对齐
Fig. 7 在线求解时间统计
```

这些图**不应由 Image 2.0 生成实验内容**。它们只能由真实仿真、实物或外部液面观测数据脚本绘制。Image 2.0 最多用于生成空白版式参考，不能生成曲线、柱状图、统计值、误差带或性能趋势。

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

# 推荐制图优先级

1. **Fig. 2：SPMPC 增强状态与 OCP 结构图**  
   方法章核心图，建议优先用 TikZ / SVG / draw.io / Figma 手工重画。Image 2.0 只能作为构图参考，不建议直接使用其输出。

2. **Fig. 3：实验平台与外部液面评价流程图**  
   当前实验章强调仿真--实物双层证据链、真实液面评价和内部代理量边界，因此 Fig. 3 对论文可信度很重要。建议保留或优化当前 TikZ 图，确保 internal proxy 和 external observation 明确分离。

3. **Fig. 1：任务与方法定位示意图**  
   引言 novelty 图，决定读者第一印象；重点是层级差异，不是结果优越性。建议用简洁 TikZ/SVG 三栏图，不用 AI 信息图风格。

4. **Fig. 4：内部消融变体与验证关系图**  
   当前已有 TikZ 设计图；建议继续使用 TikZ 或矢量软件打磨。若使用 Image 2.0，只能作为矩阵布局参考，不生成结果、不生成复杂图标。

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
- 对 Fig. 3，内部代理量与真实外部液面观测的边界不能被画混；
- 对 Fig. 4，必须是消融设计图，不是结果图；
- 任何 `H_model^modal`、`H_model^full`、`H_model^eval` 或 `/spmpc/slosh_height` 都不能被画成真实液面传感器输出；
- 任何 `H_vis`、RGB、液位传感等外部观测流必须与内部模型代理量视觉区分。

如果一张图结构好但文字差，可以保留为底图并覆盖文字；如果结构混乱、视觉过满或出现假结果，即使颜色漂亮也不要用于论文正文。
