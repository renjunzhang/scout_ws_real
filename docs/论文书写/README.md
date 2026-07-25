# 论文书写与本地 LaTeX 编译说明

本文档记录本仓库中论文/报告类 LaTeX 文档的本地编译用法。当前机器已经安装完整 TeX Live 2026，主要文件放在 `/data/a`，避免占用系统根目录。

## 论文相关 AI Skills

截至 2026-07-18，当前为论文研究、写作和实验图表制作安装了以下 Skills。Codex GPT Pro 与 Claude GPT Pro 使用相互隔离的配置目录，Skill 不跨环境自动共享。

### Academic Research Suite

- 使用环境：`codex-gpt-pro`
- 当前版本：`0.1.17`
- 安装位置：`/home/zrj/.local/share/codex-gpt-pro/codex-home/skills/academic-research-suite`
- 主要用途：深度研究、文献综述、研究问题收敛、论文提纲与写作、引用检查、论文修改、模拟审稿、实验规划、统计解释，以及从研究到成稿的完整工作流。
- 常用入口：自然语言描述任务，或使用 `ars-plan`、`ars-outline`、`ars-lit-review`、`ars-citation-check`、`ars-reviewer`、`ars-full` 等别名。若客户端拦截斜杠命令，优先使用不带 `/` 的普通别名。
- 适用阶段：从选题、调研、实验设计到写作、返修和审稿的全过程。

### scipilot-figure-skill

- 使用环境：`codex-gpt-pro`
- 上游仓库：<https://github.com/Haojae/scipilot-figure-skill>
- 安装位置：`/home/zrj/.local/share/codex-gpt-pro/codex-home/skills/scipilot-figure-skill`
- 专用 Python 环境：`/home/zrj/.local/share/codex-gpt-pro/codex-home/venvs/scipilot-figure-skill`
- 主要用途：先剖析 CSV、Excel 或 DataFrame，再根据论文论点推荐合适的图型，生成出版级数据图，并检查缺字、裁切、标签重叠、灰度辨识度和期刊尺寸规范。
- 支持内容：折线图、散点图、箱线图/小提琴图、柱状图、热力图、分布图、误差棒、相关性图和多面板组合；支持 PDF、SVG、PNG 和 Plotly 静态导出。
- 适用场景：不知道数据该用什么图、需要 Nature/Science/IEEE/Elsevier/PNAS 或中文论文风格、需要色盲友好配色和投稿前视觉自检。
- 调用示例：`用 scipilot-figure-skill 分析 figures/results.csv，并生成 Nature 双栏论文图。`
- 运行约定：Skill 内部脚本统一通过 `scripts/run_python.sh` 执行，不要直接使用系统 `/usr/bin/python3`。该入口会隔离 ROS 的 `PYTHONPATH`，并配置 Kaleido 使用的专用 Chrome。
- 不适用内容：方法示意图、机制图、流程图、系统架构图和模型结构图。

### paper-figure

- 使用环境：`claude-gpt-pro`
- 上游仓库：<https://github.com/wanshuiyin/Auto-claude-code-research-in-sleep/tree/main/skills/paper-figure>
- 安装时上游提交：`86ea69caa6da4baab01e17a4212ecf75fe87661a`
- 安装位置：`/home/zrj/.local/share/claude-gpt-pro/claude-config/skills/paper-figure`
- 主要用途：根据 `PAPER_PLAN.md`、实验 JSON/CSV 和已有图表，批量生成论文实验图、消融图、多面板图和 LaTeX 对比表格，并为每张图保留独立生成脚本。
- 适用场景：论文已经有明确的 Figure Plan 和实验结果，需要在 Claude Code 中批量生产风格统一的图表和表格。
- 调用方式：`/paper-figure <figure-plan-or-data-path>`，例如 `/paper-figure docs/论文书写/草稿/spmpc_paper_cn`。
- 不适用内容：方法框架图、架构图、定性样本图、照片和截图，这些内容仍需人工准备。

### 选择建议

| 当前任务 | 优先使用 |
|----------|----------|
| 选题、文献综述、论文提纲、写作、引用检查、返修或模拟审稿 | `academic-research-suite` |
| 拿到一份数据但不确定该画什么，或需要严格的期刊规范与视觉自检 | `scipilot-figure-skill` |
| 已有 Figure Plan 和实验数据，需要批量生成实验图及 LaTeX 表格 | `paper-figure` |
| 方法框架图、系统架构图、流程图或机制示意图 | TikZ、draw.io、Figma 等人工维护 |

两个画图 Skill 都主要面向数据驱动图表。`scipilot-figure-skill` 更偏向“分析数据、选择正确图型、按期刊规范检查”；`paper-figure` 更偏向“读取既定 Figure Plan，批量生成实验图和 LaTeX 表格”。

## 环境位置

TeX Live 安装位置：

```text
/data/a/texlive/2026
/data/a/texlive/current -> /data/a/texlive/2026
```

常用命令路径：

```text
/data/a/texlive/current/bin/x86_64-linux/xelatex
/data/a/texlive/current/bin/x86_64-linux/latexmk
/data/a/texlive/current/bin/x86_64-linux/tlmgr
```

`~/.zshrc` 已配置 PATH。新终端一般可直接使用：

```bash
xelatex --version
latexmk --version
tlmgr --version
```

如果当前终端找不到 `xelatex` 或 `latexmk`，先执行：

```bash
source ~/.zshrc
```

## 推荐编译方式

中文论文/报告建议使用 XeLaTeX：

```bash
latexmk -xelatex main.tex
```

更严格的非交互编译命令：

```bash
latexmk -xelatex -interaction=nonstopmode -halt-on-error -file-line-error main.tex
```

清理中间文件：

```bash
latexmk -C main.tex
```

如果只想直接运行一次 XeLaTeX：

```bash
xelatex -interaction=nonstopmode -halt-on-error -file-line-error main.tex
```

但一般推荐 `latexmk`，因为它会自动处理多轮编译、引用、目录和交叉引用。

## 中文文档最小示例

可以用下面的内容测试中文编译：

```latex
\documentclass[UTF8]{ctexart}
\usepackage{amsmath}

\begin{document}
你好，LaTeX。

这是一个中文 XeLaTeX 测试。

\[
E = mc^2
\]

\end{document}
```

保存为 `main.tex` 后执行：

```bash
latexmk -xelatex main.tex
```

生成结果应为：

```text
main.pdf
```

## 常用检查命令

检查中文相关宏包：

```bash
kpsewhich ctexart.cls
kpsewhich xeCJK.sty
kpsewhich fontspec.sty
```

检查 TeX Live 用户目录是否在 `/data/a`：

```bash
kpsewhich -var-value=TEXMFHOME
kpsewhich -var-value=TEXMFVAR
kpsewhich -var-value=TEXMFCONFIG
```

期望大致为：

```text
/data/a/texlive/texmf-home
/data/a/texlive/user/2026/texmf-var
/data/a/texlive/user/2026/texmf-config
```

## 更新 TeX Live

更新全部包：

```bash
tlmgr update --self --all
```

当前 `tlmgr` 仓库使用清华 TUNA CTAN 镜像：

```text
https://mirrors.tuna.tsinghua.edu.cn/CTAN/systems/texlive/tlnet
```

不要使用：

```bash
sudo tlmgr
```

因为当前 TeX Live 是用户级安装在 `/data/a`，不需要也不应该用 sudo 管理。

## 安全注意事项

- 不要随便编译来源不可信的完整 LaTeX 工程，尤其是带自定义 `.sty`、`.cls`、脚本或 Makefile 的工程。
- 不可信文档不要开启 `--shell-escape`。
- `minted` 等包通常需要 `--shell-escape`，只应在确认文档可信时使用。
- 不要执行 `sudo apt install texlive-full`，否则会把大量 TeX Live 文件安装到系统目录。
- 如果模板要求额外字体，优先把字体放到 `/data/a/fonts`，再刷新字体缓存。

刷新字体缓存示例：

```bash
fc-cache -fv /data/a/fonts
```

## 推荐目录习惯

建议每篇论文/报告至少包含：

```text
main.tex          # 主文件
references.bib    # 参考文献，可选
figures/          # 图片
sections/         # 章节拆分，可选
```

示例编译入口统一使用：

```bash
latexmk -xelatex main.tex
```
