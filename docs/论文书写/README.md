# 论文书写与本地 LaTeX 编译说明

本文档记录本仓库中论文/报告类 LaTeX 文档的本地编译用法。当前机器已经安装完整 TeX Live 2026，主要文件放在 `/data/a`，避免占用系统根目录。

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
