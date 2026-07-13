# SPMPC IEEE Transactions Journal Draft

This directory contains the English IEEE Transactions-style journal draft of the SPMPC paper.

## Template Source

The draft uses the local `IEEEtran.cls` supplied with the original template package:

```text
/home/zrj/Downloads/spmpc_paper.zip
```

The local class is the standard IEEEtran v1.8b release and matches the TeX Live system copy. The paper entry point uses the journal option:

```latex
\documentclass[journal]{IEEEtran}
```

Conference-specific author blocks and manual title-font overrides are not used. The previous ACC/PaperPlaza `ieeeconf.cls` template remains outside this directory.

## Build

Use pdfLaTeX/BibTeX:

```bash
cd docs/论文书写/草稿/spmpc_paper
latexmk -pdf main.tex
```

or:

```bash
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```

## Draft Policy

The current English version is a journal-paper draft, not a finished submission. The core narrative is:

1. sloshing is a dynamic-memory state prediction problem, not only a smoothing problem;
2. SPMPC embeds a low-order slosh state in an online MPCC local planner for a standard mobile base;
3. predictive slosh-risk speed-reference shaping is a soft pre-MPCC adaptation module, not a hard safety layer;
4. model proxy quantities must be separated from external liquid-surface observations.
