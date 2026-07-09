# SPMPC IEEE Conference Paper Draft

This directory contains the English IEEE conference-style draft of the SPMPC paper.

## Template Source

This draft currently uses the IEEE conference template package supplied by:

```text
/home/zrj/Downloads/spmpc_paper.zip
```

The active template class is `IEEEtran.cls`, and the paper entry point uses:

```latex
\documentclass[conference]{IEEEtran}
\IEEEoverridecommandlockouts
```

The previous ACC/PaperPlaza `ieeeconf.cls` template has been removed from this directory for now.

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

The current English version is a conference-paper scaffold, not a finished translation. The core narrative is:

1. sloshing is a dynamic-memory state prediction problem, not only a smoothing problem;
2. SPMPC embeds a low-order slosh state in an online MPCC local planner for a standard mobile base;
3. the Slosh-risk Reference Governor is a soft pre-MPCC reference adaptation module, not a hard safety layer;
4. model proxy quantities must be separated from external liquid-surface observations.
