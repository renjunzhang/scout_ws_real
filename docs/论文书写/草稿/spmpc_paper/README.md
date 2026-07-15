# S-MPCC IEEE Transactions Journal Draft

This directory contains the complete English IEEE Transactions-style journal draft of the S-MPCC paper. Its method and experiment sections are aligned with `../spmpc_paper_core`.

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
2. S-MPCC embeds a container-parameterized low-order slosh state in an online MPCC local planner for a standard mobile base;
3. the core comparison uses Baseline MPCC, Smooth-only MPCC, and S-MPCC, with the reference governor and modal hard cap disabled;
4. model quantities must be separated from the calibrated vision-based experimental reference measurement;
5. the selected 40-, 64-, or 88-trial evidence package determines which path and container claims may remain in the paper.

Full solver settings, controller weights, the experimental metadata freeze checklist, development-only records, the tiered experiment matrix, sensitivity plans, and runtime details are stored in `supplementary/supplementary_material.tex`. The file is not included by `main.tex` and must be packaged separately if used for submission.
