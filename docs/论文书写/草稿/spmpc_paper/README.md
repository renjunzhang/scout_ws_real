# S-MPCC IEEE Transactions Journal Draft

This directory contains the complete English IEEE Transactions-style journal
draft of the S-MPCC paper. The content authority is `../spmpc_paper_core`:
shared method, experiment, supplementary, abstract-scope, terminology, and
claim boundaries must remain synchronized with the core draft. This directory
adds the Introduction, Related Work, and Conclusion required for the complete
paper and keeps those sections consistent with the core evidence chain.

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

The current English version is a journal-paper draft, not a finished
submission. Its frozen narrative is:

1. sloshing is a dynamic-memory state-prediction problem, not only a smoothing
   problem;
2. S-MPCC embeds a container-parameterized low-order slosh state in an online
   path-progress MPCC for a standard mobile base;
3. the core physical comparison uses Baseline MPCC, Smooth-only MPCC, and
   S-MPCC, while RQ2 uses an independently tuned Smooth-match controller;
4. the main physical outcome is trial-level RGB p95 over the full motion
   window; the 10%--90% path-progress window is sensitivity only;
5. raw solver, post-gate, and published commands are recorded separately, and
   replay quantities are described as optimized first-action differences unless
   both branches pass through the identical shared execution layer;
6. the selected 40-, 64-, or 88-trial evidence package determines which path
   and container claims may remain in the paper.

All unexecuted formal outputs in RQ1--RQ4 and the supporting analyses remain
explicitly marked by `Pending result` blocks, pending table cells, or pending
supplementary status entries. Frozen protocol definitions, estimands, and
acquisition rules remain ordinary prose because they are design commitments,
not claimed observations. Formal findings must replace the placeholders only
after the corresponding analysis has been completed.

The reference governor and modal hard cap are disabled in the core comparison.
Delay prediction, terminal handling, command gating, rate limiting, and fallback
logic are shared deployment-layer mechanisms rather than S-MPCC contributions.

Full solver settings, controller weights, the experimental metadata freeze
checklist, synchronized-signal requirements, formal matrix, replay audit,
development-only records, sensitivity plans, and runtime details are stored in
`supplementary/supplementary_material.tex`. The file is not included by
`main.tex` and must be packaged separately if used for submission.
