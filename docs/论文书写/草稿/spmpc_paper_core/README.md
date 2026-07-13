# SPMPC Method-Core Draft

This directory contains a standalone IEEE Transactions-style draft of the
SPMPC method and its experimental evidence structure. It is intended to
stabilize the problem scope, augmented prediction model, optimal control
problem, matched internal comparisons, optional speed-reference shaping, and
receding-horizon execution interface.

The entry point uses the local IEEEtran v1.8b class in journal mode:

```latex
\documentclass[journal]{IEEEtran}
```

## Build

```bash
cd docs/论文书写/草稿/spmpc_paper_core
latexmk -pdf main.tex
```

## Draft Status

The method equations are accompanied by a complete experiment chapter. Existing
fixed-path physical statistics are marked as preliminary. Experiments that have
not been completed use explicit `pending` placeholders, including the complete
four-cell factorial matrix, the completion-time-matched comparison, model
sensitivity, runtime distributions, and predictive reference shaping.

The main comparison strategy uses matched internal MPCC variants. No broad
ranking against obstacle-oriented local planners is claimed for the current
fixed-path setting.
