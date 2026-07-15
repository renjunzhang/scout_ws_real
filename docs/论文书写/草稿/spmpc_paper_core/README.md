# S-MPCC Method-Core Draft

This directory contains a standalone IEEE Transactions-style draft of the
S-MPCC method and its experimental evidence structure. It is intended to
stabilize the problem scope, container-parameterized augmented prediction model,
optimal control problem, matched internal comparisons, cross-container
generalization protocol, and receding-horizon execution interface.

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
fixed-path physical statistics are marked as development-only evidence.
Experiments that have not been completed use explicit `pending` placeholders,
including the randomized-block three-method comparison, completion-time matching,
cross-container generalization, model sensitivity, and runtime distributions.

The core draft contains two paper sections: the S-MPCC method and the experimental
evaluation. The modal cap, predictive speed-reference governor, delay-state
predictor, and deployment safety gates are intentionally excluded. They can be
reintroduced later as supplementary material if the corresponding experiments
support them.

The main text uses only the modal prediction \(H_{\mathrm{modal}}\) and the RGB
observation \(H_{\mathrm{vis}}\). Parabolic diagnostic augmentation is disabled
for the formal protocol. Full weights, solver settings, the experimental
metadata freeze checklist, development-only records, sensitivity plans, and
detailed runtime tables are stored in supplementary/supplementary_material.tex
and are not included by main.tex.

The physical design is frozen as one of three evidence packages before formal
outcomes are inspected. At \(n=8\), the minimum mechanism package contains 40
trials, the two-path package contains 64 trials, and the full RQ1--RQ4 package
contains 88 trials. Only the full package supports the container-transfer
claim; the package cannot be expanded after inspecting formal outcomes.

The main comparison strategy uses matched internal MPCC variants. No broad
ranking against obstacle-oriented local planners is claimed for the current
fixed-path setting.

Before formal acquisition, the numerical controller weights, identified
damping ratio, liquid-state initialization rule, executed-command limits, and
shared deployment settings must be frozen in the supplementary configuration.
The primary estimand and analysis windows are already fixed in the experiment
chapter. A liquid-aware neighboring-method baseline will be considered only
after the core randomized-block, time-matched, and container-transfer evidence
has been completed; it is not part of the current acquisition matrix.
