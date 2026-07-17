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
metadata freeze checklist, the synchronized-signal checklist, development-only
records, sensitivity plans, and detailed runtime tables are stored in
supplementary/supplementary_material.tex and are not included by main.tex.

The formal physical design uses \(n=8\) randomized blocks and 88 trials:
24 low-risk \(C_1\) trials, 48 high-risk \(C_1/C_2\) super-block trials, and
16 high-risk completion-time-matched trials. The optional \(C_2\) mismatch
group adds eight trials but remains supporting evidence. Controlled
container-parameter and liquid-phase planning comparisons are computational
studies and add no physical trials.

The main comparison strategy uses matched internal MPCC variants. No broad
ranking against obstacle-oriented local planners is claimed for the current
fixed-path setting.

Before formal acquisition, the numerical controller weights, configured
damping ratio, liquid-state initialization rule, executed-command limits, and
shared deployment settings must be frozen in the supplementary configuration.
The primary estimand and analysis windows are already fixed in the experiment
chapter. A liquid-aware neighboring-method baseline will be considered only
after the core randomized-block, time-matched, and container-transfer evidence
has been completed; it is not part of the current acquisition matrix.
