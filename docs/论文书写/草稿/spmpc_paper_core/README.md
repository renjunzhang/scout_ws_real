# S-MPCC Method-Core Draft

This directory contains a standalone IEEE Transactions-style draft of the
S-MPCC method and experimental evidence chain. It stabilizes the prescribed-path
scope, container-parameterized augmented prediction model, optimal control
problem, matched internal comparisons, cross-container transfer protocol,
propagated-state replay, and receding-horizon execution interface.

The paper-level organization is defined by
`../../论文组织思路/S-MPCC_当前论文组织思路.md`. The authoritative experimental
protocol is `../../实验章节设计/S-MPCC_experimental_design.md`, with the
acquisition matrix summarized in `../../实验章节设计/SPMPC实验矩阵设计.md`.

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
Experiments that have not been completed use explicit `pending` placeholders.
Protocol language is used throughout: the draft does not present a planned
comparison, figure, or replay analysis as an observed result.
Every unexecuted evidence output in RQ1--RQ4 and the supporting analyses is
covered by a `Pending result` block, pending result-table cells, or a pending
supplementary status entry. Protocol definitions, estimands, and acquisition
rules remain ordinary prose because they are frozen design choices rather than
experimental findings.

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

The primary physical outcome is the trial-level RGB p95 over the full motion
window from first effective motion to the common arrival event. The 10%--90%
path-progress result is a pre-specified sensitivity analysis. Raw solver,
post-gate, and published commands are logged separately so that optimizer
differences are not conflated with downstream execution.

The formal physical design uses \(n=8\) randomized blocks and 88 trials:
24 low-risk \(C_1\) trials, 48 high-risk \(C_1/C_2\) super-block trials, and
16 high-risk completion-time-matched trials. The optional \(C_2\) mismatch
group increases the design to 96 trials and is required before a physical
necessity claim about container reparameterization can be considered.
Controlled container-parameter comparisons, longitudinal and lateral
liquid-phase studies, and actual/zero propagated-state replay are computational
studies and add no physical trials. The staged evidence packages contain 40,
64, or 88 trials, with claims reduced if acquisition stops before the full
design.

The main comparison strategy uses matched internal MPCC variants. No broad
ranking against obstacle-oriented local planners is claimed for the current
fixed-path setting.

Before formal acquisition, the numerical controller weights, configured
damping ratio, liquid-state initialization rule, executed-command limits,
intervention tolerances, replay reproduction tolerances, and shared deployment
settings must be frozen in the supplementary configuration. The RQ1 primary
contrast is S-MPCC minus Smooth-only MPCC; the RQ2 primary contrast is S-MPCC
minus Smooth-match MPCC. Whole-block bootstrap intervals, exact paired
sign-flip inference, and leave-one-block-out sensitivity are pre-specified.
A liquid-aware neighboring-method baseline may be considered only after the
core evidence has been completed; it is not part of the current acquisition
matrix.
