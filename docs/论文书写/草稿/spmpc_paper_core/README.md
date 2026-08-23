# Phase-Rejoining Residual MPC Core

This directory is a standalone IEEE journal-style draft of the method and
experimental core for **Phase-Rejoining Residual MPC**. Its central rule is: an
online residual correction is admitted only when the unequal-delay prediction
remains eligible to rejoin the complete phase-indexed offline anti-slosh tail.
The implemented online objective is nominal-relative tracking, not a
contour/lag MPCC objective; **OrdinaryMPCC** is retained only as the name of the
liquid-agnostic comparison baseline.

This is an implementation-aligned method and evidence plan, not a formal
performance report or a released physical result.  The implementation-facing
method contract is maintained in
`../../../../docs_for_offlineslosh/Methods/Methods章节组织思路.md`; the older
S-MPCC-named full-paper outline is legacy planning material rather than the
method authority for this draft.

## Local and Full-Paper Numbering

The standalone core intentionally omits Introduction, Related Work, and
Conclusion. Consequently, LaTeX numbers its two included sections locally as:

| Standalone core | File | Full-paper destination |
| --- | --- | --- |
| Section I | `sections/01_method.tex` | Section III, Method |
| Section II | `sections/02_experiments.tex` | Section IV, Experimental Evaluation |

References to “Section III/IV” in planning documents describe the full paper;
the generated core PDF correctly displays “I/II”. Do not hard-code section
numbers in prose or figures—use LaTeX labels and references.

## Contents

- `main.tex`: title, abstract, shared macros, bibliography entry, and document
  entry point;
- `sections/01_method.tex`: parameterized offline speed-profile artifact,
  unequal-delay alignment, neighboring phase selection, residual OCP,
  execution compatibility, empirical terminal gate, and supervisor;
- `sections/02_experiments.tex`: RQ1--RQ5, C0--C4, A0--A3, G0--G4, RGB outcome,
  recovery evaluation, and runtime/command-chain audit;
- `figures/phase_rejoining_framework_paper.{svg,pdf,png}`: editable and
  publication-ready architecture figure used by the method section; and
- `supplementary/supplementary_material.tex`: a repository/supplement fragment
  containing release-freeze and analysis templates.

The supplementary fragment is not included by `main.tex`. It relies on the
packages and macros in `main.tex` and must be wrapped or included by a venue-
specific supplementary document.

## Current Evidence Status

The implementation now contains the unequal-delay 22-D execution augmentation,
phase selection, current and horizon-wide execution compatibility, terminal
empirical gate, frozen bounded recovery feedback, fail-closed supervision, and
receipt-backed final-command history. These are implementation and development
evidence only.

| Evidence layer | Current status | What it establishes |
| --- | --- | --- |
| Development implementation and simulation | Completed for the pre-review implementation under development checks | The controller paths and evidence plumbing run; this is not a comparative performance result. |
| Formal C0--C4 campaign | **Not executed** | No preregistered anti-slosh, task-performance, noninferiority, or ablation result exists yet. |
| Physical qualification and experiments | **Not started** | No physical efficacy, robustness, or deployment claim is available. |

Development branch coverage, internal liquid-state trends, and readiness checks
cannot establish physical anti-slosh efficacy.

## Comparison and Ablation Identifiers

The system-level conditions are:

| ID | Frozen interpretation |
| --- | --- |
| C0 OrdinaryMPCC | Liquid-agnostic contour/lag/progress MPCC and the common downstream safety chain. |
| C1 SmoothMPCC | C0 with independently task-only-tuned and frozen smoothing settings plus the frozen `global_time_scale=1.0`; completion time is reported as a fairness outcome, not claimed to be exactly matched to C4. The paper-facing name is SmoothMPCC; legacy engineering identifiers remain `smooth_match_mpcc` and `C1_smooth_match_mpcc.yaml`. |
| C2 OfflineReplay | Clocked replay of the same frozen offline artifact as C4, with zero online residual. |
| C3 GateMonitorPR-RMPC | The same execution-aware residual MPC, phase selector, full-horizon execution compatibility, frozen recovery feedback, safety chain, and publication transaction as C4; only the 9-D empirical gate is monitor-only. The paper-facing name avoids implying that execution compatibility or recovery is absent; legacy engineering identifiers remain `residual_no_gate` and `C3_residual_no_gate.yaml`. |
| C4 Full | C3 with the 9-D empirical gate enforced in the OCP/terminal residual admission and in current-state recovery admission. |

C0--C4 are a system evidence ladder, not a sequence of single-factor changes.
The three paired C4 contrasts have distinct roles. C4--C0 is the sole
preregistered primary contrast. C4--C1 is the fixed-sequence secondary fairness
comparison: because C1 is independently tuned rather than exactly matched in
time and smoothness, a favorable result weakens---but cannot eliminate---the
generic slower/smoother alternative explanation. C4--C3 reports the paired
effect of changing the 9-D empirical gate from monitor-only to enforced,
together with gate activation, recovery, zero requests, and failures; it does
not veto the C4--C0 result. C2 and IS are descriptive references reported in
one secondary benchmark table. Of these pairs, only C3--C4 is structurally
single-factor.

Only A2 below is instantiated in the current 96-trial matrix.  The other
component-attribution designs would require separately frozen strict pairs:

| ID | Pair | Sole planned change |
| --- | --- | --- |
| A0 | OfflineSmoothReplay vs. C2 | Offline liquid objective off/on. |
| A1 | PhaseAlignedNoResidual vs. C3 | Finite online residual off/on. |
| A2 | C3 vs. C4 | The same empirical gate monitored versus enforced. |
| A3 | IdealExec vs. IdentifiedExec | Idealized versus identified unequal-delay execution model and consistent artifact. |

The formal simulation contract freezes sixteen seeds crossed with all six
conditions (C0--C4 and IS), for 96 trials in a pre-generated interleaved order.
Paths, seeds, condition files, order, outcome definitions, and
analysis code are frozen before acquisition, and formal results are not used to
retune the methods. The sixteen-block count and 0.5-mm margin do not yet have a
prospective power or precision justification; this is reported as a study
limitation rather than introduced as a new release asset. The simulation count
must not be confused with the later hardware sample size.

## Next Execution Step

After this necessary control/statistics patch is committed, run one clean build
and readiness check to confirm the path, plant, six conditions, seeds, order,
and analysis settings.  Human review then precedes the 96-trial campaign.  No
new platform contract or result-driven retuning is added at this stage.  The
physical qualification remains a later, separate activity.

## Build

Normal local build:

```bash
cd docs/论文书写/草稿/spmpc_paper_core
latexmk -pdf main.tex
```

Clean verification build without generated files beside the source:

```bash
paper_out="$(mktemp -d /tmp/spmpc-paper-build.XXXXXX)"
latexmk -pdf -interaction=nonstopmode -halt-on-error \
  -outdir="$paper_out" main.tex
rg -n 'Warning|undefined|Overfull|Underfull|Error' \
  "$paper_out/main.log" "$paper_out/main.blg"
pdfinfo "$paper_out/main.pdf" | rg 'Pages|Page size|File size'
```

The current clean standalone build is **9 pages**. That count covers the local
Method and Experiments sections, title/abstract, and references; it excludes the
full paper's Introduction, Related Work, and Conclusion. It is therefore a page-
budget warning, not a submission-length target. Equations and
diagnostic detail will need deliberate movement to supplementary material or
compression before the full manuscript is assembled.

## Evidence Boundaries

- The path is frozen before motion in a static obstacle-clear workspace. MBF may
  provide a path only before a trial; there is no online obstacle avoidance or
  replanning claim.
- Independent RGB max-LCR over a frozen motion-plus-tail window is the physical
  outcome. Internal liquid state, IMU, and an online slosh monitor are mechanism
  signals and cannot replace RGB.
- Only final published commands update the linear and angular delay buffers.
  Solver proposals overwritten by supervision, execution-contract finalization,
  or safety are not
  treated as executed inputs.
- The phase-indexed set is empirical. The recovery branch uses a frozen bounded
  feedback law, but it is not a certified funnel or a recursive-feasibility or
  robust-recovery guarantee. Spill-free behavior and true signed liquid-phase
  observation are also not claimed.
