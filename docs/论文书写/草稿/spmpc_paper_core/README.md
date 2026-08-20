# Phase-Rejoining Residual S-MPCC Core

This directory is a standalone IEEE journal-style draft of the method and
experimental core for **Phase-Rejoining Residual S-MPCC**. Its central rule is:
an online residual correction is admitted only when the unequal-delay prediction
remains eligible to rejoin the complete phase-indexed offline anti-slosh tail.

This is a target formulation and evidence plan, not a released physical result.
The paper-level organization is maintained in
`../../论文组织思路/S-MPCC_当前论文组织思路.md`.

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
- `sections/01_method.tex`: offline artifact, unequal-delay alignment,
  neighboring phase selection, residual OCP, joint terminal gate, and
  supervisor;
- `sections/02_experiments.tex`: RQ1--RQ5, C0--C4, A0--A3, G0--G4, RGB outcome,
  recovery evaluation, and runtime/command-chain audit;
- `figures/phase_rejoining_framework_paper.{svg,pdf,png}`: editable and
  publication-ready architecture figure used by the method section; and
- `supplementary/supplementary_material.tex`: a repository/supplement fragment
  containing release-freeze and analysis templates.

The supplementary fragment is not included by `main.tex`. It relies on the
packages and macros in `main.tex` and must be wrapped or included by a venue-
specific supplementary document.

## Current Release Status

Physical `enforce` is **NO-GO**. In particular:

- **B0 / G0 is open:** the current development front predictor does not retain
  the new-command dependence between unequal linear and angular delays and also
  mixes the physical front epoch with its grid index;
- **G1 is pending:** no formal OfflineSloshOCP artifact with a validated
  motion--settle--zero-hold tail and complete contract has been frozen;
- **G2 is pending:** the source-correct RGB motion-plus-tail measurement and its
  detectability rules are not yet frozen;
- **G3 is NO-GO:** the formal execution-compatibility set and independent
  held-out gate test do not yet exist; and
- **G4 is pending:** the frozen paired pilot and complete recorder/postflight
  chain have not passed.

Development branch coverage, no-motion tests, or internal slosh-model trends do
not establish physical anti-slosh efficacy. No positive physical performance
claim is made in this core.

## Comparison and Ablation Identifiers

The system-level conditions are:

| ID | Frozen interpretation |
| --- | --- |
| C0 OrdinaryMPCC | Liquid-agnostic contour/lag/progress MPCC and the common downstream safety chain. |
| C1 SmoothMatch | C0 with independently tuned smoothing and one global scale used to match C4 completion time. |
| C2 OfflineReplay | Clocked replay of the same formal offline artifact as C4, with zero online residual. |
| C3 ResidualNoGate | C4's aligned residual OCP with the empirical gate, execution compatibility, and stored action disabled together. |
| C4 Full | Unequal-delay augmentation, neighboring phase, bounded residual, joint terminal gate, stored action, and fail-closed supervision. |

C0--C4 are a system evidence ladder, not a sequence of single-factor changes.
The primary comparison is C4 versus C0; only after it passes is C4 versus C1
tested confirmatorily. The nominal main matrix uses C0, C1, C2, and C4 on the
frozen P-core path. C3 is paired with C4 in controlled recovery experiments.

Component attribution uses strict pairs:

| ID | Pair | Sole planned change |
| --- | --- | --- |
| A0 | OfflineSmoothReplay vs. C2 | Offline liquid objective off/on. |
| A1 | PhaseAlignedNoResidual vs. C3 | Finite online residual off/on. |
| A2 | C3 vs. C4 | Joint gate, execution compatibility, and stored action off/on. |
| A3 | IdealExec vs. IdentifiedExec | Idealized versus identified unequal-delay execution model and consistent artifact. |

Formal sample size is not fixed in this repository draft. The obsolete fixed-
count matrix is not used; sample size and randomization are frozen from the RGB
minimum detectable difference, SESOI, independent pilot variability, and failure
allowance.

## Release Gates

| Gate | Required evidence | Current status |
| --- | --- | --- |
| G0 | Causal unequal-delay front, grid-consistent epochs, complete-lead prediction, and detectable first-action influence. | **NO-GO: B0 open** |
| G1 | Formal OfflineSloshOCP artifact and complete path/model/container/schema contract. | Pending |
| G2 | Frozen RGB calibration, source timing, motion-plus-tail window, validity, and detectability. | Pending |
| G3 | Trial-separated gate construction/test, execution compatibility, conditional false-safe analysis, and independent recovery labels. | **NO-GO** |
| G4 | Frozen paired pilot, condition separation, tracking, RGB direction, supervisor branches, recorder, and postflight audit. | Pending |

All five gates require numerical criteria, immutable data identities, and a
declared failure action before formal acquisition.

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

The current clean standalone build is **8 pages**. That count already covers
only the local Sections I/II, title/abstract, and references; it excludes the
full paper's Introduction, Related Work, and Conclusion. It is therefore a page-
budget warning, not a submission-length target. Equations, freeze tables, and
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
  Solver proposals overwritten by supervision, limiting, or safety are not
  treated as executed inputs.
- The phase-indexed set is empirical and the stored action is not a feedback
  recovery policy. Recursive feasibility, robust recovery, spill-free behavior,
  and true signed liquid-phase observation are not claimed.
