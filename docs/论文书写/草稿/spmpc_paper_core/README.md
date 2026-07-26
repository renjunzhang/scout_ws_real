# S-MPCC Method-and-Experiment Core

This directory contains the standalone IEEE journal-style core for the S-MPCC
method and experimental evidence chain. Its theme is prescribed-path trajectory
generation: a finite-horizon virtual path-progress MPCC uses an odometry-driven,
model-propagated internal liquid state to adapt chassis motion along supplied
geometry.

The paper-level organization is defined by
`../../论文组织思路/S-MPCC_当前论文组织思路.md`. The five-condition matrix in this
core is currently a redesign proposal, not an executable acquisition protocol.
The existing field protocol remains authoritative until the experimental design,
matrix, analysis rules, random tables, and freeze template are synchronously
superseded. Formal acquisition is therefore `NO-GO`; only a manifest template,
not an immutable `freeze_manifest.yaml/FREEZE_ID`, currently exists.

## Scope

The core deliberately contains only:

- `main.tex`: title, abstract, keywords, and document entry point;
- `sections/01_method.tex`: the reproducible S-MPCC formulation;
- `sections/02_experiments.tex`: the proposed RQ1--RQ4 evidence structure; and
- `supplementary/supplementary_material.tex`: freeze/configuration templates and
  supporting analyses.

Introduction, Related Work, and Conclusion are added in the full-paper tree only
after the method release and evidence scope are known. The core does not claim
global route planning, obstacle reasoning, true signed liquid-phase observation,
or a formal spill-free guarantee.

## Build

The normal local build is:

```bash
cd docs/论文书写/草稿/spmpc_paper_core
latexmk -pdf main.tex
```

For a clean verification build that does not place generated files beside the
source:

```bash
out="$(mktemp -d /tmp/spmpc-paper-build.XXXXXX)"
latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir="$out" main.tex
rg -n 'Warning|undefined|Overfull|Underfull|Error' "$out/main.log" "$out/main.blg"
```

The supplementary fragment is intentionally not included by `main.tex`; it must
be wrapped or included separately when the venue permits supplementary material.

## Current Experimental Design

The proposed matrix uses a provisional minimum of eight randomized blocks:

| Stage | Configuration | Conditions | Added | Cumulative |
| --- | --- | --- | ---: | ---: |
| I | C1 + H1 | Baseline, Smooth-only, Smooth-match, Fixed-profile, S-MPCC | 40 | 40 |
| II-A | C1 + L1 | Smooth-only, Fixed-profile, S-MPCC | 24 | 64 |
| II-B, conditional | C2 + H1 | Smooth-only, Fixed-profile, S-MPCC | 24 | 88 |

The value `n=8` is not yet a statistical guarantee. It must be frozen from the
minimum meaningful RGB effect, paired development variability, target
precision/power, and method/vision failure allowance. Stage II-B is a
preregistered conditional extension; 64 trials already form the preferred
two-path trajectory-generation evidence package.

The five high-risk conditions test four competing explanations:

1. base MPCC behavior;
2. generic smoothing;
3. uniform slowdown at matched completion time;
4. physics-aware fixed timing; and
5. model-state-conditioned online progress and chassis motion.

Fixed-profile is a transparent prior-art-inspired end-to-end comparator, not a
faithful Hamaguchi or Lim reproduction and not a single-factor memory ablation.

## Formal-Acquisition Gates

Before any formal block, the project must complete and archive:

- claim and comparator definition;
- current-versus-rotation-consistent release selection;
- limited weight-candidate screening;
- an independent RGB efficacy pilot for one final candidate, with an exact
  preregistered development block count and no early stopping;
- the odometry-based `s_proj` trajectory extractor and immutable phase/replay
  toolchain;
- Smooth-match and Fixed-profile fairness/tuning records;
- measurement, failure, estimand, sample-size, and randomization rules; and
- one immutable software/configuration release and `FREEZE_ID`.

The confirmatory order is S-MPCC versus Smooth-only, followed, only after that
gate passes, by S-MPCC versus Fixed-profile. Exact inference follows the actual
five-condition randomization support; paired sign-flip is only a sensitivity
analysis. Continuous effects are explicitly success-conditional and are reported
with `n_pair/n`, success margins, and a failure-penalized sensitivity.

## Evidence Boundaries

Actual trajectory timing is reconstructed from odometry-derived path projection
`s_proj`; the OCP variable `s_ocp` is never used as executed ground truth.
`H_vis` is the calibrated experimental reference and `H_modal` is an internal
model response. Online-input/zero-state replay tests whether the propagated
internal state changes optimization; it does not validate the true liquid phase
or a counterfactual physical outcome.

The K6-style modal--vision population is 8/16/24 S-MPCC planned units after
Stage I/II-A/II-B. Long-horizon propagation, physical parameter mismatch, and
broad planner rankings are optional exploratory studies with no reserved formal
count and no role in the 40/64/88 packages.
