# Simulation controller fork provenance

This directory is a frozen, source-level simulation fork.  It is not an
include-path overlay, a symlink, or a link-time wrapper around the real robot
controller.

## Origin snapshot

- Source repository commit: `73495b0ef50ce6bced955db275e178f22cde0ac2`.
- Origin header tree: `5b530020bd9fd331697c1b61b160b92302dfecd2`.
- Origin C++ source tree: `5d095eb49a40e53e21db73ebf0dbc935d0f6cf1e`.
- Origin message tree: `d1c3d7a2a95dcc1fbd11bca36c7cd6790ac6e531`.
- Copied ACADOS asset-tree content digest: `a17c73f1de66a58cd84e555f7f0e46c5a1191c0e88a9f2758661e7335f8c1cac`.
- Current fork C++/message/solver-tree content digest: `d195fb9018e93320d1c2d599c3f48d43d7ea7d905527669042a82cacf41cd837`.

The origin commit is recorded for review only.  No CMake, launch, runtime
script, generated-solver path, or ROS package dependency resolves into
`src/scout_apps/control/spmpc_local_planner`.

## Deliberate fork changes

- ROS package, generated messages, include root and C++ namespace are
  `spmpc_sim_local_planner`.
- The controller node is `/sim_spmpc_local_planner`; diagnostics are rooted at
  `/sim_spmpc/*`, not `/spmpc/*`.
- Simulation variants, container declarations, release gate and launch files
  are owned by this package.
- ACADOS solver assets are local copies under `generated/acados/`; their
  historical model symbols are retained only to keep the frozen solver ABI.
- The simulation controller's modal proxy kernel is compiled from
  `src/dynamics/slosh_dynamics.cpp` in this fork.  It is a frozen local
  implementation of the published modal equations and has no build, link, or
  include dependency on `control/slosh_models`.  `/slosh/height` remains a
  non-primary H_proxy; this source separation does not turn it into plant
  truth.

`tests/test_source_isolation.py` is the machine-checkable boundary: it rejects
physical-package CMake/package dependencies, symlinks, old include/message
paths, a physical diagnostics root, and stale binary RUNPATHs.

Historical R7 artifacts remain historical shared-target evidence.  This fork
starts a new source-separated R8 release boundary and does not relabel R7.
