# `scout_dualsphysics_liquid`

Development-only, offline DualSPHysics liquid replay tooling for the
source-separated SIM-R8 simulator.

This directory is deliberately **not** a ROS package and is not part of the
Gazebo/controller launch graph.  The first integration stage runs only after a
SIM-R8 case has finished and its bag has been closed.  It may consume executed
motion from that immutable bag and produce a separate liquid-height report; it
must never publish commands or change the source attempt outcome.

## Hard boundary

```text
development_only = true
formal = false
fidelity_validation_status = UNVALIDATED
physical_primary_eligible = false
```

- Allowed repository code domain: `src/scout_apps/simulation/`.
- Approved external data root:
  `/data/a/scout_sim_replacement/r8_liquid`.
- `/data/a`, `/data`, the workspace build/devel trees and the R8 controller
  build prefix are never accepted as liquid output roots.
- No `sudo`, system install, `killall`, `pkill`, implicit cleanup or overwrite.
- No ROS/Gazebo process is started by the tools in this package.
- The dependency is pinned to an exact repository URL and commit before any
  network operation.

The design and staged admission gates are documented in
`docs/实物实验注意事项/对比试验/仿真接入液体/20260805_DualSPHysics物理液体接入SIM-R8方案.md`.

## Current implementation stage

The active implementation contains only:

1. a fail-closed filesystem/resource/process safety preflight;
2. exact-layout preparation under the approved root;
3. exact-commit DualSPHysics acquisition and source verification;
4. fail-closed P0-B VM/CPU-build/GenCase admission policies and receipts; and
5. no-ROS unit tests for those boundaries.

It does not yet contain a liquid case, motion exporter, solver runner, height
extractor, ROS bridge or formal evidence adapter.

The exact ESM-patched QEMU installation now has a separate append-only v2
install-state admission.  This admits only the observed package/filesystem
state; QEMU/qemu-img execution, image creation, VM start, build and GenCase
remain `NO_GO`.  The legacy v1 policy, gate, schema and receipts are preserved
byte-for-byte.  Neither gate has a VM-start, build or upstream-tool execution
subcommand, and the minimal CPU build recipe remains unauthorized.

Read-only self-check:

```bash
python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_safety.py self-check
```

No-ROS tests:

```bash
python3 -m unittest discover \
  -s src/scout_apps/simulation/scout_dualsphysics_liquid/tests -v
```

P0 state-changing commands use timestamped, create-new receipts under the
fixed manifest directory.  They refuse an existing final path or an unaudited
`.partial` path:

```bash
python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_safety.py \
  prepare-root \
  --receipt /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/safety_preflight_prepare_root_YYYYMMDDTHHMMSSZ.json

python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_dependency_gate.py \
  acquire \
  --preflight-receipt /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/dependency_acquire_preflight_YYYYMMDDTHHMMSSZ.json \
  --manifest-output /data/a/scout_sim_replacement/r8_liquid/dependency/manifests/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.json \
  --timeout-sec 300

python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_dependency_gate.py verify
```

`acquire` executes only the trusted host `/usr/bin/git` in its own process
group.  It does not execute files from the checkout.  A timeout stops only
that owned process group.  A failed acquisition is retained for explicit
audit and is never auto-cleaned.

Read-only P0-B checks:

```bash
python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_sandbox_gate.py \
  self-check --mode vm-host

python3 src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_sandbox_gate.py \
  self-check --mode cpu-build
```

Both commands are expected to return `NO_GO` until QEMU and the build recipe
receive separate admissions.  This is a safety result, not a test failure.

Read-only QEMU install-state v2 audit (does not invoke QEMU or qemu-img):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_qemu_install_gate_v2.py \
  self-check
```

After a read-only PASS and independent review, one create-new receipt may be
published under the fixed sandbox audit directory:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_qemu_install_gate_v2.py \
  write-receipt \
  --receipt /data/a/scout_sim_replacement/r8_liquid/audits/sandbox/qemu_install_preflight_v2_YYYYMMDDTHHMMSSZ.json
```

The v2 receipt is not reusable as a later VM start authorization.  Dynamic
host state must be checked again immediately before a future start under a
separate execution policy and lock.

The post-reboot, current-boot binary-probe gate is a narrower stage than a VM
smoke.  Its read-only check never invokes QEMU:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_qemu_binary_probe_gate.py \
  self-check
```

The only executable contract implemented by that gate is the exact binary
load/version argv `/usr/bin/qemu-system-x86_64 -no-user-config -version`.
The independently reviewed, current-boot probe completed once and published
the fixed singleton receipt
`/data/a/scout_sim_replacement/r8_liquid/audits/sandbox/qemu_binary_probe_v1_boot_20260805T094926Z.json`.
The receipt admits only the observed binary/version result.  Repeating
`run-probe` is now `NO_GO` because that exact create-new path exists.  The gate
has no configurable argv/path/environment and no qemu-img, machine, VM,
network, disk, KVM, build or GenCase entry.  The receipt explicitly does not
authorize another probe or any later stage.

Creating the approved directory layout and acquiring a dependency are
state-changing operations.  They require explicit subcommands and are not
performed by import, tests or `self-check`.

The next `-machine none` stage now has a separate fail-closed policy, three
strict schemas, a dedicated gate and mock-only tests.  Its read-only check is:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_qemu_machine_none_gate.py \
  self-check
```

The checked-in policy intentionally returns `NO_GO`: the running HWE kernel is
`5.15.0-139-generic`, while the enabled ESM repository offers the security
candidate represented by `linux-generic-hwe-20.04 5.15.0.186.196~20.04.1`.
Because this stage would newly exercise unprivileged user/network namespaces,
no namespace syscall or QEMU machine execution is admitted on the older boot.
Do not invoke `run-smoke`; the gate rejects it before creating its one-shot
attempt marker, forking, unsharing or executing QEMU.

After an explicitly approved kernel update and reboot, a new boot-bound policy
and independent review are required.  The current policy must not be edited to
flip its authorization boolean.  A future admitted run is one-shot: a fixed
`O_EXCL` attempt marker is fsynced before the first fork.  Any namespace/QEMU
attempt, including failure or power loss, permanently consumes that admission
and must not be automatically retried or have its marker deleted.

The frozen future contract is TCG-selected `-machine none`, no guest/vCPU,
disk, firmware, KVM, network backend, host mount or external namespace helper;
QMP is restricted to capability negotiation, `query-kvm`, `query-block`,
`query-status` and clean `quit`.  This stage claims network-namespace
isolation only.  It does not claim filesystem isolation and grants no authority
for qemu-img, images, a VM, GenCase, build, solver, ROS or Gazebo.
