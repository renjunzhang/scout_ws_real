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
- Legacy-target-only external data root:
  `/data/a/scout_sim_replacement/r8_liquid`.
- On `LIQUID_ZRJ_MSI_U2404`, that legacy root, its receipts and every
  state-changing legacy gate are forbidden.  The only target root is
  `/home/zrj/scout_liquid_lab`, governed by the separate target-host profile.
- `/data/a`, `/data`, the workspace build/devel trees and the R8 controller
  build prefix are never accepted as liquid output roots on either target.
- No `sudo`, system install, `killall`, `pkill`, implicit cleanup or overwrite.
- No ROS/Gazebo process is started by the tools in this package.
- The dependency is pinned to an exact repository URL and commit before any
  network operation.

The design and staged admission gates are documented in
`docs/实物实验注意事项/对比试验/仿真接入液体/20260805_DualSPHysics物理液体接入SIM-R8方案.md`.

## Current implementation stage

The active implementation contains:

1. a fail-closed filesystem/resource/process safety preflight;
2. exact-layout preparation under the approved root;
3. exact-commit DualSPHysics acquisition and source verification;
4. one separately admitted, offline materialization of only src/source/**
   into a sealed, non-executable source tree;
5. fail-closed P0-B VM/CPU-build/GenCase admission policies and receipts;
6. one narrowly admitted U3 C1 GenCase-only case generation, with immutable
   execution/lifecycle evidence; and
7. strict read-only BI4/XML/OUT cross-validation plus static-case visualization,
   with no-ROS unit tests for those boundaries.

It now contains one development-only U3 C1 static particle case. It does not
yet contain an admitted solver runner, motion exporter, time-series height
extractor, ROS bridge or formal evidence adapter.

### Separate MSI Ubuntu 24.04 target-host profile

`config/target_hosts/liquid_zrj_msi_u2404_profile_v1.json` is deliberately
separate from the legacy `/data/a` policy.  Its companion gate has exactly one
read-only command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_profile_gate.py \
  self-check
```

It verifies the narrow `/home/zrj/scout_liquid_lab` root, mount identity,
required layout, resources, tool presence and `bwrap --version`.  It has no
subcommand for creating a namespace, sandbox, receipt, source checkout, build
or upstream execution.  A `PASS_PROFILE_READ_ONLY_ONLY` result is explicitly
not build or GenCase authorization.

The next MSI-only admission is intentionally narrower still:
`liquid_zrj_msi_u2404_bwrap_system_true_smoke_policy_v1.json` can authorize
only `/usr/bin/true` inside an empty-root, network-less bwrap namespace. It
has no host writable bind, no source bind and no generic command argument. It
also requires `kernel.apparmor_restrict_unprivileged_userns=0` for this
unconfined invocation. If that state is nonzero, the gate returns `NO_GO`; do
not change AppArmor/sysctl state or use `sudo` to force a pass. A separately
reviewed profile or isolation backend is required before any further attempt.

`liquid_zrj_msi_u2404_source_fetch_policy_v1.json` is a separate, narrow
network admission.  Its `self-check` is read-only; its only state-changing
subcommand can create one new bare repository and fetch one frozen commit over
HTTPS.  It validates the root-owned `/usr/bin/git` binary and its resolved
HTTPS transport helper, disables credentials/hooks/LFS smudge/lazy fetching,
uses no checkout or submodules, and records a create-new receipt.  After the
fetch it performs both `git fsck --full --strict` and a bounded local
`rev-list --missing=print` walk: a single missing reachable object is `NO_GO`,
including a blob marked as a promisor object.  It never invokes an upstream
file, a build tool, ROS/Gazebo, or the GPU.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_source_fetch_gate.py \
  self-check
```

On 2026-08-06, its one permitted MSI attempt completed with
`PASS_FULL_BARE_SOURCE_FETCH` and `755` reachable objects / `0` missing
objects.  The resulting repository is still bare; the only next stage is a
separately admitted, offline static inventory.  This does not authorize
checkout, build, GenCase, solver, or a retry in the same directory.

That inventory now has its own MSI-only policy and mock-only tests:
`liquid_zrj_msi_u2404_source_static_inventory_policy_v1.json` and
`r8_liquid_target_source_static_inventory_gate.py`.  It disables every Git
protocol, validates a specific full-fetch receipt, parses the NUL-delimited
tree with hard size/path bounds, and streams only the frozen `bin/linux`
candidate object IDs to compute SHA-256 and check ELF magic.  No content is
checked out or written to a worktree.  The successful 2026-08-06 receipt
records 768 tree entries, no symlinks or gitlinks, and seven ELF paths that
remain `EXECUTION_FORBIDDEN`.  Its next stage is only a separately reviewed
offline ELF metadata/dependency policy.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_source_static_inventory_gate.py \
  self-check
```

That ELF metadata policy is now implemented as
`liquid_zrj_msi_u2404_elf_metadata_policy_v1.json` and
`r8_liquid_target_elf_metadata_gate.py`.  It has no external ELF parser:
each frozen blob is bounded in memory and parsed by range-checked Python for
ELF64 program/dynamic-table metadata only.  The successful receipt found that
`libdsphchrono.so` has
`RUNPATH=/root/libs/Chrono/build_centos8/lib` and depends on
`libChronoEngine.so`.  This is a hard `PRECOMPILED_ELF_EXECUTION_NO_GO`; none
of the repository-provided binaries or shared libraries may be run, loaded,
or transferred for execution.  The next action required human review of a
separate source-only build policy, not a retry or relaxation of this gate.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_elf_metadata_gate.py \
  self-check
```

That review has now produced the static-only
`liquid_zrj_msi_u2404_source_cpu_build_policy_v1.json`, its schema, and
`r8_liquid_target_source_cpu_build_policy_gate.py`.  This policy freezes the
future source-materialization root, read-only source bind, single writable
output bind, network/GPU/environment exclusions, resource limits, exact
Make/G++ argv, and the output dynamic-library allowlist.  It contains only a
read-only `self-check`; it has no checkout, materialization, sandbox, build,
receipt, or generic-command entry point.  Its canonical policy SHA-256 is
`bcad3eed1bda70e24e6e0bd317a2241eef54637b5fb83f1411f10cb1cf944dcb`.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_source_cpu_build_policy_gate.py \
  self-check
```

`PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY` means only that the checked-in
contract is internally consistent.  It explicitly does **not** authorize a
writable host bind, source checkout/materialization, CMake, Make, compiler,
output execution, GenCase, solver, ROS, Gazebo, GPU, or a build receipt.

The next narrower gate is
`liquid_zrj_msi_u2404_bwrap_output_bind_smoke_policy_v1.json` and
`r8_liquid_target_bwrap_output_bind_smoke_gate.py`.  Its only possible child
is the hashed `/usr/bin/touch`, which could create one empty `0600` marker in
one newly created output directory; no source is bound and no build tool has
an admitted argv entry.  The v1 policy explicitly requires
`kernel.apparmor_restrict_unprivileged_userns=0`.  On 2026-08-06 the observed
value is `1`, so its self-check returned `NO_GO_OUTPUT_BIND_SMOKE` and only
published the immutable no-go receipt
`/home/zrj/scout_liquid_lab/audits/u3_bwrap_output_bind_smoke_v1_20260806T111936Z.json`
(file SHA-256 `be0de443d5a25a8273366ad823a892a920ed574f46c258a0d39157bf668c24d0`).
No namespace, `touch`, output directory, checkout, source materialization, or
build was started.

The earlier v3 system-true gate now enforces that same AppArmor condition on
future preflights; its historical receipt remains unchanged.  Do not modify
AppArmor/sysctl, use `sudo`, or retry either gate to force a pass.  A reviewed
restricted AppArmor profile or a separately admitted isolation backend is
required before a future output-bind smoke, source-materialization policy, or
first build can be considered.

A separate MSI-only default-deny AppArmor review package now records that
future review boundary without changing the host.  It contains
`config/apparmor_drafts/r8-liquid-output-bind-smoke.profile`,
`liquid_zrj_msi_u2404_apparmor_output_bind_draft_policy_v1.json`, its schema,
`r8_liquid_target_apparmor_output_bind_draft_gate.py`, and mock-only tests.
The named-only profile has no attachment path and is marked
`DRAFT_ONLY: NOT_APPROVED_FOR_LOADING`.  It retains only a review-target
`userns,` rule and the future one-marker output path; it intentionally grants
no network, mount, capability, executable, broad-file, unconfined, workspace,
ROS, source, or precompiled-binary permission.  It is therefore deliberately
non-operational and cannot authorize bwrap.

Its read-only self-check returned
`PASS_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_ONLY`; the frozen policy canonical
SHA-256 is `7b6fb8ecc957e54e0fbffd142b75243274729797f6736638c7996e31975b4d9a`.
The check reads only the checked-in files.  It does not copy a profile into the
system, invoke a profile parser, load/select a profile, create a namespace,
write a receipt, or execute a command from the upstream source.

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_apparmor_output_bind_draft_gate.py \
  self-check
~~~

Only a reviewed patch/commit, hashes, and this static report may be shared with
the Gazebo host.  An administrator must independently review an exact narrow
mount/transition plan and approve a new profile revision plus a separate
execution admission before any system profile action or smoke retry.

Under a subsequent, explicit one-time local authorization, a **separate**
named-only profile was temporarily loaded from the workspace for a bounded
`aa-exec → bwrap → /usr/bin/true` enforcement probe, then immediately removed.
It has no attachment path and permits only the fixed executables plus the
owner-limited `/proc/*/{uid_map,gid_map,setgroups}` writes required while
creating the child user namespace.  It has no mount rule, host writable bind,
source/output access, stream socket, unconfined flag, ROS/Gazebo/GPU access, or
upstream executable rule.  The probe reached UID/GID mapping and then failed
closed when bwrap attempted `rslave` mount propagation for `/`; the AppArmor
audit recorded that mount denial.  No namespace completed, host writable mount,
network use, source materialization, build, or upstream execution occurred.

The profile was removed immediately afterwards; `aa-status` found zero matching
loaded profiles and the process table found no `bwrap` or `aa-exec` residue.
The resulting create-new audit receipt is
`/home/zrj/scout_liquid_lab/audits/u2_apparmor_userns_probe_v1_20260806T131601Z.json`
(SHA-256 `b36b9233815b6f1c68bbf85a68d68538f85f4fd7110e4ebe48f74638bde71123`).
This is user-namespace admission evidence only, not mount, output-bind, source
materialization, build, GenCase, solver, ROS, or GPU authorization.

`liquid_zrj_msi_u2404_apparmor_userns_probe_policy_v1.json`, its closed schema,
`r8_liquid_target_apparmor_userns_probe_gate.py`, and mock-only tests freeze
that result for two-host review.  The static gate cannot load a profile, invoke
a parser, start a process, create a namespace, or write a receipt:

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_apparmor_userns_probe_gate.py \
  self-check
~~~

The next admissible design activity is an independent review of an exact,
default-deny mount-confinement plan.  Do not add a bare `mount,`, a broad file
rule, or `flags=(unconfined)` and do not treat this record as permission to
replay the probe or run the output-bind smoke.

That review was subsequently advanced only through six transient, named-only
`/usr/bin/true` probes.  The v6 profile added just the exact internal
`pivot_root` transition to the preceding rslave/tmpfs/newroot/self-rbind/oldroot
setup rules; it did not permit a host bind, generic mount, remount, umount,
source/output path, upstream executable, ROS, GPU, or persistent installation.
Both observed v6 attempts stopped at the next AppArmor boundary, the temporary
`/oldroot/` `rprivate` propagation transition.  The first session was cleaned
without a terminal receipt; the one subsequent identical bounded observation
recorded the terminal result and removed the profile in the same session.  Its
create-new receipt is
`/home/zrj/scout_liquid_lab/audits/u2_apparmor_mount_probe_v6_20260806T144041Z.json`
(SHA-256 `fad9938b4111d99b2121ea7db73c07645d395f67facdec6cd3fb4310862bf48a`),
with status `PASS_EXACT_INTERNAL_ROOT_SETUP_OLDROOT_RPRIVATE_DENIED`.  Post-cleanup
matching-profile, `bwrap`/`aa-exec` process, and liquid/bwrap mount residue counts
are all zero.  This is only evidence for a future exact `/oldroot/ rprivate`
review; it does not authorize output-bind, source materialization, build,
GenCase, solver, ROS, Gazebo, GPU, or a repeat of the probe.

The subsequent empty-root v7--v11 sequence reached the fixed child-exec point
without an AppArmor denial; `/usr/bin/true` then correctly failed because no
host path was present in the synthetic root. A separate named-only
read-only-`/usr` probe was therefore introduced rather than extending the
mount probe into an output or source policy. Its v1--v8 progression admitted
only the audited synthetic `/newroot/usr/` setup, one exact rbind followed by
an exact `ro,nosuid,nodev` remount, self-profile cleanup signals, and the
single synthetic `lib64 -> usr/lib64` link required by the inspected ELF
interpreter. The final fixed `/usr/bin/true` command returned zero under new
user/pid/net/ipc/uts namespaces, with `/usr` as its only host input and no
host writable bind. The profile was removed in the same session; matching
profile, `bwrap`/`aa-exec`, and mount residue counts are zero.

The append-only evidence is
`/home/zrj/scout_liquid_lab/audits/u2_apparmor_ro_usr_probe_v1_v8_20260806T151136Z.json`
(SHA-256 `a738b035150bc03cd4d0f3448df1bc2d693414d4c3119fd74abff1272cbf7a95`),
with `PASS_RESTRICTED_READ_ONLY_USR_SYSTEM_TRUE_SMOKE`. It did not use
`--disable-userns`, bind an output/source/workspace path, or run an upstream
program. It is not a substitute for the historical unconfined v3 receipt and
does not authorize an output bind, source materialization, build, GenCase,
solver, ROS, Gazebo, GPU, or reuse under a different argv. The next design
stage is a separate static review for a one-marker writable-output bind.

That one-marker output-bind review is now complete under a separate named-only
profile series. A new empty `0700` directory under the liquid root was its only
writable host source. The v1--v6 audit progression admitted just the synthetic
`/work` and `/work/output` targets, one exact rbind of that empty directory,
the matching `rw,nosuid,nodev` remount, and exactly one output filename. The
fixed `/usr/bin/touch` command returned zero and created only
`.r8_output_bind_probe_v1`: a regular `0600`, zero-byte, single-link file owned
by `zrj`. `/usr` was the sole read-only host input; no source tree, workspace,
results, GPU, ROS, Gazebo, network, compiler, CMake, Make, or upstream binary
was available. Profile, process, and mount residue counts are zero after
cleanup.

The create-new evidence is
`/home/zrj/scout_liquid_lab/audits/u3_apparmor_output_bind_probe_v1_20260806T152757Z.json`
(SHA-256 `a6996cf4e685419c6762a9325b3968483610f6b52fa11645d5ce9202bb99abfa`),
with `PASS_RESTRICTED_ONE_MARKER_OUTPUT_BIND_SMOKE`. It did not use
`--disable-userns` and does not change the historical unconfined output-bind
gate's `NO_GO` result. It proves only this one marker write, not a second
output bind, source materialization, checkout, compiler, build/output
execution, GenCase, solver, ROS, Gazebo, GPU, or a changed argv. The next
design stage was a separate static admission for source materialization.

That separate source-materialization admission is now implemented by
liquid_zrj_msi_u2404_source_materialization_policy_v1.json, its closed schema,
r8_liquid_target_source_materialization_gate.py, and mock-only tests. Its
only state-changing subcommand accepts one timestamped fixed-format attempt
ID; it validates the frozen bare commit/tree and the three immutable U2
receipts, disables every Git protocol and user configuration, then uses only
fixed ls-tree and cat-file blob calls. It cannot run a source file, hook,
filter, CMake, Make, compiler, precompiled ELF, ROS/Gazebo process, GPU
program, or command selected by a caller.

The gate can materialize only src/source/** into a newly created
dependency/materialized/<attempt-id>.partial/src/source tree. It rejects
symlinks, gitlinks, hardlinks, paths outside that prefix, ELF magic, source
tree drift, existing paths, and any output write/execute bit. It first hashes
each copied blob and then seals the result as directories 0550 and files 0440;
the .partial suffix is intentionally retained until a later, separate
build-execution admission re-verifies it.

On 2026-08-06, attempt u3_source_materialization_v1_20260806T155752Z
completed with PASS_STATIC_SOURCE_MATERIALIZATION: 352 source files and
5,473,917 bytes, zero symlinks/hardlinks, no executable output bits, no
network, no checkout, no source/ELF execution, no CMake/Make/compiler, no
GPU, and no sudo. The final receipt is
/home/zrj/scout_liquid_lab/audits/u3_source_materialization_v1_20260806T155752Z.json
(file SHA-256
90af263fb7ec8b7d6a46a53aa5354dd5b676cd4167d48fde93911e014a70b745;
canonical receipt SHA-256
017a660bd38da43ae14fb97df9f19cc6cd2b90cf56f91ce047869dbf795d93e2).
Its only next stage is SEPARATE_BUILD_SANDBOX_EXECUTION_ADMISSION_REQUIRED;
the existing CPU build policy and every build/tool execution remain NO_GO.

Read-only admission check:

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_source_materialization_gate.py \
  self-check
~~~

The materialized tree is now bound to a second static-only AppArmor source CPU
build draft: r8-liquid-source-cpu-build-draft-v1.profile, its policy/schema,
r8_liquid_target_apparmor_source_cpu_build_draft_gate.py, and mock-only tests.
The named profile has no attachment path and is explicitly not approved for
loading. It names only the exact sealed source root and a future single build
output template, but deliberately grants no executable, mount, remount,
capability, network, transition, workspace, ROS, GPU, or source-write rule.
It therefore cannot run bwrap, copy source, invoke Make/G++, link, run
GenCase/solver, or execute an artifact.

Its read-only check verifies all 352 source hashes and the 0440/0550 sealed
tree against the immutable materialization receipt, the static CPU build
policy, and the one-marker output-bind evidence. It passed as
PASS_APPARMOR_SOURCE_CPU_BUILD_DRAFT_STATIC_ONLY, with policy canonical
SHA-256 ce33b2c3a040019dd9f6128f847aea893b7b67c625bc8d219ab91af180f29f13
and draft file SHA-256
58a9b8cdf9307b4588324a64f742ab0a633949c1996cc9ddcb01388bfb1607ca.
No profile was copied/parsed/loaded/selected; no namespace, source copy,
Make/compiler, output execution, network, GPU, sudo, or system configuration
action occurred.

~~~bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_apparmor_source_cpu_build_draft_gate.py \
  self-check
~~~

The next stage was an independent review of an exact build mount/transition
profile and two separately bounded source-copy and Make/compiler argv phases.
Its append-only outcome is recorded below; it does not authorize upstream
runtime execution.

### Target U3 CPU build: v15 static-audit pass, runtime still `NO_GO`

The later v13 and v14 fresh attempts compiled and linked but failed closed in
post-build inventory: v13 classified a normal retained `Makefile_cpu` object
as an unexpected source file, and v14 applied the single-link file rule to the
ordinary `buildtree/` directory. Their receipts and `.partial` trees remain
immutable evidence. v15 byte-pins v14 and changes only that narrow distinction:
directories must be real non-symlink directories (normal POSIX directory link
counts are allowed), while every output file remains regular, non-symlink and
single-link.

On 2026-08-07, fresh attempt
`u3_source_cpu_build_20260807T023724Z` completed both source-copy and CPU
build. The immutable build receipt is
`/home/zrj/scout_liquid_lab/audits/u3_source_cpu_build_20260807T023724Z_cpu_build.json`
(SHA-256 `d407233107d4bc9eeea81b0c5a95cbfc98ad6529f1cb7494b68ae5ecc4b1d604`),
with status `PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT`. It records a zero compiler/
linker return code, the exact 109 top-level x86-64 `ET_REL` objects, unchanged
host `user.max_user_namespaces`, no network/GPU/ROS/Gazebo or upstream binary
execution, and static PIE ELF checks on the generated candidate.

The candidate's SHA-256 is
`5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202` and
its mode is deliberately `0400`. It must not be executed, copied as a runtime
binary, or used to infer permission for GenCase, solver, precompiled
`bin/linux` tools, ROS, Gazebo or GPU. Both v15 AppArmor profiles were removed
in their respective temporary sudo sessions; no matching profile or compiler/
`bwrap`/`aa-exec` process remained and sudo credentials were cleared.

The only next stage is a **separate GenCase/solver runtime admission**. It must
fix the runtime executable provenance, case/input/output ABI, geometry and
motion inputs, resource budget, signal/timeout behavior, fresh-clone rule and
QC/settled-state contract before any upstream program is launched.

### Target U3 C1 GenCase seed v2: non-executing materialization pass

The first seed and its v1 runtime draft are permanently `NO_GO`. Independent
geometry review found that `mask="1 | 2"` removed both cylinder end caps and
left the C1 vessel without a bottom; `PartsOutMax` is also deprecated and
ignored by the pinned 5.4 source. The v1 seed, receipts, policy, profile and
gate remain append-only failure evidence and must never execute or become v2
inputs.

The corrected `u3_c1_static_v2.xml` uses `mask="2"`, retains the bottom and
side wall, and replaces the deprecated parameter with `MinFluidStop=1`. Its
SHA-256 is
`d738d303300b3f339ac37ca3a604fbe8dff9e034d8c8ad7aacb2654434525819`.

After an independent static `GO`, the one-shot non-executing v2 gate created
seed `u3_c1_gencase_seed_v2_20260807T135341Z`. Its immutable receipt is
`/home/zrj/scout_liquid_lab/audits/u3_c1_gencase_seed_v2_20260807T135341Z.json`
(SHA-256 `1bbf958dfe2f7ce026ce05d77e7ee2c2516c5d0ddc4345b021904e355003009d`),
with status `PASS_NONEXECUTABLE_GENCASE_SEED_V2_MATERIALIZATION`. It invoked
only the pinned system Git `cat-file blob` operation; no ELF, sudo, network,
namespace, AppArmor profile, GPU, ROS or Gazebo action occurred. The seed
directories are `0500`, and the exact GenCase, `DsphConfig.xml` and corrected
case files are regular, single-link host files with mode `0400`.

This pass still does not authorize GenCase. The next stage remains a fresh v2
runtime admission with guest-tmpfs input verification, bounded output,
bootstrap/runtime label separation and a separate cleanup lifecycle receipt.

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

That full discovery command includes legacy QEMU tests intentionally bound to
the old Ubuntu 20.04 `/data/a/scout_sim_replacement/r8_liquid` host and its
admitted `/usr/bin/qemu-img`.  Do **not** create that root or install QEMU on
the MSI target merely to make those legacy tests pass.  On
`LIQUID_ZRJ_MSI_U2404`, run the target-profile test and self-check instead:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest \
  src.scout_apps.simulation.scout_dualsphysics_liquid.tests.test_target_host_profile_gate -v
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

## 2026-08-07 append-only GenCase runtime admission status

This section supersedes no earlier receipt or failure record. A static-test
PASS means only that a checked-in design is internally consistent; it is not
evidence that a profile was loaded or that confinement, cleanup, or an
upstream executable was exercised.

### Production GenCase runtime v2: static PASS, explicit `NO_GO`

The production-target GenCase runtime v2 now has a separate policy/schema,
AppArmor profile draft, gate, bootstrap helper, root-supervisor draft, and
mock-only tests. Its static contract fixes stdin-only input framing, a 64 MiB
guest `/work` tmpfs, separate bootstrap/runtime labels, lowercase `rpx`, NNP
and capability checks, bounded process-group termination, one success-only
stdout frame, host-side `O_EXCL` publication, and separate execution and
lifecycle receipts.

The focused static/mock suite passed **13/13**. The gate and supervisor report
`PASS_STATIC_DRAFT_NO_GO` and `PASS_SUPERVISOR_STATIC_DRAFT_NO_GO`, while the
policy remains
`NO_GO_V2_PENDING_HARMLESS_RPX_SIGNAL_TIMEOUT_PROBE_AND_INDEPENDENT_STATIC_GO`.
No AppArmor profile was loaded, no namespace or mount was created, no bwrap,
GenCase, solver, or candidate ELF was executed, and no production attempt,
execution receipt, or lifecycle receipt was created. Parser and mock-test
success must not be presented as runtime evidence.

### stdio/AppArmor transport probe v2: static PASS, permanent `NO_GO`

The stdio-only harmless transport probe v2 policy/schema/profile draft,
read-only gate, and mock-only tests are frozen. Its focused static suite passed
**14/14**, and self-check reports
`PASS_STATIC_STDIO_APPARMOR_PROBE_PLAN_REMAINS_NO_GO`; the frozen status is
`STATIC_NO_GO_V2_MOUNT_RULES_UNOBSERVED_EXECUTION_FORBIDDEN`.

The v2 profile has no audit-observed mount/remount/pivot-root/unmount rules and
the package has no executable root supervisor, bounded AppArmor-audit capture,
or complete lifecycle verifier. It is therefore permanent `NO_GO`: do not
load it, run it, retry it, or reuse its probe ID, attempt paths, or profile
labels. This work did not parse or load that profile, run a harmless child,
create an attempt/receipt/snapshot, or touch GenCase, GPU, ROS, Gazebo, or the
network.

### Fresh v3 denial discovery: preparation only

A fresh v3 successor is being prepared with new IDs, paths, and labels for one
expected-denial discovery. Its design is intended to add an independent root
supervisor, a precise audit window, one sanitized matching denial, and explicit
profile/process/sudo-timestamp lifecycle verification. As of this record it
has **not run**: no profile load, sudo session, namespace, mount, harmless
runtime, attempt, receipt, or AppArmor denial is claimed.

Only static completion and independent review may occur next. Even a future
harmless-probe PASS would still require a fresh production-runtime revision
and a separate `GO`; it would not authorize GenCase or progression to U3/U4/U5.

### v3 pre-execution command-mode finding: permanent `NO_GO`

The v3 root snapshot was created at
`/run/r8-liquid-u3_stdio_apparmor_transport_probe_v3_20260807T144659Z.snapshot`
with a `0555` directory and root-owned `0444` files. Its frozen run/recover
templates incorrectly direct-executed the non-executable supervisor file and
would necessarily fail with `EACCES`. Preflight found this before profile load
or probe execution. No profile, namespace, mount, bwrap child, execution
receipt, or lifecycle receipt was created. The identity is permanently
`SNAPSHOT_CREATED_NOT_EXECUTED_COMMAND_MODE_NO_GO`; its snapshot and six
workspace artifacts remain preserved and must not be modified or reused.

### Fresh v4 successor: pinned Python entrypoint, static only

The distinct successor identity is
`u3_stdio_apparmor_transport_probe_v4_20260807T163041Z`, with separate v4
artifact names, labels, snapshot root, tokens, and receipt paths. Both root
entrypoints are exact arrays beginning with:

```text
/usr/bin/sudo /usr/bin/python3.12 -I -B <root-owned-0444-v4-snapshot-supervisor>
```

Direct execution of the snapshot supervisor is forbidden and covered by
negative policy/static tests. `verify_snapshot` continues to require the
snapshot directory to be root-owned `0555` and every file root-owned `0444`.
This is still static-only work: it does not claim that the v4 snapshot exists,
that a profile or probe ran, or that GenCase, solver, ROS, GPU, U4, or U5 is
authorized.

## 2026-08-08 U3 C1 GenCase and visualization result

The later fresh v3--v5 production identities failed closed and remain
append-only evidence. Fresh identity
`u3_c1_gencase_v6_20260808T072315Z` then completed the single admitted
GenCase-only execution and lifecycle cleanup. Its canonical lifecycle status
is `PASS_U3_C1_GENCASE_V6_LIFECYCLE_CLEANUP_AND_CASE_EXPORT`; the immutable
source case is under:

```text
/home/zrj/scout_liquid_lab/cases/u3_c1_gencase_v6_20260808T072315Z.partial/output
```

The `.partial` parent name is intentionally retained as provenance. The
published `C1_static.bi4`, `C1_static.xml`, and `C1_static.out` are regular,
single-link `0440` files. AppArmor reported zero matching/unexpected denials,
both temporary profiles were unloaded, process-label scans were stably empty,
the monitored sysctls were byte-identical, and the UID-1000 sudo timestamp was
cleared. This admission did not run a solver or authorize U4/U5.

`scripts/r8_liquid_bi4_reader_v1.py` and
`scripts/r8_liquid_u3_case_visualize_v1.py` read those three case products
without executing another DualSPHysics binary. They pin SHA-256, refuse final
symlinks and existing output directories, enforce bounded/strict BI4 parsing,
and cross-check particle counts, classes, `dp`, extents, zero static velocity,
density limits, and empty motion across BI4/XML/OUT. Focused coverage is in
`tests/test_bi4_reader_v1.py` and `tests/test_u3_case_visualize_v1.py`.

The accepted two-host handoff candidate is:

```text
/home/zrj/scout_liquid_lab/visualizations/u3_c1_gencase_v6_20260808T072315Z_v8
artifact_manifest.json sha256=03f56486d31a365b0d9f18af84f4ae3504e328c4379165a33f893adfef5e5d1a
visual_review_receipt.json sha256=9bc938670631640a1c83bd09b254ab7ba06a751a02b52e1be7500cdedcf6ab54
```

It contains `particles.csv`, JSON/Markdown validation reports, orthographic
PNG/PDF/SVG figures, grayscale checks, and a self-contained offline 3-D HTML
view. Programmatic layout QA reports zero issues and compliance reports zero
failures. Three font-embedding heuristic warnings were independently closed
with `pdffonts`, which reports embedded CID TrueType fonts; there are zero
unresolved warnings. All 23 manifest products match their recorded hashes,
sizes and final `0640` modes, and QA paths are package-relative. The separate
visual-review receipt has status
`PASS_U3_C1_GENCASE_V6_VISUAL_REVIEW`. For two-host handoff, transfer only the
manifest-pinned CSV/JSON/PNG/PDF/SVG/HTML products needed by the receiver,
never the workspace or a shared writable tree.

Visualization v5 remains valid image evidence but is superseded for handoff
because its manifest captured pre-hardening modes and its QA paths named an
ephemeral partial directory. v6 fixed the modes but not the paths; v7 stopped
before publication on a local variable-name error and its partial directory is
retained. v8 fixes both metadata issues and its six final PNG files are
byte-identical to the already visually reviewed v5 renderings.

That was the C1 visualization checkpoint. The later C1M solver result below
supersedes `U3_SOLVER_RUNTIME_NOT_ADMITTED` as the global status; this paragraph
remains the historical boundary of the C1 package only.

## 2026-08-09 U3 C1M CPU solver smoke, QC and visualization

Fresh GenCase identity `u3_c1m_gencase_v8_20260808T153753Z` produced the
restart-compatible zero-motion C1M case with 9078 particles: fixed 0, moving
2669, fluid 6409. The pinned inputs are:

```text
C1M_zero.bi4 b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7
C1M_zero.xml 28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb
```

Fresh identity `u3_c1m_solver_cpu_smoke_v3_20260808T160108Z` then completed
its only admitted CPU-only, single-core one-second run. Its exact 30-file output
contains 21 particle frames, zero excluded particles and the pinned
`PartMotionRef.ibi4` hash
`c6532b3b26ff22e0165ea64cb10688f23f0c99e2f17074bfd76c8410c5a95849`.
The execution and lifecycle receipt hashes are respectively
`8ff89dd45a6548d684e75b18ef4443f63728c5604a850e81da890df7c74d916f`
and `8d4541cfbba652dc0849dfaceb6ef1be2a876a0ee36136247365decef5f2ffe9`.
Both temporary profiles were unloaded, monitored sysctls were unchanged, the
sudo timestamp was cleared, and production/settled authorization remained
false.

`scripts/r8_liquid_u3_solver_output_qc_v3.py` reports
`C1M_ZERO_MOTION_SMOKE_PASS`: structural checks pass, but
`duration_eligible_for_settle_qc=false`, `tail_pass=false`, and
`numeric_settle_qc_pass=false`. The particle-q99 surface series is explicitly a
development proxy, not an admitted SWL gauge. Coverage for the derived package
is in `tests/test_u3_solver_smoke_visualize_v1.py`; the visualizer refuses
existing destinations, pins both case inputs and both lifecycle receipts, and
never executes DualSPHysics.

The accepted diagnostic handoff package is:

```text
/home/zrj/scout_liquid_lab/visualizations/u3_c1m_solver_cpu_smoke_v3_20260808T160108Z_v2
artifact_manifest.json sha256=d096e6663352b464646b81cea138797c98d86c8b6d351b2595a0b157cb003636
solver_output_qc_v3.json sha256=e008fe0201802899a77e2f50bf273ef18c7cef92ca6d9a4a6d33c17adc47ee3e
dashboard PNG sha256=c64a38f6a793b80d09b7c46036049595749ab3cf8fd5496b9022141f625a6e9e
```

It contains PNG/PDF/SVG and grayscale dashboards, derived CSV metrics, the full
QC report, SciPilot EDA/selection records, receipt validation and a portable
manifest. Layout QA has zero issues, external PNG review passes, grayscale
encodings remain distinguishable, and the Type0/CID font warning is closed by
an explicit descendant-font audit (`FontFile2`, subset, ToUnicode, non-Type3),
leaving zero unresolved warnings. v1 remains append-only history; its final
color/grayscale PNG bytes equal v2.

Current boundary: still U3. The consumed v3 identity does not authorize a
parameter change or rerun. Before U4, create independently reviewed, one-shot
`cold_a`/`cold_b` longer-settle identities, run them serially, and then validate
a separately admitted restart clone. Even success can establish only a
`U3_DEVELOPMENT_SETTLED_CANDIDATE / U4_SYNTHETIC_ONLY / SIM_ONLY_UNVALIDATED`
state until an admitted gauge and physical inputs exist. The Ubuntu 20.04
vehicle-motion bag is not needed until U5.
