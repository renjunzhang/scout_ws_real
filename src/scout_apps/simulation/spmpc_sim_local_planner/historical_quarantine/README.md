# R7/R1 historical execution quarantine

This directory preserves the minimum source record needed to explain the
superseded R7/R1 simulation work.  It is not part of the R8 execution source,
the installed ROS package, the R8 source-artifact registry, or any active test
discovery path.

The archived files have a `.disabled` suffix and are read-only.  Every archived
Python source contains a `QUARANTINED_R7_R1_ARCHIVE` fail-closed sentinel
before any operational import, so even an explicit
`python3 path/to/file.py.disabled` exits before it can start a historical
tool.  They must not be renamed back to
`.py`/`.launch`, imported by an active R8/H0/formal tool, or added to
`CMakeLists.txt` installation lists.  In particular, the archived R7
freeze/release/runner paths could inspect, hash, or build historical
`control/slosh_models` material; no source-separated simulation run may do
that.

Current executable surface is intentionally limited to the explicit
`SPMPC_SIM_ACTIVE_RUNTIME_SCRIPTS` and `SPMPC_SIM_ACTIVE_LAUNCH_FILES`
allowlists in the package `CMakeLists.txt`.  R8/H0/formal entry points must
continue to use only the simulation-owned controller, monitor, launch, and
source-separation tools listed there.

Archive layout:

- `scripts/`: former R7/R1 and physical-alignment helper entry points.
- `tests/`: their former unit/launch contract tests.
- `launch/`: the former R7 FixedProfile launch contract.
- `fixed_profile/`: the former profile generator and its helper modules; an
  active formal row may replay only a pre-generated, hash-bound read-only CSV.
- `config/`: historical 0705 fixture input, not an R8/H0 runtime setting.
- `bytecode/`: stale generated Python cache snapshots.  Their magic headers
  are intentionally invalidated and filenames no longer end in `.pyc`; the
  source archives above are the readable historical record.

This archive is historical context only.  It supplies no authority to run a
case, generate a release, build a controller, or claim a formal result.
