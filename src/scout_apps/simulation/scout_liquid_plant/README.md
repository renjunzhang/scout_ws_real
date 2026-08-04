# `scout_liquid_plant`

This is a standalone, development-only multi-mode liquid-plant surrogate for
the isolated simulator. It keeps its own modal state and equations and does
not use a controller implementation or state. Its only ROS subscription is
the actual simulator odometry topic, `/odom`.

It publishes the following record-only signals:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/sim_truth/liquid_height` | `std_msgs/Float64` | Unvalidated surrogate crest elevation above static level, metres. |
| `/sim_truth/liquid_state` | `std_msgs/Float64MultiArray` | Timestamp, executed-base excitation, height summary, then four states per mode. The field order is carried in the array layout and metadata. |
| `/sim_truth/liquid_metadata` | latched `std_msgs/String` JSON | Input/output contract, parameter/code hashes, status and state-field order. |

## Hard status boundary

Every accepted configuration requires and publishes:

```text
development_only = true
formal = false
fidelity_validation_status = UNVALIDATED
physical_primary_eligible = false
```

The `/sim_truth` namespace is a routing name, not a fidelity claim. Until a
pre-registered H0-to-RGB validation demonstrates amplitude, frequency,
damping, phase and ranking behaviour, these outputs must not be used as a
formal physical-primary outcome. They must never be supplied to a planner,
tracker, command gate or observer.

`C1_development_unvalidated.yaml` and `C2_development_unvalidated.yaml` are
illustrative parameter templates only. They are explicitly not frozen C1/C2
physical manifests and cannot satisfy a formal experiment gate.

## Dynamics and information boundary

```text
/odom (executed pose, body velocity, yaw rate)
  -> finite-difference world acceleration, rotated into the body frame
  -> container offset / turn-rate terms
  -> independently implemented nonlinear multi-mode RK4 plant
  -> /sim_truth/liquid_height + state + metadata
```

No raw command topic is subscribed. A timestamp reversal or a gap larger than
the configured `max_odom_dt_sec` is not integrated; the state topic records
the rejection while the next valid odometry sample re-establishes the input
baseline. This prevents a paused simulator from becoming an artificial high
acceleration impulse.

The formal message contract is in
[`schema/liquid_plant_io_schema_v1.json`](schema/liquid_plant_io_schema_v1.json).

## Build and development integration

After sourcing the ROS distribution in the workspace:

```bash
cd /home/a/scout_ws
catkin_make --pkg scout_liquid_plant
source devel/setup.bash
python3 -m unittest discover -v \
  -s src/scout_apps/simulation/scout_liquid_plant/tests
```

At the time this package was added, the existing workspace build cache had a
package whitelist containing only `spmpc_local_planner`. That cache will not
discover this new package until the build selection is intentionally updated
to include `scout_liquid_plant`, or the package is built in a separate catkin
workspace. This package does not alter that user build configuration itself.

If this package has been intentionally built in a separate/selected catkin
workspace, a manual development-only launch alongside an already fresh
isolated ROS/Gazebo stack is:

```bash
roslaunch scout_liquid_plant liquid_plant_development.launch \
  config:=$(rospack find scout_liquid_plant)/config/C1_development_unvalidated.yaml
```

The fail-closed H0 adapter is the preferred integration path.  Its explicit
`prepare --with-development-liquid-plant` option validates one of the two
checked-in `*_development_unvalidated.yaml` templates, hashes node/core/I-O
schema/parameters into the case manifest, and starts this node as direct
`python3` with the package `src/` added to `PYTHONPATH`.  It does not depend on
this package being in the workspace's catkin whitelist and does not modify
that whitelist.  The adapter records all `/sim_truth/liquid_*` outputs only
when that opt-in is present; normal H0 runs remain unchanged.

Use the C2 template only by passing its explicit file path. Each fresh run
creates a fresh node and zeroes the plant state at node start. Record all three
topics plus `/odom` and `/clock`; keep bags outside the repository.

Before any development run is interpreted, inspect the live graph at ready,
pre-motion and postflight. The recorder and metrics nodes may subscribe, but
the controller/planner/tracker/command-gate node set must have no subscription
under `/sim_truth/`. A future formal release also requires immutable plant
code/parameter/I-O/fidelity artifacts and the independent formal freeze
inputs; this package intentionally supplies none of those approvals.

No ROS or Gazebo process was started while adding this package.

## Offline development fidelity verifier

`liquid_plant_fidelity_verify.py` is an offline, hash-bound comparison tool.
It reads exported CSV (`time_sec,value`) or JSON signal files and reports the
five required dimensions: amplitude, frequency, damping, phase and
cross-case amplitude ranking. It never starts ROS or Gazebo.

The command requires an absolute, hash-bound comparison manifest and an
absolute, hash-bound threshold policy. Every comparison case supplies a plant
signal, reference signal and a separate hash-bound reference-evidence JSON.
The evidence is accepted only when it identifies an independent, frozen real
RGB/liquid-sensor measurement with a freeze ID, source bag hash, extraction
pipeline hash and calibration hash. `/slosh/height`, `/spmpc/slosh_height`,
`H_proxy`, `H_modal` and `/sim_truth/*` are explicitly rejected as reference
sources.

```bash
python3 src/scout_apps/simulation/scout_liquid_plant/scripts/liquid_plant_fidelity_verify.py \
  --comparison-manifest /absolute/path/comparison_manifest.json \
  --comparison-manifest-sha256 <manifest_sha256> \
  --threshold-policy /absolute/path/threshold_policy.json \
  --threshold-policy-sha256 <policy_sha256> \
  --output /absolute/path/development_fidelity_report.json
```

The report ABI is
[`schema/liquid_plant_fidelity_report_v1.json`](schema/liquid_plant_fidelity_report_v1.json).
Its shape deliberately includes the future gate's identity fields
(`plant_code_hash`, parameter and I/O hashes, five dimension statuses and
input artifact hashes), but this implementation is hard-coded to write:

```text
formal = false
development_only = true
fidelity_validation_status = UNVALIDATED_DEVELOPMENT_ONLY
physical_primary_eligible = false
```

Thus even a numerically complete development comparison has status
`DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL`, not `PASS`; a missing/malformed or
non-real/non-frozen reference produces a schema-valid `NO_GO` report and exit
status 2. It cannot be placed in a formal freeze or used as a
physical-primary outcome.

## Formal evidence intake (not a formal plant release)

`liquid_plant_formal_evidence_intake.py` is a separate, offline evidence
assembler for a future reviewed plant release.  It deliberately does **not**
reuse the development verifier, generate a fidelity report, launch ROS or
Gazebo, or turn the checked-in development configuration into a formal
artifact.  In particular, it rejects the two
`*_development_unvalidated.yaml` templates, the current development I/O and
fidelity schemas/verifier, `UNVALIDATED`, `/slosh/height` (`H_proxy`),
`/spmpc/slosh_height` (`H_modal`) and `LiquidSloshModel` evidence sources.

Its one input is an absolute, SHA-256-bound JSON request with exactly these
artifacts:

| Request field | Required external artifact |
| --- | --- |
| `formal_release_manifest` | Frozen formal release binding the code, parameters and separate input/output schemas. |
| `plant_code` | Independently reviewable formal plant source, not the current surrogate. |
| `plant_parameters` | Frozen formal parameter document with initial-state-rule hash and integration step. |
| `plant_input_schema` / `plant_output_schema` | Formal I/O contracts: only executed `/odom` enters; only read-only `/sim_truth/liquid_height` leaves. |
| `fidelity_report` | Independently produced formal report with all five dimensions PASS. |
| `controller_isolation_evidence` | Static plus live ROS-graph evidence at ready, pre-motion and postflight. |
| `external_approval` | Hash-bound external approval of the exact release, fidelity, isolation and reference-evidence set. |

The formal fidelity report must bind at least two real-reference cases for
the ranking test.  Each case is a hash-bound external evidence record for a
`REAL_RGB_LIQUID_HEIGHT` or `REAL_LIQUID_HEIGHT_SENSOR` measurement, with its
frozen signal, source bag, extraction pipeline, calibration and freeze ID.
For the same case set it must also bind a hash-bound
`SMPCC_SIM_FORMAL_LIQUID_PLANT_SIGNAL_EVIDENCE` record: the exact
`/sim_truth/liquid_height` plant signal and formal run manifest, both bound to
the release's code/parameter/I-O hashes.  This prevents a report from hiding
an `H_proxy` or `H_modal` trace behind a copied signal file.
It must explicitly declare `formal=true`, `development_only=false`,
`independently_produced=true`, `fidelity_validation_status=PASS`, and PASS for
amplitude, frequency, damping, phase and ranking.  The approval repeats and
cross-binds all of those hashes; it is not synthesized by this package.

When every check succeeds, use:

```bash
python3 src/scout_apps/simulation/scout_liquid_plant/scripts/liquid_plant_formal_evidence_intake.py \
  --intake-request /absolute/path/formal_liquid_plant_intake_request.json \
  --intake-request-sha256 <request_sha256> \
  --capability-report-output /absolute/new/formal_liquid_plant_capability_report.json \
  --toolchain-binding-output /absolute/new/liquid_plant_capability_binding.json
```

Both outputs must be new paths; they are written read-only.  The first is an
evidence-intake report and the second is the mapping to place at
`liquid_plant_capability` in the formal freeze.  The binding contains the
existing toolchain ABI (`plant_*_path/hash`, `fidelity_report_path/hash`, and
`plant_capability_report_path/hash`) plus immutable provenance fields:

```text
formal_intake_tool_id
formal_intake_request_path / formal_intake_request_hash
formal_release_manifest_path / formal_release_manifest_hash
external_approval_path / external_approval_hash
controller_isolation_evidence_path / controller_isolation_evidence_hash
fidelity_verifier_source_path / fidelity_verifier_source_hash
formal_reference_evidence / formal_reference_evidence_set_hash
formal_plant_signal_evidence / formal_plant_signal_evidence_set_hash
formal_intake_report_path / formal_intake_report_hash
```

`formal_intake_report_path/hash` exactly equal
`plant_capability_report_path/hash`.  The formal toolchain must require and
cross-bind these provenance fields; a bare handwritten `PASS` report is not a
replacement for this intake.  `--validate-only` verifies the same evidence
without creating any output.  On every failure the command exits 2 and writes
no new capability/binding artifact.

The repository currently has no pinned approval public key or organizational
trust store.  The assembler therefore always records
`cryptographic_trust_anchor=NOT_CONFIGURED` and
`external_approval_authentication_status=NOT_INDEPENDENTLY_AUTHENTICATED`.
It verifies structural hash binding and never claims to authenticate the
issuer.  A user/organization-supplied approval process remains mandatory for
formal use; adding a real trust anchor is a separate, explicitly reviewed
release action.
