#!/usr/bin/env python3
"""Single source of truth for the legacy-v1 and short100-v2 ABBA profiles.

The hardware runner consumes this module through its ``shell`` output, while
the Python analyzer/tests import the same profile objects.  Keeping identity,
row mapping, artifact suffixes and liquid-cost expectations here prevents a
shell/Python split-brain without making the generic fixed-path runner aware of
this experiment.
"""

import argparse
import shlex
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, Optional


@dataclass(frozen=True)
class StrictRuntimeContract:
    """Environment values that a fail-closed hardware profile must pin."""

    solver_backend: str
    cmd_topic: str
    reference_path_topic: str
    costmap_topic: str
    reference_target_frame: str
    base_frame: str
    imu_topic: str
    imu_ready_topic: str
    observer_selection_topic: str
    odom_topic: str
    mocap_tracker: str
    w_smooth: float
    w_alpha: float
    w_du_a: float
    w_du_vs: float
    slosh_height_max: float
    alpha_max: float
    observer_max_imu_state_age_sec: float
    observer_max_odom_state_age_sec: float
    observer_max_future_skew_sec: float
    liquid_nowcast_max_prediction_sec: float
    liquid_nowcast_max_excitation_age_sec: float
    liquid_nowcast_max_future_skew_sec: float
    liquid_nowcast_max_state_excitation_skew_sec: float
    liquid_nowcast_max_integration_step_sec: float
    state_timing_max_raw_skew_sec: float
    state_timing_max_interpolation_gap_sec: float
    state_timing_max_robot_extrapolation_sec: float
    execution_contract_max_delta_v: float
    execution_contract_max_delta_omega: float
    shared_linear_accel_max: float
    shared_angular_rate_max: float
    shared_angular_accel_max: float
    recorder_startup_sec: float
    recorder_active_timeout_sec: float
    planner_startup_sec: float
    path_source_startup_sec: float
    path_publish_rate_hz: float
    variant_timeout_sec: float


@dataclass(frozen=True)
class AbbaProfile:
    profile_id: str
    protocol_id: str
    version_label: str
    output_tag: str
    run_label_prefix: str
    runner_selector_mode: str
    treatment_variant: str
    treatment_cost_horizon_steps: int
    treatment_cost_tail_discount: float
    exact_report_suffix: str
    observer_report_suffix: str
    chain_report_suffix: str
    rgb_report_suffix: str
    unit_pass_suffix: str
    rgb_analysis_report_name: str
    rgb_analysis_report_type: str
    exact_report_schema: str
    minimum_p95_improvement_mm: float
    minimum_rms_improvement_mm: float
    maximum_slowdown_ratio: float
    require_fresh_session: bool
    session_marker_name: str
    supersedes_protocol: str
    legacy_source_commit: str
    operator_note: str
    strict_runtime_contract: Optional[StrictRuntimeContract]


@dataclass(frozen=True)
class AbbaRow:
    row: str
    block: str
    position: str
    condition: str
    condition_label: str
    stem_condition: str
    pilot_method: str
    variant: str
    w_slosh: float
    slosh_enabled: bool
    observer_applied: str
    cost_horizon_steps: int
    cost_tail_discount: float


LEGACY_SOURCE_COMMIT = "41f0937831aab2edfb82d579c6ed853b7919556b"

SHORT100_STRICT_RUNTIME = StrictRuntimeContract(
    solver_backend="continuous_mpcc_acados",
    cmd_topic="/cmd_vel",
    reference_path_topic="/scout/global_path_fixed",
    costmap_topic="/map",
    reference_target_frame="map",
    base_frame="base_link",
    imu_topic="/imu/data",
    imu_ready_topic="/spmpc/debug/slosh_observer_imu",
    observer_selection_topic="/spmpc/debug/slosh_observer_selection",
    odom_topic="/odom",
    mocap_tracker="Tracker0",
    # Pin resolved values explicitly.  Using the launch ``-1`` sentinel here
    # would make the preregistration look as if the solver used negative
    # weights, and would defer a YAML/fallback mismatch until postflight.
    w_smooth=0.1,
    w_alpha=0.1,
    w_du_a=0.1,
    w_du_vs=0.1,
    slosh_height_max=0.001,
    alpha_max=1.2,
    observer_max_imu_state_age_sec=0.10,
    observer_max_odom_state_age_sec=0.50,
    observer_max_future_skew_sec=0.005,
    liquid_nowcast_max_prediction_sec=0.050,
    liquid_nowcast_max_excitation_age_sec=0.060,
    liquid_nowcast_max_future_skew_sec=0.005,
    liquid_nowcast_max_state_excitation_skew_sec=0.001,
    liquid_nowcast_max_integration_step_sec=0.020,
    state_timing_max_raw_skew_sec=0.080,
    state_timing_max_interpolation_gap_sec=0.050,
    state_timing_max_robot_extrapolation_sec=0.010,
    execution_contract_max_delta_v=0.0001,
    execution_contract_max_delta_omega=0.0001,
    shared_linear_accel_max=0.6,
    shared_angular_rate_max=1.2,
    shared_angular_accel_max=1.2,
    recorder_startup_sec=8.0,
    recorder_active_timeout_sec=15.0,
    planner_startup_sec=2.0,
    path_source_startup_sec=2.0,
    path_publish_rate_hz=2.0,
    variant_timeout_sec=5.0,
)

PROFILES: Dict[str, AbbaProfile] = {
    "legacy_v1": AbbaProfile(
        profile_id="legacy_v1",
        protocol_id="SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1",
        version_label="i0_failclosed_fixed_abba_dev_v1",
        output_tag="spmpc_i0_failclosed_fixed_abba",
        run_label_prefix="DEV_I0FC_FIXED",
        runner_selector_mode="pilot_method",
        treatment_variant="B_slosh",
        treatment_cost_horizon_steps=-1,
        treatment_cost_tail_discount=1.0,
        exact_report_suffix="_i0_fixed_postflight.json",
        observer_report_suffix="_observer_postflight.json",
        chain_report_suffix="_mocap_chain_postflight.json",
        rgb_report_suffix="_i0_fixed_rgb_postflight.json",
        unit_pass_suffix="_unit_pass.env",
        rgb_analysis_report_name="I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json",
        rgb_analysis_report_type="I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS",
        exact_report_schema="spmpc_i0_failclosed_fixed_abba_postflight_v1",
        minimum_p95_improvement_mm=0.05,
        minimum_rms_improvement_mm=0.0,
        maximum_slowdown_ratio=1.05,
        require_fresh_session=False,
        session_marker_name="",
        supersedes_protocol="",
        legacy_source_commit=LEGACY_SOURCE_COMMIT,
        operator_note=(
            "new dev profile 0.20/0.25; literal B0/B_slosh; "
            "I0 source then legacy L22 fixed rollout"
        ),
        strict_runtime_contract=None,
    ),
    "short100_v2": AbbaProfile(
        profile_id="short100_v2",
        protocol_id="SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2",
        version_label="i0_failclosed_fixed_short100_abba_dev_v2",
        output_tag="spmpc_i0_failclosed_fixed_short100_abba_v2",
        run_label_prefix="DEV_I0FC_FIXED_S100_V2",
        runner_selector_mode="direct_variant",
        treatment_variant="B_slosh_short100",
        treatment_cost_horizon_steps=3,
        treatment_cost_tail_discount=0.0,
        exact_report_suffix="_i0_fixed_short100_v2_postflight.json",
        observer_report_suffix="_short100_v2_observer_postflight.json",
        chain_report_suffix="_short100_v2_mocap_chain_postflight.json",
        rgb_report_suffix="_i0_fixed_short100_v2_rgb_postflight.json",
        unit_pass_suffix="_short100_v2_unit_pass.env",
        rgb_analysis_report_name=(
            "I0_FAILCLOSED_FIXED_SHORT100_ABBA_RGB_ANALYSIS.json"
        ),
        rgb_analysis_report_type=(
            "I0_FAILCLOSED_FIXED_SHORT100_ABBA_RGB_ANALYSIS"
        ),
        exact_report_schema=(
            "spmpc_i0_failclosed_fixed_short100_abba_postflight_v2"
        ),
        # Preserve the v1 rapid-screen threshold so the only runtime treatment
        # change is the liquid-cost window.  The protocol document explicitly
        # treats 0.05--0.10 mm as directional, not measurement-resolved efficacy.
        minimum_p95_improvement_mm=0.05,
        minimum_rms_improvement_mm=0.0,
        maximum_slowdown_ratio=1.05,
        require_fresh_session=True,
        session_marker_name=(
            "SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2_session.env"
        ),
        supersedes_protocol="SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1",
        legacy_source_commit=LEGACY_SOURCE_COMMIT,
        operator_note=(
            "short100 v2; literal B0/B_slosh_short100; liquid cost stages "
            "0..3 only; I0 source then legacy L22 fixed rollout"
        ),
        strict_runtime_contract=SHORT100_STRICT_RUNTIME,
    ),
}


ROW_LAYOUT = {
    "01": ("01", "01", "B0"),
    "02": ("01", "02", "Bslosh"),
    "03": ("02", "01", "Bslosh"),
    "04": ("02", "02", "B0"),
}


def get_profile(profile_id: str) -> AbbaProfile:
    try:
        return PROFILES[profile_id]
    except KeyError as exc:
        raise ValueError(
            "unknown ABBA profile {!r}; choose {}".format(
                profile_id, ",".join(sorted(PROFILES))
            )
        ) from exc


def resolve_row(profile: AbbaProfile, row: str) -> AbbaRow:
    try:
        block, position, condition = ROW_LAYOUT[row]
    except KeyError as exc:
        raise ValueError("row must be one of 01,02,03,04") from exc

    if condition == "B0":
        variant = "B0"
        pilot_method = "B0" if profile.runner_selector_mode == "pilot_method" else ""
        return AbbaRow(
            row=row,
            block=block,
            position=position,
            condition=condition,
            condition_label="B0",
            stem_condition="B0",
            pilot_method=pilot_method,
            variant=variant,
            w_slosh=0.0,
            slosh_enabled=False,
            observer_applied="none",
            cost_horizon_steps=-1,
            cost_tail_discount=1.0,
        )

    pilot_method = "W5" if profile.runner_selector_mode == "pilot_method" else ""
    short = profile.profile_id == "short100_v2"
    return AbbaRow(
        row=row,
        block=block,
        position=position,
        condition=condition,
        condition_label="Bslosh-short100" if short else "Bslosh",
        stem_condition="BsloshS100" if short else "Bslosh",
        pilot_method=pilot_method,
        variant=profile.treatment_variant,
        w_slosh=5.0,
        slosh_enabled=True,
        observer_applied="L22",
        cost_horizon_steps=profile.treatment_cost_horizon_steps,
        cost_tail_discount=profile.treatment_cost_tail_discount,
    )


def iter_rows(profile: AbbaProfile) -> Iterable[AbbaRow]:
    for row in sorted(ROW_LAYOUT):
        yield resolve_row(profile, row)


def canonical_run_label(profile: AbbaProfile, row: AbbaRow) -> str:
    return "{}_{}_{}_b{}_p{}_a01".format(
        profile.run_label_prefix,
        row.row,
        row.stem_condition,
        row.block,
        row.position,
    )


def previous_run_label(profile: AbbaProfile, row: AbbaRow) -> str:
    previous = {"01": None, "02": "01", "03": "02", "04": "03"}[row.row]
    if previous is None:
        return ""
    return canonical_run_label(profile, resolve_row(profile, previous))


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


def shell_values(profile: AbbaProfile, row: Optional[AbbaRow]) -> Dict[str, str]:
    values = {
        "PROFILE_ID": profile.profile_id,
        "PROTOCOL_ID": profile.protocol_id,
        "VERSION_LABEL": profile.version_label,
        "OUTPUT_TAG": profile.output_tag,
        "RUN_LABEL_PREFIX": profile.run_label_prefix,
        "RUNNER_SELECTOR_MODE": profile.runner_selector_mode,
        "TREATMENT_VARIANT": profile.treatment_variant,
        "TREATMENT_COST_HORIZON_STEPS": str(
            profile.treatment_cost_horizon_steps
        ),
        "TREATMENT_COST_TAIL_DISCOUNT": str(
            profile.treatment_cost_tail_discount
        ),
        "EXACT_REPORT_SUFFIX": profile.exact_report_suffix,
        "OBSERVER_REPORT_SUFFIX": profile.observer_report_suffix,
        "CHAIN_REPORT_SUFFIX": profile.chain_report_suffix,
        "RGB_REPORT_SUFFIX": profile.rgb_report_suffix,
        "UNIT_PASS_SUFFIX": profile.unit_pass_suffix,
        "RGB_ANALYSIS_REPORT_NAME": profile.rgb_analysis_report_name,
        "RGB_ANALYSIS_REPORT_TYPE": profile.rgb_analysis_report_type,
        "EXACT_REPORT_SCHEMA": profile.exact_report_schema,
        "MINIMUM_P95_IMPROVEMENT_MM": str(
            profile.minimum_p95_improvement_mm
        ),
        "MINIMUM_RMS_IMPROVEMENT_MM": str(
            profile.minimum_rms_improvement_mm
        ),
        "MAXIMUM_SLOWDOWN_RATIO": str(profile.maximum_slowdown_ratio),
        "REQUIRE_FRESH_SESSION": _bool_text(profile.require_fresh_session),
        "SESSION_MARKER_NAME": profile.session_marker_name,
        "SUPERSEDES_PROTOCOL": profile.supersedes_protocol,
        "LEGACY_SOURCE_COMMIT": profile.legacy_source_commit,
        "OPERATOR_NOTE_FROZEN": profile.operator_note,
        "STRICT_RUNTIME_CONTRACT": _bool_text(
            profile.strict_runtime_contract is not None
        ),
    }
    runtime = profile.strict_runtime_contract
    if runtime is not None:
        for field, value in runtime.__dict__.items():
            values["RUNTIME_{}".format(field.upper())] = str(value)
    if row is not None:
        values.update(
            {
                "PAIR_ROW": row.row,
                "BLOCK": row.block,
                "POSITION": row.position,
                "CONDITION": row.condition,
                "CONDITION_LABEL": row.condition_label,
                "STEM_CONDITION": row.stem_condition,
                "PILOT_METHOD": row.pilot_method,
                "VARIANT": row.variant,
                "W_SLOSH": str(row.w_slosh),
                "SLOSH_ENABLED": _bool_text(row.slosh_enabled),
                "OBSERVER_APPLIED": row.observer_applied,
                "EXPECTED_COST_HORIZON_STEPS": str(
                    row.cost_horizon_steps
                ),
                "EXPECTED_COST_TAIL_DISCOUNT": str(
                    row.cost_tail_discount
                ),
                "CANONICAL_RUN_LABEL": canonical_run_label(profile, row),
                "PREVIOUS_RUN_LABEL": previous_run_label(profile, row),
            }
        )
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, choices=sorted(PROFILES))
    parser.add_argument("--row", choices=sorted(ROW_LAYOUT))
    parser.add_argument("--format", choices=("shell",), default="shell")
    args = parser.parse_args()
    try:
        profile = get_profile(args.profile)
        row = resolve_row(profile, args.row) if args.row else None
    except ValueError as exc:
        parser.error(str(exc))
    for key, value in shell_values(profile, row).items():
        print("I0FC_{}={}".format(key, shlex.quote(value)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
