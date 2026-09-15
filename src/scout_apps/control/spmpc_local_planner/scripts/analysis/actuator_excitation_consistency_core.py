"""Conditional command replay and matched IMU processing, without ROS or fitting.

The recorded OCP uses sampled FIFO/RK4 dynamics. Command replay uses the
existing continuous FOPDT helper. Their difference is not purely replanning:
publication timing and discretization also differ. No physical parameters or
time shifts are estimated here.
"""
from collections import Counter
from dataclasses import asdict, dataclass

import numpy as np

from analyze_ocp_imu_forecast import require, unique_cycles, validate_cycle
from robot_state_prediction_core import (
    ROBOT_FIELDS, check_command_support, check_snapshot_history, command_at,
    replay_robot, snapshot_contract, strict_rows,
)


AXES = ("ax", "ay", "omega", "alpha")
INTERVENTIONS = (
    "terminal_phase", "terminal_controller_intervened", "safety_gate_intervened",
    "zero_due_to_speed_safety", "command_contract_violation", "linear_limited",
    "angular_rate_limited", "angular_accel_limited",
)
IMU_FIELDS = (
    "measurement_stamp", "source_stamp", "receive_stamp", "accel_effective_stamp",
    "gyro_effective_stamp", "alpha_effective_stamp", "sample_dt_sec",
    "ax_mps2", "ay_mps2", "omega_z_radps", "alpha_z_radps2",
    "accel_filtered_base_x_mps2", "accel_filtered_base_y_mps2",
)


@dataclass(frozen=True)
class ImuProcessing:
    accel_cutoff_hz: float
    gyro_cutoff_hz: float
    sensor_delay_sec: float
    accel_phase_delay_sec: float
    gyro_phase_delay_sec: float
    alpha_phase_delay_sec: float
    lever_arm_imu_to_target_x_m: float
    lever_arm_imu_to_target_y_m: float

    @classmethod
    def from_launch(cls, values):
        prefix = "/spmpc_local_planner/imu_shadow/"
        data = {name: float(values[prefix + name]) for name in cls.__dataclass_fields__}
        require(all(np.isfinite(v) for v in data.values()), "nonfinite IMU configuration")
        require(data["accel_cutoff_hz"] > 0 and data["gyro_cutoff_hz"] > 0,
                "invalid IMU cutoff")
        require(all(data[name] >= 0 for name in data if name.endswith("delay_sec")),
                "negative IMU delay")
        return cls(**data)


class ImuSamples:
    """Keep native sensor intervals; never interpolate across gaps or resets."""
    def __init__(self, rows, processing, max_gap):
        self.rows, self.segments, self.counts = [], [], Counter()
        segment, previous = 0, None
        rx = processing.lever_arm_imu_to_target_x_m
        ry = processing.lever_arm_imu_to_target_y_m
        for row in rows:
            good = (all(row.get(k) for k in ("valid", "configured", "filter_ready", "bias_ready"))
                    and row.get("source") == 2
                    and all(np.isfinite(row.get(k, np.nan)) for k in IMU_FIELDS)
                    and row["measurement_stamp"] > 0 and row["sample_dt_sec"] > 0)
            if not good:
                previous = None
                segment += 1
                self.counts["invalid"] += 1
                continue
            require(row.get("excitation_axes_frame") == "base_link" and
                    row.get("excitation_reference_point") == "liquid_observer_target_icr_proxy",
                    "unsupported IMU axes/reference point")
            require(row["receive_stamp"] >= row["measurement_stamp"], "IMU receive precedes measurement")
            require(abs(row["source_stamp"] - row["measurement_stamp"] -
                        processing.sensor_delay_sec) < 2e-6, "IMU sensor timestamp/config mismatch")
            for name in ("accel", "gyro", "alpha"):
                require(abs(row["measurement_stamp"] - row[name + "_effective_stamp"] -
                            getattr(processing, name + "_phase_delay_sec")) < 2e-6,
                        "IMU effective timestamp/config mismatch")
            # Check the recorded operator/lever arm before applying it to models.
            omega, alpha = row["omega_z_radps"], row["alpha_z_radps2"]
            expected = [row["accel_filtered_base_x_mps2"] - alpha*ry - omega**2*rx,
                        row["accel_filtered_base_y_mps2"] + alpha*rx - omega**2*ry]
            require(np.allclose(expected, [row["ax_mps2"], row["ay_mps2"]],
                                rtol=1e-6, atol=1e-7), "IMU lever-arm/config mismatch")
            if self.rows:
                last = self.rows[-1]
                if row["measurement_stamp"] == last["measurement_stamp"]:
                    keys = (*IMU_FIELDS, "reset_epoch", "observer_update_count")
                    require(all(row.get(k) == last.get(k) for k in keys), "conflicting IMU duplicate")
                    self.counts["identical_duplicate"] += 1
                    continue
                require(row["measurement_stamp"] > last["measurement_stamp"], "IMU clock moved backwards")
            if previous is not None:
                dt = row["measurement_stamp"] - previous["measurement_stamp"]
                if (dt > max_gap or abs(dt-row["sample_dt_sec"]) > 2e-6 or
                        row.get("reset_epoch") != previous.get("reset_epoch") or
                        row.get("observer_update_count") != previous.get("observer_update_count", -2)+1):
                    segment += 1
                    self.counts["continuity_break"] += 1
            self.rows.append(row)
            self.segments.append(segment)
            previous = row
        require(len(self.rows) >= 2, "insufficient valid IMU samples")
        self.times = np.array([r["measurement_stamp"] for r in self.rows])
        self.max_gap = max_gap

    def window(self, epoch, end):
        # The initial filter state must have been received before the forecast.
        first = int(np.searchsorted(self.times, epoch, side="right"))
        seeds = [i for i in range(max(0, first-10), first)
                 if self.rows[i]["receive_stamp"] <= epoch]
        require(bool(seeds), "no causal IMU filter seed")
        seed = seeds[-1]
        require(epoch-self.times[seed] <= self.max_gap, "stale IMU filter seed")
        require(seed == first-1, "unavailable intermediate IMU filter sample")
        last = int(np.searchsorted(self.times, end, side="right"))
        require(last > first and end-self.times[last-1] <= self.max_gap,
                "incomplete future IMU window")
        require(self.segments[seed] == self.segments[last-1], "future crosses IMU gap or reset")
        return self.rows[seed], self.rows[first:last]


def process_model(raw, rows, seed, params):
    """Replay the planar part of ProcessedImuPipeline on native sensor stamps.

    raw columns are target ax, ay, omega, alpha. Transform target -> nominal
    IMU point, filter accel/gyro, differentiate filtered gyro, then apply the
    same nominal IMU -> target lever arm. Gravity/bias/yaw calibration accuracy
    is an input assumption, not validated by this comparison.
    """
    rx, ry = params.lever_arm_imu_to_target_x_m, params.lever_arm_imu_to_target_y_m
    filtered = np.array([seed["accel_filtered_base_x_mps2"],
                         seed["accel_filtered_base_y_mps2"], seed["omega_z_radps"]])
    previous, result = seed["measurement_stamp"], []
    for motion, row in zip(raw, rows):
        ax, ay, omega, alpha = motion
        dt = row["measurement_stamp"]-previous
        accel_imu = np.array([ax+alpha*ry+omega**2*rx, ay-alpha*rx+omega**2*ry])
        beta_a = -np.expm1(-2*np.pi*params.accel_cutoff_hz*dt)
        beta_w = -np.expm1(-2*np.pi*params.gyro_cutoff_hz*dt)
        old_omega = filtered[2]
        filtered[:2] += beta_a*(accel_imu-filtered[:2])
        filtered[2] += beta_w*(omega-filtered[2])
        alpha_filtered = (filtered[2]-old_omega)/dt
        result.append([filtered[0]-alpha_filtered*ry-filtered[2]**2*rx,
                       filtered[1]+alpha_filtered*rx-filtered[2]**2*ry,
                       filtered[2], alpha_filtered])
        previous = row["measurement_stamp"]
    return np.array(result)


def recorded_plan_motion(horizon, params, times):
    """Dense within-interval FOPDT evaluation anchored to recorded OCP nodes."""
    dt, epoch = horizon["dt"], horizon["solver_input_epoch"]
    relative = np.asarray(times)-epoch
    require(np.all(relative >= 0) and np.all(relative <= horizon["t"][-1]+1e-8),
            "plan extrapolation")
    indices = np.minimum(np.floor(relative/dt+1e-10).astype(int), horizon["horizon_steps"])
    phase = np.maximum(0, relative-indices*dt)
    v0, w0 = np.asarray(horizon["v"])[indices], np.asarray(horizon["omega"])[indices]
    tv = params["actuator_gain_v"]*np.asarray(horizon["delayed_v_cmd"])[indices]
    tw = params["actuator_gain_omega"]*np.asarray(horizon["delayed_omega_cmd"])[indices]
    v = tv+(v0-tv)*np.exp(-phase/params["actuator_tau_v"])
    w = tw+(w0-tw)*np.exp(-phase/params["actuator_tau_omega"])
    return np.column_stack([(tv-v)/params["actuator_tau_v"], v*w,
                            w, (tw-w)/params["actuator_tau_omega"]])


def command_motion(snapshot, params, commands, times):
    initial = [snapshot["robot_"+name] for name in ROBOT_FIELDS]
    states = replay_robot(initial, snapshot["solver_input_epoch"], times, commands, params)
    delayed_v = np.array([command_at(commands, t-params["delay_v"])[0] for t in times])
    delayed_w = np.array([command_at(commands, t-params["delay_omega"])[1] for t in times])
    v, w = states[:, 3], states[:, 4]
    return np.column_stack([(params["actuator_gain_v"]*delayed_v-v)/params["actuator_tau_v"],
                            v*w, w, (params["actuator_gain_omega"]*delayed_w-w)/params["actuator_tau_omega"]])


def comparison_metrics(rows):
    result = {"count": len(rows)}
    if not rows:
        return result
    for left, right in (("ocp", "imu"), ("replay", "imu"), ("ocp", "replay")):
        axes = {}
        for axis in AXES:
            a = np.array([r[left+"_"+axis] for r in rows])
            b = np.array([r[right+"_"+axis] for r in rows])
            error = a-b
            axes[axis] = {"bias": float(np.mean(error)), "mae": float(np.mean(abs(error))),
                          "rmse": float(np.sqrt(np.mean(error**2))),
                          "correlation": float(np.corrcoef(a, b)[0, 1])
                          if min(np.std(a), np.std(b)) > 1e-10 else None}
        result[left+"_vs_"+right] = axes
    return result


def analyze(horizons, snapshots, audits, imu_rows, processing, windows, height_coeff,
            duration=2.0, min_lead=0.1, max_imu_gap=0.035, max_command_gap=0.1, cycle_id=None):
    require(np.isfinite([duration, min_lead, max_imu_gap, max_command_gap]).all() and
            0 <= min_lead < duration <= 2 and max_imu_gap > 0 and max_command_gap > 0,
            "invalid analysis intervals")
    monitor = ImuSamples(imu_rows, processing, max_imu_gap)
    snap, duplicate_s = unique_cycles(snapshots)
    audit, duplicate_a = unique_cycles(audits)
    hor, duplicate_h = unique_cycles(horizons)
    require(not duplicate_a, "duplicate audit cycle: ambiguous command history")
    published = [a for a in audits if a.get("command_was_published")]
    commands = strict_rows([[a["command_publish_stamp"], a["published_cmd_v"], a["published_cmd_omega"]]
                            for a in published], 3, "published command audit")
    task_start, task_end = windows["task"]
    matches, origins, rejected = [], [], []
    for ident, h in hor.items():
        if cycle_id is not None and ident != cycle_id:
            continue
        try:
            require(ident not in duplicate_s | duplicate_h, "duplicate snapshot/horizon cycle")
            require(ident in snap and ident in audit, "missing snapshot or audit")
            s, a = snap[ident], audit[ident]
            validate_cycle(h, s, a, height_coeff)
            require(not any(a.get(k) for k in INTERVENTIONS), "origin output intervention")
            epoch = h["solver_input_epoch"]
            require(task_start <= epoch < task_end, "origin outside task")
            require(all(abs(row[key]-epoch) < 1e-6 for row in (h, s)
                        for key in ("robot_state_stamp", "liquid_state_stamp")),
                    "initial state epoch mismatch")
            params = snapshot_contract(s)
            check_snapshot_history(s, commands)
            for name in ("v", "omega", "a_actual", "alpha_actual", "delayed_v_cmd", "delayed_omega_cmd"):
                require(len(h[name]) == h["horizon_steps"]+1 and np.isfinite(h[name]).all(),
                        "malformed motion horizon")
            require(np.allclose([h["v"][0], h["omega"][0]], [s["robot_v"], s["robot_omega"]],
                                atol=1e-7, rtol=0), "robot x0 mismatch")
            node_motion = recorded_plan_motion(h, params, epoch+np.asarray(h["t"]))
            require(np.allclose(node_motion[:, [0, 3]], np.array([h["a_actual"], h["alpha_actual"]]).T,
                                atol=1e-6, rtol=1e-6), "recorded acceleration/model mismatch")
            seed, samples = monitor.window(epoch, epoch+duration)
            times = np.array([r["measurement_stamp"] for r in samples])
            check_command_support(commands, epoch-max(params["delay_v"], params["delay_omega"]),
                                  times[-1], max_command_gap)
            raw_ocp = recorded_plan_motion(h, params, times)
            raw_replay = command_motion(s, params, commands, times)
            ocp = process_model(raw_ocp, samples, seed, processing)
            replay = process_model(raw_replay, samples, seed, processing)
            future = [r for r in published if r["cycle_id"] != ident and
                      epoch < r["command_publish_stamp"] <= times[-1]]
            intervened = sum(any(r.get(k) for k in INTERVENTIONS) or
                             (r.get("solve_attempted") and not r.get("solve_success")) for r in future)
            origin_rows = []
            for i, sample in enumerate(samples):
                effective = [sample["accel_effective_stamp"], sample["gyro_effective_stamp"],
                             sample["alpha_effective_stamp"]]
                if min(effective) < max(epoch+min_lead, h["horizon_available_stamp"], a["command_publish_stamp"]):
                    continue
                row = dict(cycle_id=ident, solver_input_epoch=epoch, measurement_stamp=times[i],
                           accel_effective_stamp=effective[0], gyro_effective_stamp=effective[1],
                           alpha_effective_stamp=effective[2], lead_sec=effective[0]-epoch,
                           window="task" if max(effective) <= task_end else "cross_goal",
                           future_interventions=int(intervened),
                           future_replans=sum(bool(r.get("solve_attempted")) for r in future))
                actual = [sample["ax_mps2"], sample["ay_mps2"], sample["omega_z_radps"], sample["alpha_z_radps2"]]
                for j, axis in enumerate(AXES):
                    row.update({"ocp_"+axis: float(ocp[i, j]), "replay_"+axis: float(replay[i, j]),
                                "imu_"+axis: float(actual[j]), "raw_ocp_"+axis: float(raw_ocp[i, j]),
                                "raw_replay_"+axis: float(raw_replay[i, j])})
                origin_rows.append(row)
            require(bool(origin_rows), "no future samples after availability/minimum lead")
            matches.extend(origin_rows)
            origins.append(dict(cycle_id=ident, solver_input_epoch=epoch, actuator_parameters=params,
                                causal_filter_seed_stamp=seed["measurement_stamp"],
                                future_interventions=int(intervened), metrics=comparison_metrics(origin_rows)))
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            rejected.append({"cycle_id": ident, "reason": str(exc)})
    task = [r for r in matches if r["window"] == "task"]
    report = dict(status="DIAGNOSTIC_COMPLETE" if matches else "INCONCLUSIVE", origins=origins,
                  rejected_origins=rejected, rejection_counts=dict(Counter(r["reason"] for r in rejected)),
                  imu_counts=dict(monitor.counts),
                  groups={"task": comparison_metrics(task),
                          "task_no_future_intervention": comparison_metrics([r for r in task if not r["future_interventions"]]),
                          "cross_goal": comparison_metrics([r for r in matches if r["window"] == "cross_goal"])},
                  method=dict(duration_sec=duration, min_lead_sec=min_lead,
                              max_imu_gap_sec=max_imu_gap, max_command_gap_sec=max_command_gap,
                              selected_cycle_id=cycle_id, imu_processing=asdict(processing),
                              command_source="control_cycle_audit.command_publish_stamp and published_cmd_*",
                              prediction="recorded OCP nodes; within-interval FOPDT dense evaluation",
                              replay="continuous FOPDT with actual published ZOH commands; not bitwise OCP replay",
                              measurement_operator="nominal target-to-IMU lever arm, native-dt one-pole accel/gyro, gyro difference, IMU-to-target lever arm",
                              model_target="base_link, assumed coincident with nominal liquid_observer_target_icr_proxy",
                              filter_seed="last valid received IMU filter state at/before forecast epoch",
                              time_shift_fitted=False, parameters_fitted=False,
                              limitations=["Nominal geometry, gravity/bias correction and physical timestamp calibration remain unverified.",
                                           "Overlapping rolling windows are not independent trials or liquid ground truth.",
                                           "Processing similarity does not identify a unique cause or validate a calibration."]))
    return report, matches
