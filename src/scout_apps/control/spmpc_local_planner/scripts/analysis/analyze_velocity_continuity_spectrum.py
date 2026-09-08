#!/usr/bin/env python3
"""Offline stage-local spectrum audit of direct-command chassis continuity bags.

Reads bags only; no ROS node, publisher, parameter access, or MPC snapshots.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

import analyze_mocap_velocity_continuity as continuity
import analyze_mocap_velocity_step as step
import continuity_spectrum_core as spectral


PHASES = ("pre_zero", "ramp_up", "hold", "ramp_down", "post_zero")
TOPICS = {"cmd": "/cmd_vel", "stamped": continuity.STAMPED_CMD_TOPIC,
          "imu": "/imu/data", "odom": "/odom", "mocap": "/vrpn_client_node/Tracker0/pose"}
LABELS = {"cmd": ["vx", "vy", "vz", "wx", "wy", "wz"],
          "stamped": ["vx", "vy", "vz", "wx", "wy", "wz"],
          "imu": ["ax", "ay", "az", "wx", "wy", "wz", "roll", "pitch", "yaw"],
          "mocap": ["x", "y", "z", "roll", "pitch", "yaw", "qx", "qy", "qz", "qw"],
          "odom": ["vx", "wz", "x", "y", "yaw"]}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-8:
        return (float("nan"),) * 3
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(np.clip(2*(w*y-z*x), -1, 1)),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def twist_values(twist):
    return [getattr(vector, axis) for vector in (twist.linear, twist.angular) for axis in "xyz"]


def load_raw(bag_path):
    import rosbag
    rows = {key: [] for key in TOPICS}
    lookup = {topic: key for key, topic in TOPICS.items()}
    connections, topic_info = [], {}
    with rosbag.Bag(str(bag_path)) as bag:
        topic_info = {topic: {"type": info.msg_type, "count": info.message_count}
                      for topic, info in bag.get_type_and_topic_info().topics.items()}
        for connection in bag._get_connections():
            caller = connection.header.get("callerid", b"")
            connections.append({"topic": connection.topic, "callerid": caller.decode() if isinstance(caller, bytes) else caller})
        for topic, msg, received, header in bag.read_messages(topics=list(TOPICS.values()), return_connection_header=True):
            key = lookup[topic]
            native = msg.header.stamp.to_sec() if key != "cmd" else received.to_sec()
            caller = header.get("callerid", b"")
            common = {"header_sec": native, "bag_sec": received.to_sec(),
                      "seq": msg.header.seq if key != "cmd" else -1,
                      "frame_id": msg.header.frame_id if key != "cmd" else "HEADERLESS_BAG_TIME_ONLY",
                      "callerid": caller.decode() if isinstance(caller, bytes) else caller}
            if key == "cmd":
                values = twist_values(msg)
            elif key == "stamped":
                values = twist_values(msg.twist)
            elif key == "imu":
                values = [getattr(msg.linear_acceleration, a) for a in "xyz"] + [getattr(msg.angular_velocity, a) for a in "xyz"] + list(rpy(msg.orientation))
            elif key == "mocap":
                values = [getattr(msg.pose.position, a) for a in "xyz"] + list(rpy(msg.pose.orientation)) + [getattr(msg.pose.orientation, a) for a in ("x", "y", "z", "w")]
            else:
                values = [msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                          msg.pose.pose.position.x, msg.pose.pose.position.y, step.quaternion_yaw(msg.pose.pose.orientation)]
            rows[key].append({**common, **dict(zip(LABELS[key], map(float, values)))})
    return rows, connections, topic_info


def timing_stats(rows, channels):
    t = np.array([r["header_sec"] for r in rows])
    b = np.array([r["bag_sec"] for r in rows])
    dt = np.diff(t)
    seq = np.array([r["seq"] for r in rows])
    positive = dt[dt > 0]
    return {"count": len(t), "invalid_header_count": int(np.sum(t <= 0)),
            "duplicate_header_count": int(np.sum(dt == 0)), "header_regressions": int(np.sum(dt < 0)),
            "seq_regressions": int(np.sum(np.diff(seq) < 0)), "seq_gap_count": int(np.sum(np.diff(seq) > 1)),
            "rate_hz": float((len(t)-1)/(t[-1]-t[0])),
            "dt_ms": {str(p): float(np.percentile(positive, p)*1000) for p in (0, 1, 5, 50, 95, 99, 100)},
            "arrival_minus_header_ms": {str(p): float(np.percentile(b-t, p)*1000) for p in (0, 5, 50, 95, 99, 100)},
            "frames": sorted(set(r["frame_id"] for r in rows)),
            "channels": {c: {"unique_values": len(set(r[c] for r in rows)),
                              "nonfinite_count": int(np.sum(~np.isfinite([r[c] for r in rows]))),
                              "unchanged_neighbor_fraction": float(np.mean(np.diff([r[c] for r in rows]) == 0))} for c in channels}}


def mirror_audit(raw):
    mirrors = raw["stamped"]
    callers = set(r["callerid"] for r in mirrors)
    actual = [r for r in raw["cmd"] if r["callerid"] in callers]
    extras = [r for r in raw["cmd"] if r["callerid"] not in callers]
    # Phase switches can publish duplicate-valued messages within <1 ms while
    # the actual Twist publisher has queue_size=1. Do not invent 1:1 coverage.
    distances = abs(np.subtract.outer([r["bag_sec"] for r in actual], [r["bag_sec"] for r in mirrors]))
    pairs = sorted((distances[i,j], i,j) for i,j in zip(*np.where(distances < .01)))
    used_actual, used_mirror, matched = set(), set(), []
    for delta, i,j in pairs:
        if i not in used_actual and j not in used_mirror:
            used_actual.add(i); used_mirror.add(j); matched.append((actual[i],mirrors[j]))
    unmatched = [r for i,r in enumerate(mirrors) if i not in used_mirror]
    differences = np.array([[a[c]-b[c] for c in LABELS["cmd"]] for a, b in matched])
    skew = np.array([a["bag_sec"]-b["header_sec"] for a, b in matched])
    receipt_skew = np.array([a["bag_sec"]-b["bag_sec"] for a, b in matched])
    return {"matched_count": len(matched), "actual_main_publisher_count": len(actual), "stamped_count": len(mirrors),
            "unmatched_stamped": unmatched, "unmatched_actual_count": len(actual)-len(matched),
            "all_six_components_max_error": float(abs(differences).max()),
            "order_pair_header_to_cmd_receipt_p95_abs_ms": float(np.percentile(abs(skew), 95)*1000),
            "order_pair_bag_receipt_skew_p95_abs_ms": float(np.percentile(abs(receipt_skew), 95)*1000),
            "extra_command_count": len(extras), "extra_commands": extras,
            "extra_commands_all_zero": all(r[c] == 0 for r in extras for c in LABELS["cmd"]),
            "pass": bool(len(matched)==len(actual) and np.max(abs(differences)) <= 1e-12 and all(r[c] == 0 for r in extras for c in LABELS["cmd"])),
            "timebase_note": "headerless cmd receipt times are used only for auditing, not precise physical phase"}


def make_signals(raw, metadata):
    zero = metadata["phase_stamps_sec"]["ramp_up"]["first"]
    signals = {}
    for sensor, channels in LABELS.items():
        matrix = step.unique_matrix([(r["header_sec"]-zero, *[r[c] for c in channels]) for r in raw[sensor] if r["header_sec"] > 0])
        for i, channel in enumerate(channels):
            if channel in ("qx", "qy", "qz", "qw") or (sensor in ("cmd", "stamped") and channel not in ("vx", "wz")):
                continue
            values = matrix[:, i+1]
            if channel in ("roll", "pitch", "yaw"):
                values = np.unwrap(values)
            unit = "m" if sensor in ("mocap", "odom") and channel in ("x", "y", "z") else "rad" if channel in ("roll", "pitch", "yaw") else "m/s^2" if channel.startswith("a") else "rad/s" if channel.startswith("w") else "m/s"
            signals[sensor+"_"+channel] = {"t": matrix[:, 0], "y": values, "sensor": sensor, "channel": channel, "unit": unit, "fs": 90. if sensor == "mocap" else 50.}
    mx, my = signals["mocap_x"], signals["mocap_y"]
    pre = (mx["t"] >= -2.5) & (mx["t"] < -.5)
    end = metadata["phase_stamps_sec"]["post_zero"]["last"]-zero
    post = (mx["t"] >= end-1) & (mx["t"] <= end)
    first = np.array([np.median(mx["y"][pre]), np.median(my["y"][pre])])
    last = np.array([np.median(mx["y"][post]), np.median(my["y"][post])])
    direction = (last-first)/np.linalg.norm(last-first)
    for name, vector in [("s", direction), ("cross", np.array([-direction[1], direction[0]]))]:
        signals["mocap_"+name] = {**mx, "channel": name, "y": (mx["y"]-first[0])*vector[0] + (my["y"]-first[1])*vector[1]}
    return signals, {"observed_xy_motion_unit_vector": direction.tolist(), "observed_xy_displacement_m": float(np.linalg.norm(last-first)), "note": "data-defined projection in NOKOV world, not a calibrated base_link longitudinal axis"}


def degree_for(signal, phase, profile):
    ramp = phase in ("ramp_up", "ramp_down")
    ramp_degree = 2 if profile == "linear_accel" else 1
    if signal["sensor"] in ("mocap", "odom") and signal["channel"] in ("x", "y", "z", "s", "cross"):
        return ramp_degree+1 if ramp else 1
    if signal["sensor"] in ("cmd", "stamped", "odom") and signal["channel"] == "vx":
        return ramp_degree if ramp else 1
    return 1


def additional_checks(raw, signals, metadata, windows):
    selected=["stamped_vx","odom_vx","imu_ax","imu_ay","imu_wx","imu_wy",
              "mocap_s","mocap_cross","mocap_z","mocap_roll","mocap_pitch"]
    rows, halves, sampling_controls = [], [], []
    for window in windows:
        if window["kind"] != "core" or window["phase"] not in ("ramp_up","hold","ramp_down"):
            continue
        for name in selected:
            signal=signals[name]
            mask=(signal["t"]>=window["start"])&(signal["t"]<=window["stop"])
            degree=degree_for(signal,window["phase"],metadata["profile"])
            for label,extra_degree in [("primary_native",0),("trend_degree_plus_one",1)]:
                fit=spectral.scan_tone(signal["t"][mask],signal["y"][mask],degree+extra_degree)
                rows.append({"window":window["name"],"signal":name,"variant":label,"sample_count":int(mask.sum()),"degree":degree+extra_degree,**{k:v for k,v in fit.items() if k!="tone"}})
            if signal["sensor"]=="mocap":
                dt=np.diff(signal["t"])
                regular=np.r_[True,(dt>=.002)&(dt<=.025)] & np.r_[(dt>=.002)&(dt<=.025),True]
                guarded=mask&regular
                fit=spectral.scan_tone(signal["t"][guarded],signal["y"][guarded],degree)
                rows.append({"window":window["name"],"signal":name,"variant":"omit_neighbors_of_dt_outside_2_to_25ms_native_fit_only","sample_count":int(guarded.sum()),"degree":degree,**{k:v for k,v in fit.items() if k!="tone"}})
            if name in ("imu_ax","mocap_s"):
                t=signal["t"][mask]
                elapsed=t-t[0]
                control=.6*elapsed**degree+.01*np.sin(2*np.pi*5.13*elapsed+.3)
                fit=spectral.scan_tone(t,control,degree)
                rms, _=spectral.phase_spectrum(t,control,degree,signal["fs"])
                sampling_controls.append({"window":window["name"],"signal_timing":name,"true_frequency_hz":5.13,"true_peak_amplitude":.01,"fitted_frequency_hz":fit["frequency_hz"],"fitted_peak_amplitude":fit["peak_amplitude"],"hann_band_rms":rms["band_rms"],"pass":abs(fit["frequency_hz"]-5.13)<.02 and abs(fit["peak_amplitude"]-.01)<.0003})
    hold=metadata["phase_stamps_sec"]["hold"]
    zero=metadata["phase_stamps_sec"]["ramp_up"]["first"]
    first,last=hold["first"]-zero,hold["last"]-zero
    middle=(first+last)/2
    for part,start,stop in [("early",first,middle),("late",middle,last)]:
        for name in selected:
            signal=signals[name];mask=(signal["t"]>=start)&(signal["t"]<stop)
            fit=spectral.scan_tone(signal["t"][mask],signal["y"][mask],degree_for(signal,"hold",metadata["profile"]))
            halves.append({"part":part,"start_sec":start,"stop_sec":stop,"signal":name,"sample_count":int(mask.sum()),"nominal_resolution_hz":1/(stop-start),**{k:v for k,v in fit.items() if k!="tone"}})
    post=metadata["phase_stamps_sec"]["post_zero"]["first"]-zero
    signal=signals["imu_ax"]
    mask=(signal["t"]>=post+.28)&(signal["t"]<=post+1.2)
    tail=spectral.damped_tone_fit(signal["t"][mask]-post,signal["y"][mask])
    tail.update({"signal":"imu_ax","start_after_post_zero_sec":.28,"stop_after_post_zero_sec":1.2,
                 "note":"descriptive transient-tail fit only; sensor dynamics and physical body ringing not separated"})
    tail_sensitivity=[]
    for left in (.18,.28,.38):
        mask=(signal["t"]>=post+left)&(signal["t"]<=post+1.2)
        fit=spectral.damped_tone_fit(signal["t"][mask]-post,signal["y"][mask])
        tail_sensitivity.append({"start_after_post_zero_sec":left,"stop_after_post_zero_sec":1.2,
                                 **{k:v for k,v in fit.items() if k not in ("time","raw","fit")}})
    repetition={}
    for phase,bounds in metadata["phase_stamps_sec"].items():
        r=[v for v in raw["mocap"] if bounds["first"]<=v["header_sec"]<=bounds["last"]]
        matrix=np.array([[v[k] for k in ["x","y","z","qx","qy","qz","qw"]] for v in r])
        repetition[phase]={"sample_count":len(r),"identical_pose_neighbor_fraction":float(np.mean(np.all(np.diff(matrix,axis=0)==0,axis=1))),"identical_z_neighbor_fraction":float(np.mean(np.diff(matrix[:,2])==0))}
    return {"native_fit_sensitivity":rows,"hold_nonoverlapping_halves":halves,
            "actual_timestamp_synthetic_controls":sampling_controls,"mocap_phase_repetition":repetition,
            "post_stop_fit_window_sensitivity":tail_sensitivity,
            "post_stop_damped_tail":{k:v for k,v in tail.items() if k not in ("fit","time","raw")}},tail


def analyze_bag(bag, output, no_plots=False):
    metadata_path = bag.with_name(bag.stem+"_command.json")
    old_path = bag.with_name(bag.stem+"_continuity.json")
    profile = json.loads(metadata_path.read_text())["profile"]
    metadata, phases = continuity.load_metadata(metadata_path, "linear", profile)
    reused_data, _ = step.load_bag(bag, "Tracker0", "/imu/data", continuity.STAMPED_CMD_TOPIC)
    reused_audit = continuity.command_mirror_audit(reused_data["command_header"], reused_data["command_bag"], "linear", phases)
    raw, connections, topic_info = load_raw(bag)
    audit = mirror_audit(raw)
    if not audit["pass"]:
        raise ValueError("actual command differs from stamped mirror")
    for sensor, rows in raw.items():
        write_csv(output/(sensor+"_raw.csv"), rows)
    signals, projection = make_signals(raw, metadata)
    zero = phases["ramp_up_start"]
    windows = []
    for phase in PHASES:
        bounds = metadata["phase_stamps_sec"][phase]
        start, stop = bounds["first"]-zero, bounds["last"]-zero
        windows.append({"name": phase, "phase": phase, "start": start, "stop": stop, "kind": "full"})
        windows.append({"name": phase+"_core", "phase": phase, "start": start+.4, "stop": stop-.1, "kind": "core"})
    summaries, traces, spectrum_rows, residual_rows, rolling = [], {}, [], [], []
    for window in windows:
        for name, signal in signals.items():
            mask = (signal["t"] >= window["start"]) & (signal["t"] <= window["stop"])
            t, y = signal["t"][mask], signal["y"][mask]
            degree = degree_for(signal, window["phase"], profile)
            result, trace = spectral.phase_spectrum(t, y, degree, signal["fs"])
            result.update({"signal": name, "unit": signal["unit"], "window": window["name"], "window_start": window["start"], "window_stop": window["stop"]})
            summaries.append(result)
            traces[(window["name"], name)] = trace
            for f, power in zip(trace["frequency"], trace["psd"]):
                spectrum_rows.append({"window": window["name"], "signal": name, "frequency_hz": f, "psd": power})
            if window["kind"] == "full":
                for i in range(len(t)):
                    residual_rows.append({"phase": window["phase"], "signal": name, "time_from_ramp_start_sec": t[i], "raw": y[i], "trend": trace["trend"][i], "residual": trace["residual"][i], "fitted_tone": trace["tone"][i]})
                for row in spectral.rolling_tone(signal["t"], signal["y"], degree, window["start"], window["stop"]):
                    rolling.append({"phase": window["phase"], "signal": name, "unit": signal["unit"], **row})
    timing = {key: timing_stats(rows, LABELS[key]) for key, rows in raw.items()}
    frame_checks = {}
    for phase, bounds in metadata["phase_stamps_sec"].items():
        rows = [r for r in raw["stamped"] if bounds["first"] <= r["header_sec"] <= bounds["last"]]
        frame_checks[phase] = {"count": len(rows), "frame_labels": sorted(set(r["frame_id"] for r in rows)), "first_header": rows[0]["header_sec"], "last_header": rows[-1]["header_sec"]}
    result = {"bag": str(bag), "bag_sha256": sha(bag), "metadata": metadata, "metadata_sha256": sha(metadata_path),
              "old_report": str(old_path), "old_report_sha256": sha(old_path),
              "connections": connections, "topics": topic_info, "command_mirror_audit_all_axes": audit,
              "reused_command_mirror_audit": reused_audit, "timing": timing, "phase_header_checks": frame_checks,
              "projection": projection, "windows": windows, "spectral_statistics": summaries,
              "rolling_tone": rolling,
              "methods": {"timebase": "native header stamps; headerless cmd bag reception used for command audit and secondary timing diagnostics, never precise physical phase", "resampling": "linear interpolation at 50 Hz, NOKOV 90 Hz; no median or low-pass", "detrend": "phase-local polynomial; position ramp degree 2/3 for trapezoidal/linear_accel, position hold/static degree 1; velocity ramp degree 1/2; IMU and angles degree 1", "spectrum": "one Hann periodogram per uninterrupted phase/window; no zero padding", "tone": "native timestamp joint polynomial + sinusoid fit; 4.5..5.5 Hz step .01 Hz is search grid, not resolution", "rolling": "native 5.0 Hz joint fit, 1 s window, .1 s step, never crosses phase boundary; overlapping windows not independent", "coherence": "not calculated; short windows do not support strong coherence claims", "phase_alignment": "no optimized cross-sensor time shifts or physical gain/phase estimates", "damped_tail": "linear trend + exponentially damped tone, search 3..12 Hz/.05 Hz and tau .08...8 s/.01 s; post-zero +.28..1.2 s primary, +.18 and +.38 start sensitivity; descriptive only"}}
    result["additional_checks"], tail=additional_checks(raw,signals,metadata,windows)
    write_json(output/"metrics.json", result)
    write_csv(output/"spectra.csv", spectrum_rows)
    write_csv(output/"raw_trends_residuals.csv", residual_rows)
    write_csv(output/"rolling_5hz.csv", rolling)
    write_csv(output/"native_fit_sensitivity.csv",result["additional_checks"]["native_fit_sensitivity"])
    write_csv(output/"hold_halves.csv",result["additional_checks"]["hold_nonoverlapping_halves"])
    write_csv(output/"post_stop_fit_sensitivity.csv",result["additional_checks"]["post_stop_fit_window_sensitivity"])
    write_csv(output/"post_stop_damped_tail.csv",[{"seconds_after_post_zero":t,"raw_imu_ax":v,"descriptive_damped_fit":p} for t,v,p in zip(tail["time"],tail["raw"],tail["fit"])])
    flat = [{**{k:v for k,v in row.items() if k not in ("native_tone", "trend_coefficients_scaled_time")}, **{"native_"+k:v for k,v in row["native_tone"].items()}} for row in summaries]
    write_csv(output/"spectral_statistics.csv", flat)
    if not no_plots:
        make_plots(output, result, signals, traces,tail)
    return result


def make_plots(output, result, signals, traces,tail):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    selected = ["stamped_vx", "odom_vx", "imu_ax", "imu_ay", "imu_az", "imu_wx", "mocap_s", "mocap_cross", "mocap_z", "mocap_roll"]
    full = [w for w in result["windows"] if w["kind"] == "full"]
    fig, axes = plt.subplots(len(selected), 1, figsize=(13, 19), sharex=True)
    for ax, name in zip(axes, selected):
        signal = signals[name]
        ax.plot(signal["t"], signal["y"], color="#3177a5", lw=.75, label="raw samples")
        for w in full:
            trace = traces[(w["name"], name)]
            ax.plot(trace["time"], trace["trend"], color="#cc6622", lw=1.5)
            ax.axvline(w["start"], color=".5", lw=.6, ls="--")
        ax.set_ylabel(name+"\n"+signal["unit"]); ax.grid(alpha=.2)
    axes[0].set_title(result["metadata"]["profile"]+": raw curves and phase-local polynomial trends")
    axes[-1].set_xlabel("Seconds from ramp-up start (header time)")
    fig.tight_layout(); fig.savefig(str(output/"01_raw_and_trends.png"), dpi=150); plt.close(fig)
    fig, axes = plt.subplots(len(selected), 5, figsize=(20, 22))
    for i, name in enumerate(selected):
        for j, w in enumerate(full):
            ax = axes[i, j]; trace = traces[(w["name"], name)]
            if np.any(trace["psd"] > 0):
                ax.semilogy(trace["frequency"], np.maximum(trace["psd"], 1e-30), lw=1)
            else:
                ax.text(.5,.5,"constant signal: zero PSD",ha="center",transform=ax.transAxes,fontsize=8)
            ax.axvspan(4.5, 5.5, color="#eecc77", alpha=.5); ax.set_xlim(0,20); ax.grid(alpha=.2)
            if i == 0: ax.set_title(w["name"])
            if j == 0: ax.set_ylabel(name+"\nPSD ["+signals[name]["unit"]+"]²/Hz")
            if i == len(selected)-1: ax.set_xlabel("Frequency [Hz]")
    fig.suptitle("Whole spectrum first; shaded band 4.5–5.5 Hz. Different physical units; no amplitude ratios.")
    fig.tight_layout(rect=[0,0,1,.97]); fig.savefig(str(output/"02_stage_spectra.png"), dpi=140); plt.close(fig)
    residual_selected = ["stamped_vx", "odom_vx", "imu_ay", "imu_wx", "mocap_cross", "mocap_z"]
    fig, axes = plt.subplots(len(residual_selected), 5, figsize=(20,15))
    for i,name in enumerate(residual_selected):
        for j,w in enumerate(full):
            ax=axes[i,j]; trace=traces[(w["name"],name)]
            ax.plot(trace["time"]-w["start"], trace["residual"], '.-', color="#3177a5", ms=2,lw=.7)
            ax.plot(trace["time"]-w["start"], trace["tone"], color="#cc6622", lw=1)
            ax.grid(alpha=.2)
            if i==0: ax.set_title(w["name"])
            if j==0: ax.set_ylabel(name+"\n"+signals[name]["unit"])
            if i==len(residual_selected)-1: ax.set_xlabel("Seconds within phase")
    fig.suptitle("Unfiltered detrended native samples (blue); fitted target-band tone (orange), not filtered motion truth")
    fig.tight_layout(rect=[0,0,1,.975]); fig.savefig(str(output/"03_native_residuals.png"),dpi=140);plt.close(fig)
    fig,axes=plt.subplots(6,1,figsize=(13,13),sharex=True)
    for ax,name in zip(axes,residual_selected):
        for w in full:
            rows=[r for r in result["rolling_tone"] if r["signal"]==name and r["phase"]==w["name"]]
            ax.plot([(r["start_sec"]+r["stop_sec"])/2 for r in rows],[r["peak_amplitude"] for r in rows],'.-',label=w["name"])
            ax.axvline(w["start"],color=".6",ls="--",lw=.6)
        ax.set_ylabel(name+"\npeak "+signals[name]["unit"]);ax.grid(alpha=.2)
    axes[0].set_title("Local 5.0 Hz amplitude: native joint fits, 1 s window / 0.1 s hop, separated phases")
    axes[0].legend(ncol=5);axes[-1].set_xlabel("Seconds from ramp-up start")
    fig.tight_layout();fig.savefig(str(output/"04_local_5hz_amplitude.png"),dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,11))
    stop=result["metadata"]["phase_stamps_sec"]["post_zero"]["first"]-result["metadata"]["phase_stamps_sec"]["ramp_up"]["first"]
    s=signals["imu_ax"];m=(s["t"]>=stop-.15)&(s["t"]<=stop+2)
    axes[0].plot(s["t"][m]-stop,s["y"][m],'.-',ms=3,label="raw IMU ax")
    axes[0].plot(tail["time"],tail["fit"],label="descriptive damped-tail fit")
    axes[0].set_title("Stopping transient: fitted %.2f Hz, decay %.2f s; not an identified mechanical mode"%(tail["frequency_hz"],tail["tau_sec"]))
    axes[0].set_ylabel("IMU ax [m/s²]");axes[0].legend();axes[0].grid(alpha=.2)
    axes[1].plot(tail["time"],tail["raw"],'.-',ms=3,label="raw IMU ax tail")
    axes[1].plot(tail["time"],tail["fit"],label="descriptive fit")
    axes[1].set_xlabel("Seconds after post-zero command begins")
    axes[1].set_ylabel("IMU ax [m/s²]");axes[1].legend();axes[1].grid(alpha=.2)
    for sensor in ("stamped","imu","odom","mocap"):
        import csv
        r=list(csv.DictReader((output/(sensor+"_raw.csv")).open()))
        t=np.array([float(v["header_sec"]) for v in r])
        axes[2].plot(t[1:]-result["metadata"]["phase_stamps_sec"]["ramp_up"]["first"],np.diff(t)*1000,'.',ms=2,label=sensor)
    axes[0].set_xlabel("Seconds after post-zero command begins")
    axes[2].set_xlabel("Seconds from ramp-up start");axes[2].set_ylabel("Header interval [ms]");axes[2].legend(ncol=4);axes[2].grid(alpha=.2)
    fig.tight_layout();fig.savefig(str(output/"05_stopping_tail_and_timing.png"),dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--no-plots",action="store_true")
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if any(args.output_dir.iterdir()):
        raise SystemExit("Output must be new/empty; old reports are never overwritten")
    results=[]
    for tag in ("TRAP", "LA"):
        bag=args.data_dir/("DEV_CHASSIS_CONT_"+tag+"_LIN_F_V080_a01.bag")
        folder=args.output_dir/tag;folder.mkdir()
        result=analyze_bag(bag,folder,args.no_plots);results.append(result)
        print(tag, "analysis complete", len(result["spectral_statistics"]), "window/channel rows", flush=True)
    source=Path(__file__).resolve().parent
    source_files=[Path(__file__),source/"continuity_spectrum_core.py",source/"analyze_mocap_velocity_continuity.py",source/"analyze_mocap_velocity_step.py",source/"velocity_continuity_core.py",source/"velocity_step_response_core.py",source/"same_bag_delay_core.py"]
    snapshot=args.output_dir/"source_snapshot"/"analysis";snapshot.mkdir(parents=True)
    for path in source_files: (snapshot/path.name).write_bytes(path.read_bytes())
    def git(*parts):
        result=subprocess.run(["git","-C",str(source),*parts],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        return result.stdout.strip() if result.returncode==0 else None
    write_json(args.output_dir/"provenance.json",{"git_head":git("rev-parse","HEAD"),"git_status":git("status","--short"),"python":sys.version,"numpy":np.__version__,"argv":sys.argv,"source_sha256":{str(p):sha(p) for p in source_files},"bags":[{"path":r["bag"],"sha256":r["bag_sha256"]} for r in results],"phase_joining":False,"motion_executed":False})
    print(args.output_dir,flush=True)


if __name__ == "__main__":
    main()
