#!/usr/bin/env python3
"""Pure fixture/negative tests for S5B0 post-execution output QC v9.

No bag, candidate, GPU, network, sudo, profile, or external output is opened.
"""

from __future__ import annotations

import copy
import math
import struct
import sys
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5b0_finalized_frame_reader_v2 as frames  # noqa: E402
import r8_liquid_s5b0_native_gauge_normalizer_v1 as gauges  # noqa: E402
import r8_liquid_s5b0_replay_output_qc_v9 as qc  # noqa: E402


def runparts(start: int, times: list[float], steps: list[int]) -> bytes:
    header = ("Part;TimeStep [s];Steps;NpSave;NpSim;NpOut;NpbSim;NpfSim;"
              "NpNormal;NpOutPos;NpOutRho;NpOutMov")
    rows = [header]
    for offset, (time_s, count) in enumerate(zip(times, steps, strict=True)):
        rows.append(f"{start + offset};{time_s:.17g};{count:,};9,078;9,078;0;"
                    "2,669;6,409;9,078;0;0;0")
    return ("\n".join(rows) + "\n").encode()


def solver_path() -> bytes:
    return (b"# t,x,y,z,yaw,pitch,roll\n"
            b"0,0,0,0,0,0,0\n"
            b"1,1,2,0,90,10,-5\n"
            b"2,2,4,0,180,20,-10\n")


def probe_grid() -> list[dict[str, object]]:
    return copy.deepcopy(list(qc.PROBE_GRID))


def native_row(time_s: float, ray: dict[str, tuple[float, float, float]],
               *, z_relative: float = .058) -> str:
    p0, p2 = ray["point0_m"], ray["point2_m"]
    swlz = p0[2] + (z_relative - gauges.MIN_Z_M)
    return ";".join(format(value, ".17g") for value in
                    (time_s, p0[0], p0[1], swlz, *p0, *p2))


def native_payloads(times: list[float], rays: dict[str, list[dict[str, tuple[float, float, float]]]]) -> dict[str, bytes]:
    header = ";".join(gauges.NATIVE_HEADER)
    return {name: (header + "\n" + "\n".join(
        native_row(time_s, ray) for time_s, ray in zip(times, rays[name], strict=True)
    ) + "\n").encode() for name in qc.PROBES}


def fixture_item(name: str, *, values: dict[str, tuple[int, object]],
                 arrays: dict[str, tuple[int, list[object]]], hidden: bool = False) -> bytes:
    def string(value: str) -> bytes:
        raw = value.encode(); return struct.pack("<I", len(raw)) + raw
    def value(key: str, kind: int, item: object) -> bytes:
        if kind == 1: payload = string(str(item))
        elif kind == 2: payload = struct.pack("<i", int(bool(item)))
        elif kind == 6: payload = struct.pack("<H", int(item))
        elif kind == 8: payload = struct.pack("<I", int(item))
        elif kind == 12: payload = struct.pack("<d", float(item))
        else: raise AssertionError(kind)
        return string(key) + struct.pack("<i", kind) + payload
    def array(key: str, kind: int, items: list[object]) -> bytes:
        formats = {6: "H", 8: "I", 12: "d", 23: "ddd"}
        fmt = formats[kind]
        payload = b"".join(struct.pack("<" + fmt, *item) if kind == 23
                           else struct.pack("<" + fmt, item) for item in items)
        definition = string("\nARRAY") + string(key) + struct.pack(
            "<iiII", 0, kind, len(items), len(payload))
        return struct.pack("<I", len(definition)) + definition + payload
    records = [value(key, kind, item) for key, (kind, item) in values.items()]
    value_block = string("\nVALUES") + struct.pack("<I", len(records)) + b"".join(records)
    array_blocks = [array(key, kind, items) for key, (kind, items) in arrays.items()]
    definition = (string("\nITEM\n") + string(name) + struct.pack("<ii", int(hidden), 0)
                  + string("%.7E") + string("%.15E")
                  + struct.pack("<III", len(array_blocks), 0, len(value_block)))
    return struct.pack("<I", len(definition)) + definition + value_block + b"".join(array_blocks)


def motion_ref(frame_rows: list[dict[str, object]], positions: list[dict[int, tuple[float, float, float]]],
               *, fake_reference: bool = False) -> bytes:
    ids = [0, 1, 2668]
    root_positions = [positions[0][value] for value in ids]
    distances = [1.1 * math.dist(value, root_positions[0]) for value in root_positions]
    root = fixture_item("JPartMotRefBi4", values={
        "AppName": (1, "DualSPHysics fixture"), "FormatVer": (8, 230729),
        "MainFile": (2, True), "TimeOut": (12, .05), "MkBoundFirst": (6, 2),
        "MkMovingCount": (8, 1), "MkFloatCount": (8, 0), "MkCount": (8, 1)},
        arrays={"MkBound": (6, [0]), "Nid": (8, [3]), "Id": (8, ids),
                "Ps": (23, root_positions), "Dis": (12, distances)})
    appended = []
    for index, (row, current) in enumerate(zip(frame_rows, positions, strict=True)):
        observed = [current[value] for value in ids]
        if fake_reference and index == 1:
            observed[0] = (observed[0][0] + 1e-3, *observed[0][1:])
        appended.append(fixture_item(f"PART_{int(row['index']):04d}", hidden=True,
            values={"Cpart": (8, row["index"]), "TimeStep": (12, row["time_s"]),
                    "Step": (8, row["step"])}, arrays={"PosRef": (23, observed)}))
    header = b"#FileJBD JPartMotRefBi4".ljust(58, b" ") + b"\n\0\0\0\0\0"
    return header + root + b"".join(appended)


def test_schema_closed_and_static_self_checks() -> None:
    schema = frames.json.loads(frames.SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); frames.v1.assert_deep_closed(schema)
    assert frames.self_check()["particle_count"] == 9078
    assert gauges.self_check()["files_written"] is False
    assert qc.self_check()["optional_bag_read"] is False


@pytest.mark.parametrize("start,steps,wanted", [
    (701, [0, 2178, 2178], [0, 2177, 4355]),
    (801, [0, 2179, 2178], [0, 2178, 4356]),
])
def test_restart_runparts_variable_steps_positive(start: int, steps: list[int], wanted: list[int]) -> None:
    parsed = frames.parse_runparts(runparts(start, [35.05, 35.10, 35.15], steps))
    assert [row["part"] for row in parsed] == [start, start + 1, start + 2]
    assert [row["step"] for row in parsed] == wanted


def test_restart_runparts_rejects_first_nonzero_and_off_by_one_frame() -> None:
    with pytest.raises(frames.FinalizedFrameV2Error, match="first|restart"):
        frames.parse_runparts(runparts(901, [45.05, 45.10], [1, 2178]))
    parsed = frames.parse_runparts(runparts(901, [45.05, 45.10], [0, 2178]))
    raw = frames.v1._fixture_part_bytes(902, 45.10, ids=range(9078),
                                       class_counts=(0, 2669, 0, 6409))
    with pytest.raises(frames.FinalizedFrameV2Error, match="step"):
        frames.parse_frame(raw, index=902, runpart={**parsed[1], "step": 903})


def test_all_2669_moving_particles_intrinsic_zyx_translation_and_negatives() -> None:
    base = {value: ((value % 31) * 1e-4, ((value // 31) % 29) * 1e-4,
                    (value % 7) * 1e-4) for value in range(2669)}
    rows = [{"index": 901, "time_s": 10.0, "step": 0},
            {"index": 902, "time_s": 11.0, "step": 10},
            {"index": 903, "time_s": 12.0, "step": 20}]
    source_rows = qc._solver_rows(solver_path())
    observed = []
    for frame in rows:
        translation, angles = qc._interpolate(source_rows, frame["time_s"] - 10.0)
        observed.append({value: qc._rigid_point(point, translation, angles, (0., 0., 0.))
                         for value, point in base.items()})
    evidence_raw, evidence = qc.boundary_qc(rows, observed, solver_path(), settled_time_s=10.0)
    assert evidence["maximum_rigid_residual_m"] < 1e-12
    assert evidence["moving_particle_checks"] == 3 * 2669
    assert evidence_raw.count(b"\n") == 4
    translated = copy.deepcopy(observed); translated[1][2668] = (
        translated[1][2668][0] + 1e-3, *translated[1][2668][1:])
    with pytest.raises(qc.ReplayOutputQcV9Error, match="moving boundary"):
        qc.boundary_qc(rows, translated, solver_path(), settled_time_s=10.0)
    translation_drift = solver_path().replace(b"1,1,2,0,90", b"1,1.1,2,0,90")
    with pytest.raises(qc.ReplayOutputQcV9Error, match="moving boundary"):
        qc.boundary_qc(rows, observed, translation_drift, settled_time_s=10.0)
    yaw_drift = solver_path().replace(b"1,1,2,0,90,10,-5", b"1,1,2,0,80,10,-5")
    with pytest.raises(qc.ReplayOutputQcV9Error, match="moving boundary"):
        qc.boundary_qc(rows, observed, yaw_drift, settled_time_s=10.0)


def test_motion_ref_full_identity_alignment_and_fake_reference_rejected() -> None:
    rows = [{"index": 901, "time_s": 45.05, "step": 0},
            {"index": 902, "time_s": 45.10, "step": 2177}]
    positions = [{value: (value * 1e-6, value * 2e-6, value * 3e-6)
                  for value in range(2669)} for _ in rows]
    assert qc.parse_motion_ref(motion_ref(rows, positions), rows, positions)["pass"] is True
    with pytest.raises(qc.ReplayOutputQcV9Error, match="position"):
        qc.parse_motion_ref(motion_ref(rows, positions, fake_reference=True), rows, positions)


def test_motion_attached_global_z_rays_normalize_and_fake_ray_fails() -> None:
    times = [10.0, 11.0, 12.0]; grid = probe_grid()
    frame_rows = [{"time_s": value} for value in times]
    rays = qc.expected_probe_rays(frame_rows, solver_path(), grid, settled_time_s=10.0)
    for values in rays.values():
        assert all(abs(row["point0_m"][0] - row["point2_m"][0]) < 1e-15
                   and abs(row["point0_m"][1] - row["point2_m"][1]) < 1e-15
                   for row in values)
    raw = native_payloads(times, rays)
    normalized, manifest = qc.normalize_gauges(raw, expected_times_s=times,
        probe_grid=grid, solver_path_raw=solver_path(), settled_time_s=10.0)
    assert len(normalized) == 16 and manifest["pass"] is True
    bad = dict(raw); lines = bad[qc.PROBES[0]].decode().splitlines()
    tokens = lines[2].split(";"); tokens[4] = format(float(tokens[4]) + 1e-3, ".17g")
    lines[2] = ";".join(tokens); bad[qc.PROBES[0]] = ("\n".join(lines) + "\n").encode()
    with pytest.raises(qc.gauge_reader.NativeGaugeError, match="geometry"):
        qc.normalize_gauges(bad, expected_times_s=times, probe_grid=grid,
            solver_path_raw=solver_path(), settled_time_s=10.0)


def test_nonfinite_nout_id_and_grid_drift_fail_closed() -> None:
    with pytest.raises(qc.ReplayOutputQcV9Error, match="non-finite"):
        qc._solver_rows(solver_path().replace(b"1,1,2,0", b"1,nan,2,0"))
    with pytest.raises(qc.ReplayOutputQcV9Error, match="strictly increasing"):
        qc._solver_rows(solver_path().replace(b"2,2,4,0", b"1,2,4,0"))
    with pytest.raises(qc.ReplayOutputQcV9Error, match="coverage"):
        qc._interpolate(qc._solver_rows(solver_path()), 2.01)
    runpart = {"part": 7, "time_s": 1.0, "step": 7, "steps_since_previous_part": 0}
    bad_nout = frames.v1._fixture_part_bytes(7, 1.0, ids=range(9078), nout=1,
                                             class_counts=(0, 2669, 0, 6409))
    with pytest.raises(frames.FinalizedFrameV2Error, match="Nout"):
        frames.parse_frame(bad_nout, index=7, runpart=runpart)
    duplicate = list(range(9078)); duplicate[-1] = 1
    duplicate_ids = frames.v1._fixture_part_bytes(7, 1.0, ids=duplicate,
                                                  class_counts=(0, 2669, 0, 6409))
    with pytest.raises(frames.FinalizedFrameV2Error, match="ID"):
        frames.parse_frame(duplicate_ids, index=7, runpart=runpart)
    nonfinite = frames.v1._fixture_part_bytes(7, 1.0, ids=range(9078), nonfinite=True,
                                             class_counts=(0, 2669, 0, 6409))
    with pytest.raises(frames.FinalizedFrameV2Error, match="non-finite"):
        frames.parse_frame(nonfinite, index=7, runpart=runpart)
    drifted = probe_grid(); drifted[3]["x_m"] = float(drifted[3]["x_m"]) + 1e-6
    with pytest.raises(qc.ReplayOutputQcV9Error, match="grid"):
        qc.expected_probe_rays([{"time_s": 0.0}], solver_path(), drifted,
                               settled_time_s=0.0)
