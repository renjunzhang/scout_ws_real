#!/usr/bin/env python3
"""Offline-only tests for U3 solver output QC v1."""

from __future__ import annotations

import math
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_u3_solver_output_qc_v1 as qc  # noqa: E402


def _string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _value(name: str, type_code: int, payload: bytes) -> bytes:
    return _string(name) + struct.pack("<i", type_code) + payload


def _values(records: list[bytes]) -> bytes:
    return _string("\nVALUES") + struct.pack("<I", len(records)) + b"".join(records)


def _array(name: str, type_code: int, count: int, payload: bytes) -> bytes:
    definition = (
        _string("\nARRAY")
        + _string(name)
        + struct.pack("<iiII", 0, type_code, count, len(payload))
    )
    return struct.pack("<I", len(definition)) + definition + payload


def _item(
    name: str,
    values: list[bytes],
    arrays: list[bytes] | None = None,
    children: list[bytes] | None = None,
) -> bytes:
    arrays = arrays or []
    children = children or []
    value_block = _values(values)
    definition = (
        _string("\nITEM\n")
        + _string(name)
        + struct.pack("<ii", 0, 0)
        + _string("%.7E")
        + _string("%.15E")
        + struct.pack("<III", len(arrays), len(children), len(value_block))
    )
    return (
        struct.pack("<I", len(definition))
        + definition
        + value_block
        + b"".join(arrays)
        + b"".join(children)
    )


def _header(code: str) -> bytes:
    return (f"#FileJBD {code}".encode("ascii")).ljust(58, b" ") + b"\n\0\0\0\0\0"


def particle_fixture() -> tuple[list[int], list[tuple[float, float, float]], list[tuple[float, float, float]], list[float]]:
    positions = [(-0.018, 0.0, 0.0), (0.018, 0.0, 0.0)]
    for sector in range(16):
        angle = (sector + 0.5) * 2 * math.pi / 16
        for sample in range(8):
            radius = 0.004 + (sample % 2) * 0.001
            z = (0.052, 0.054, 0.056, 0.058)[sample % 4]
            positions.append((radius * math.cos(angle), radius * math.sin(angle), z))
    ids = list(range(len(positions)))
    velocities = [(0.0, 0.0, 0.0)] * len(ids)
    densities = [1000.0, 1000.0] + [1001.0] * (len(ids) - 2)
    return ids, positions, velocities, densities


def make_part(
    cpart: int,
    time_s: float,
    *,
    ids: list[int] | None = None,
    positions: list[tuple[float, float, float]] | None = None,
    velocities: list[tuple[float, float, float]] | None = None,
    densities: list[float] | None = None,
    nout: int = 0,
) -> bytes:
    base_ids, base_positions, base_velocities, base_densities = particle_fixture()
    ids = list(base_ids if ids is None else ids)
    positions = list(base_positions if positions is None else positions)
    velocities = list(base_velocities if velocities is None else velocities)
    densities = list(base_densities if densities is None else densities)
    arrays = [
        _array("Idp", 8, len(ids), b"".join(struct.pack("<I", value) for value in ids)),
        _array("Posd", 23, len(ids), b"".join(struct.pack("<ddd", *value) for value in positions)),
        _array("Vel", 22, len(ids), b"".join(struct.pack("<fff", *value) for value in velocities)),
        _array("Rhop", 11, len(ids), b"".join(struct.pack("<f", value) for value in densities)),
    ]
    part = _item(
        f"PART_{cpart:04d}",
        [
            _value("Cpart", 8, struct.pack("<I", cpart)),
            _value("TimeStep", 12, struct.pack("<d", time_s)),
            _value("Npok", 8, struct.pack("<I", len(ids))),
            _value("Nout", 8, struct.pack("<I", nout)),
            _value("Step", 8, struct.pack("<I", cpart * 100)),
            _value("RunTime", 12, struct.pack("<d", 0.0)),
        ],
        arrays=arrays,
    )
    root = _item(
        "JPartDataBi4",
        [
            _value("CaseNp", 10, struct.pack("<Q", len(base_ids))),
            _value("CaseNfixed", 10, struct.pack("<Q", 2)),
            _value("CaseNmoving", 10, struct.pack("<Q", 0)),
            _value("CaseNfloat", 10, struct.pack("<Q", 0)),
            _value("CaseNfluid", 10, struct.pack("<Q", len(base_ids) - 2)),
            _value("Dp", 12, struct.pack("<d", 0.002)),
            _value("H", 12, struct.pack("<d", 0.0034641016151)),
            _value("Rhop0", 12, struct.pack("<d", 1000.0)),
            _value("MassFluid", 12, struct.pack("<d", 8e-6)),
        ],
        children=[part],
    )
    return _header("JPartDataBi4") + root


def make_partout(*, nout: int = 0) -> bytes:
    root = _item(
        "JPartOutBi4",
        [
            _value("CaseNp", 10, struct.pack("<Q", 130)),
            _value("CaseNfixed", 10, struct.pack("<Q", 2)),
            _value("CaseNmoving", 10, struct.pack("<Q", 0)),
            _value("CaseNfloat", 10, struct.pack("<Q", 0)),
            _value("CaseNfluid", 10, struct.pack("<Q", 128)),
        ],
    )
    appended = b""
    if nout:
        appended = _item(
            "PART_0001",
            [
                _value("Cpart", 8, struct.pack("<I", 1)),
                _value("TimeStep", 12, struct.pack("<d", 1.0)),
                _value("Nout", 8, struct.pack("<I", nout)),
            ],
            arrays=[
                _array("Idp", 8, nout, b"".join(struct.pack("<I", 129) for _ in range(nout))),
                _array("Posd", 23, nout, b"".join(struct.pack("<ddd", 0.0, 0.0, 0.08) for _ in range(nout))),
                _array("Vel", 22, nout, b"".join(struct.pack("<fff", 0.0, 0.0, 0.1) for _ in range(nout))),
                _array("Rhop", 11, nout, b"".join(struct.pack("<f", 1000.0) for _ in range(nout))),
                _array("Motive", 4, nout, bytes([1]) * nout),
            ],
        )
    return _header("JPartOutBi4") + root + appended


CASE_XML = b"""<?xml version="1.0"?>
<case>
  <casedef>
    <constantsdef><hswl value="0.058"/></constantsdef>
    <geometry><commands><mainlist>
      <drawcylinder radius="0.0185"><point x="0" y="0" z="0"/><point x="0" y="0" z="0.058"/></drawcylinder>
    </mainlist></commands></geometry>
  </casedef>
  <execution>
    <parameters>
      <parameter key="TimeMax" value="1.0"/><parameter key="TimeOut" value="0.05"/>
      <parameter key="MinFluidStop" value="1"/><parameter key="RhopOutMin" value="700"/>
      <parameter key="RhopOutMax" value="1300"/>
      <simulationdomain><posmin x="-0.021" y="-0.021" z="-0.002"/><posmax x="0.021" y="0.021" z="0.070"/></simulationdomain>
    </parameters>
    <particles np="130"><fixed begin="0" count="2"/><fluid begin="2" count="128"/></particles>
    <constants><dp value="0.002"/><h value="0.0034641016151"/><rhop0 value="1000"/><massfluid value="0.000008"/></constants>
    <motion/>
  </execution>
</case>
"""


RUN_CSV_HEADER = (
    "#RunName;Rcode-VersionInfo;DateTime;Np;TSimul;TSeg;TTotal;MemCpu;MemGpu;MemGpuCells;"
    "Steps;GPIPS;PhysicalTime;PartFiles;PartsOut;MaxParticles;MaxCells;Hardware;RunMode;"
    "Configuration;Nbound;Nfixed;Dp;H;PartsOutRho;PartsOutVel\n"
)


def make_run_csv(final_time: float, part_count: int, parts_out: int = 0) -> bytes:
    values = [
        "run", "code", "date", "130", "1", "1", "1", "1", "0", "0", "300", "", f"{final_time:g}",
        str(part_count), str(parts_out), "130", "20", "CPU", "CellsFull", "cfg", "2", "2",
        "0.002", "0.0034641016151", "0", "0",
    ]
    return (RUN_CSV_HEADER + ";".join(values) + "\n").encode()


def make_run_out(parts_out: int = 0) -> bytes:
    return f"""DualSPHysics test
[Simulation finished  test]
Excluded particles...............: {parts_out}
Finished execution (code=0).
""".encode()


def write_run(
    root: Path,
    times: list[float],
    *,
    mutate_final_velocity: float | None = None,
    nonfinite_final: bool = False,
    leak_final: bool = False,
) -> None:
    root.mkdir()
    data = root / "data"
    data.mkdir()
    ids, positions, velocities, densities = particle_fixture()
    for index, time_s in enumerate(times):
        current_ids = ids
        current_positions = positions
        current_velocities = list(velocities)
        current_densities = densities
        nout = 0
        if index == len(times) - 1 and mutate_final_velocity is not None:
            current_velocities[2] = (mutate_final_velocity, 0.0, 0.0)
        if index == len(times) - 1 and nonfinite_final:
            current_velocities[2] = (float("nan"), 0.0, 0.0)
        if index == len(times) - 1 and leak_final:
            current_ids = ids[:-1]
            current_positions = positions[:-1]
            current_velocities = current_velocities[:-1]
            current_densities = densities[:-1]
            nout = 1
        (data / f"Part_{index:04d}.bi4").write_bytes(
            make_part(
                index,
                time_s,
                ids=current_ids,
                positions=current_positions,
                velocities=current_velocities,
                densities=current_densities,
                nout=nout,
            )
        )
    (data / "PartOut_000.obi4").write_bytes(make_partout(nout=1 if leak_final else 0))
    (root / "Run.csv").write_bytes(make_run_csv(times[-1], len(times), 1 if leak_final else 0))
    (root / "Run.out").write_bytes(make_run_out(1 if leak_final else 0))


class U3SolverOutputQcV1Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-u3-solver-qc-")
        self.base = Path(self.temporary.name)
        self.case_bi4 = self.base / "C1_static.bi4"
        self.case_xml = self.base / "C1_static.xml"
        self.case_bi4.write_bytes(make_part(0, 0.0))
        self.case_xml.write_bytes(CASE_XML)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def report(self, run: Path, compare: Path | None = None) -> dict:
        return qc.build_report(
            run,
            self.case_bi4,
            self.case_xml,
            compare_run_dir=compare,
        )

    def test_one_second_clean_output_is_smoke_and_never_settled(self) -> None:
        run = self.base / "run-smoke"
        write_run(run, [0.0, 1.0])
        report = self.report(run)
        self.assertEqual(report["verdict"]["status"], "PASS_U3_SOLVER_OUTPUT_SMOKE_ONLY")
        self.assertTrue(report["run"]["short_duration_smoke"])
        self.assertFalse(report["verdict"]["settled_state_claim_allowed"])
        self.assertFalse(report["verdict"]["settled_state_freeze_eligible"])
        self.assertEqual(report["run"]["parts"][-1]["fluid_mass_kg"], 128 * 8e-6)

    def test_two_long_identical_outputs_pass_exact_determinism(self) -> None:
        first, second = self.base / "run-a", self.base / "run-b"
        write_run(first, [0.0, 2.0, 3.0])
        write_run(second, [0.0, 2.0, 3.0])
        report = self.report(first, second)
        self.assertEqual(
            report["verdict"]["status"],
            "PASS_U3_DEVELOPMENT_SETTLE_OUTPUT_QC_TWO_RUNS",
        )
        self.assertTrue(report["comparison"]["all_part_sha256_exact"])
        self.assertTrue(report["comparison"]["all_partout_sha256_exact"])
        self.assertTrue(report["verdict"]["two_run_numeric_settle_qc_pass"])
        self.assertFalse(report["verdict"]["settled_state_freeze_eligible"])

    def test_small_numeric_difference_fails_exact_reproducibility(self) -> None:
        first, second = self.base / "run-a", self.base / "run-b"
        write_run(first, [0.0, 2.0, 3.0])
        write_run(second, [0.0, 2.0, 3.0], mutate_final_velocity=1e-5)
        report = self.report(first, second)
        self.assertEqual(
            report["verdict"]["status"],
            "FAIL_U3_DEVELOPMENT_SETTLE_REPRODUCIBILITY_QC",
        )
        self.assertFalse(report["comparison"]["all_part_sha256_exact"])

    def test_nonfinite_particle_value_is_a_hard_qc_failure(self) -> None:
        run = self.base / "run-nan"
        write_run(run, [0.0, 1.0], nonfinite_final=True)
        report = self.report(run)
        self.assertEqual(report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC")
        self.assertFalse(report["run"]["checks"]["structural"]["all_particle_arrays_finite"])

    def test_particle_loss_agrees_across_part_partout_and_run_summary(self) -> None:
        run = self.base / "run-leak"
        write_run(run, [0.0, 1.0], leak_final=True)
        report = self.report(run)
        checks = report["run"]["checks"]["structural"]
        self.assertEqual(report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC")
        self.assertFalse(checks["zero_part_nout"])
        self.assertFalse(checks["zero_partout_records"])
        self.assertFalse(checks["mass_exact"])
        self.assertEqual(report["run"]["partout"][0]["total_nout"], 1)

    def test_partout_header_only_file_means_zero_exclusions(self) -> None:
        source = qc.bi4.SecureFile(
            path=Path("PartOut_000.obi4"),
            data=make_partout(),
            sha256="0" * 64,
            size_bytes=len(make_partout()),
            mode=0o440,
            uid=os.getuid(),
            gid=os.getgid(),
        )
        parsed = qc.parse_partout(source)
        self.assertEqual(parsed["appended_part_count"], 0)
        self.assertEqual(parsed["total_nout"], 0)


if __name__ == "__main__":
    unittest.main()
