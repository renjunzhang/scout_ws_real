#!/usr/bin/env python3
"""Offline-only tests for U3 solver output QC v2."""

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

import r8_liquid_u3_solver_output_qc_v2 as qc  # noqa: E402


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


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def _next_float32(value: float) -> float:
    bits = struct.unpack("<I", struct.pack("<f", value))[0]
    return struct.unpack("<f", struct.pack("<I", bits + 1))[0]


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
    root_overrides: dict[str, int | float] | None = None,
    omit_root_fields: set[str] | None = None,
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
    root_values: dict[str, int | float] = {
        "CaseNp": len(base_ids),
        "CaseNfixed": 2,
        "CaseNmoving": 0,
        "CaseNfloat": 0,
        "CaseNfluid": len(base_ids) - 2,
        "Dp": 0.002,
        "H": 0.0034641016151,
        "Rhop0": 1000.0,
        "MassFluid": _float32(8e-6),
    }
    root_values.update(root_overrides or {})
    omitted = omit_root_fields or set()
    root_records = []
    for name, value in root_values.items():
        if name in omitted:
            continue
        if name.startswith("CaseN"):
            root_records.append(_value(name, 10, struct.pack("<Q", int(value))))
        else:
            root_records.append(_value(name, 12, struct.pack("<d", float(value))))
    root = _item("JPartDataBi4", root_records, children=[part])
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
    mutate_initial_position: float | None = None,
    nonfinite_final: bool = False,
    leak_final: bool = False,
    part_root_overrides: dict[int, dict[str, int | float]] | None = None,
    part_omit_root_fields: dict[int, set[str]] | None = None,
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
        if index == 0 and mutate_initial_position is not None:
            current_positions = list(current_positions)
            x, y, z = current_positions[2]
            current_positions[2] = (x + mutate_initial_position, y, z)
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
                root_overrides=(part_root_overrides or {}).get(index),
                omit_root_fields=(part_omit_root_fields or {}).get(index),
            )
        )
    (data / "PartOut_000.obi4").write_bytes(make_partout(nout=1 if leak_final else 0))
    (root / "Run.csv").write_bytes(make_run_csv(times[-1], len(times), 1 if leak_final else 0))
    (root / "Run.out").write_bytes(make_run_out(1 if leak_final else 0))


class U3SolverOutputQcV2Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-u3-solver-qc-")
        self.base = Path(self.temporary.name)
        self.case_bi4 = self.base / "C1_static.bi4"
        self.case_xml = self.base / "C1_static.xml"
        self.case_bi4.write_bytes(
            make_part(0, 0.0, root_overrides={"MassFluid": 8e-6})
        )
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

    def test_real_float32_quantized_mass_passes_smoke_and_never_settles(self) -> None:
        run = self.base / "run-smoke"
        write_run(run, [0.0, 1.0])
        report = self.report(run)
        self.assertEqual(report["verdict"]["status"], "PASS_U3_SOLVER_OUTPUT_SMOKE_ONLY")
        self.assertTrue(report["run"]["structural_pass"])
        self.assertTrue(report["run"]["short_duration_smoke"])
        self.assertFalse(report["verdict"]["settled_state_claim_allowed"])
        self.assertFalse(report["verdict"]["settled_state_freeze_eligible"])
        effective = _float32(8e-6)
        self.assertEqual(
            report["fixed_case"]["requested_case_massfluid_kg"], 8e-6
        )
        self.assertEqual(
            report["fixed_case"]["solver_effective_massfluid_kg"], effective
        )
        self.assertEqual(
            report["run"]["parts"][-1]["fluid_mass_kg"], 128 * effective
        )
        mass_check = report["run"]["parts"][0]["root_case_invariants"]["MassFluid"]
        self.assertEqual(mass_check["expected_case"], 8e-6)
        self.assertEqual(mass_check["expected_solver_serialized"], effective)
        self.assertEqual(mass_check["observed"], effective)
        self.assertTrue(mass_check["match"])
        required_report_fields = {
            "expected_case",
            "expected_solver_serialized",
            "observed",
            "comparison_mode",
            "match",
        }
        for invariant in report["run"]["parts"][0][
            "root_case_invariants"
        ].values():
            self.assertTrue(required_report_fields.issubset(invariant))

    def test_short_structural_pass_with_unsettled_tail_remains_smoke_only(self) -> None:
        run = self.base / "run-unsettled-smoke"
        write_run(run, [0.0, 1.0], mutate_final_velocity=0.1)
        report = self.report(run)
        self.assertEqual(
            report["verdict"]["status"],
            "PASS_U3_SOLVER_RUNTIME_SMOKE_WITH_UNSETTLED_TAIL",
        )
        self.assertTrue(report["run"]["structural_pass"])
        self.assertFalse(report["run"]["tail_pass"])
        self.assertTrue(report["run"]["short_duration_smoke"])
        self.assertFalse(report["verdict"]["settled_state_claim_allowed"])
        self.assertFalse(report["verdict"]["settled_state_freeze_eligible"])

    def test_adjacent_float32_mass_is_a_hard_invariant_failure(self) -> None:
        run = self.base / "run-mass-ulp"
        adjacent = _next_float32(_float32(8e-6))
        write_run(
            run,
            [0.0, 1.0],
            part_root_overrides={0: {"MassFluid": adjacent}},
        )
        report = self.report(run)
        part = report["run"]["parts"][0]
        mass_check = part["root_case_invariants"]["MassFluid"]
        self.assertEqual(report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC")
        self.assertFalse(part["root_case_invariants_match"])
        self.assertFalse(mass_check["match"])
        self.assertEqual(mass_check["observed"], adjacent)

    def test_later_part_root_mutation_fails_all_part_invariant_check(self) -> None:
        run = self.base / "run-later-root-mutation"
        adjacent = _next_float32(_float32(8e-6))
        write_run(
            run,
            [0.0, 1.0],
            part_root_overrides={1: {"MassFluid": adjacent}},
        )
        report = self.report(run)
        checks = report["run"]["checks"]["structural"]
        self.assertTrue(checks["first_snapshot_matches_fixed_case"])
        self.assertFalse(checks["all_part_root_invariants_match_fixed_case"])
        self.assertTrue(report["run"]["parts"][0]["root_case_invariants_match"])
        self.assertFalse(report["run"]["parts"][1]["root_case_invariants_match"])
        self.assertEqual(report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC")

    def test_initial_particle_array_mutation_remains_a_hard_failure(self) -> None:
        run = self.base / "run-initial-array-mutation"
        write_run(run, [0.0, 1.0], mutate_initial_position=1e-4)
        report = self.report(run)
        part = report["run"]["parts"][0]
        self.assertTrue(part["root_case_invariants_match"])
        self.assertFalse(part["matches_fixed_case_initial_arrays"])
        self.assertFalse(
            report["run"]["checks"]["structural"][
                "first_snapshot_matches_fixed_case"
            ]
        )
        self.assertEqual(report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC")

    def test_each_count_and_dp_mutation_remains_a_hard_failure(self) -> None:
        mutations: dict[str, int | float] = {
            "CaseNp": 131,
            "CaseNfixed": 3,
            "CaseNmoving": 1,
            "CaseNfloat": 1,
            "CaseNfluid": 129,
            "Dp": 0.0021,
        }
        for index, (field, value) in enumerate(mutations.items()):
            with self.subTest(field=field):
                run = self.base / f"run-root-{index}"
                write_run(
                    run,
                    [0.0, 1.0],
                    part_root_overrides={0: {field: value}},
                )
                report = self.report(run)
                invariant = report["run"]["parts"][0]["root_case_invariants"][field]
                self.assertFalse(invariant["match"])
                self.assertEqual(invariant["observed"], value)
                self.assertEqual(
                    report["verdict"]["status"], "FAIL_U3_SOLVER_OUTPUT_QC"
                )

    def test_missing_or_nonfinite_root_invariant_is_rejected(self) -> None:
        for index, field in enumerate(("MassFluid", "CaseNp")):
            with self.subTest(missing=field):
                missing = self.base / f"run-missing-root-{index}"
                write_run(
                    missing,
                    [0.0, 1.0],
                    part_omit_root_fields={0: {field}},
                )
                with self.assertRaisesRegex(qc.QcError, field):
                    self.report(missing)

        for index, field in enumerate(("MassFluid", "Dp")):
            with self.subTest(field=field):
                nonfinite = self.base / f"run-nonfinite-root-{index}"
                write_run(
                    nonfinite,
                    [0.0, 1.0],
                    part_root_overrides={0: {field: float("nan")}},
                )
                with self.assertRaisesRegex(qc.QcError, f"PART root {field}"):
                    self.report(nonfinite)

    def test_cli_requires_both_fixed_case_hashes(self) -> None:
        actions = {action.dest: action for action in qc._parser()._actions}
        self.assertTrue(actions["expected_case_bi4_sha256"].required)
        self.assertTrue(actions["expected_case_xml_sha256"].required)

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
