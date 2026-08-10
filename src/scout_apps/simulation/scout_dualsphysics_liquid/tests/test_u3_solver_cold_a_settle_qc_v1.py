#!/usr/bin/env python3
"""Offline-only tests for the frozen U3 cold-A settle QC wrapper."""

from __future__ import annotations

import hashlib
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT_DIR = PACKAGE / "scripts"
TEST_DIR = Path(__file__).resolve().parent
for path in (SCRIPT_DIR, TEST_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import r8_liquid_u3_solver_cold_a_settle_qc_v1 as cold_qc  # noqa: E402
import test_u3_solver_output_qc_v3 as fixture  # noqa: E402


TIMES = [index * 0.05 for index in range(162)]


def _run_csv(final_time: float, *, part_count: int = 162) -> bytes:
    values = [
        "run",
        "code",
        "date",
        "131",
        "1",
        "1",
        "1",
        "1",
        "0",
        "0",
        "16100",
        "",
        f"{final_time:.15g}",
        str(part_count),
        "0",
        "131",
        "20",
        "CPU",
        "CellsFull",
        "cfg",
        "3",
        "0",
        "0.002",
        "0.0034641016151",
        "0",
        "0",
    ]
    return (fixture.RUN_CSV_HEADER + ";".join(values) + "\n").encode()


def _runparts(
    times: list[float], *, mutate_last_npsave: bool = False
) -> bytes:
    header = (
        "Part;TimeStep [s];Steps;NpSave;NpSim;NpOut;NpbSim;NpfSim;"
        "NpNormal;NpOutPos;NpOutRho;NpOutMov\n"
    )
    rows = []
    for index, time_s in enumerate(times):
        npsave = 130 if mutate_last_npsave and index == len(times) - 1 else 131
        steps = 0 if index == 0 else 100
        rows.append(
            f"{index};{time_s:.15g};{steps};{npsave};131;0;3;128;131;0;0;0"
        )
    return (header + "\n".join(rows) + "\n").encode()


def write_cold_run(
    root: Path,
    *,
    effective_tmax: float = 8.05,
    mutate_last_npsave: bool = False,
    final_velocity: float | None = None,
    final_position_shift: float | None = None,
    final_density: float | None = None,
    final_velocity_nan: bool = False,
) -> None:
    root.mkdir()
    data = root / "data"
    data.mkdir()
    ids, positions, velocities, densities = fixture.particle_fixture()
    for index, time_s in enumerate(TIMES):
        current_positions = list(positions)
        current_velocities = list(velocities)
        current_densities = list(densities)
        if index == len(TIMES) - 1:
            if final_velocity is not None:
                current_velocities[3] = (final_velocity, 0.0, 0.0)
            if final_velocity_nan:
                current_velocities[3] = (float("nan"), 0.0, 0.0)
            if final_position_shift is not None:
                x, y, z = current_positions[3]
                current_positions[3] = (x + final_position_shift, y, z)
            if final_density is not None:
                current_densities[3:] = [final_density] * (len(ids) - 3)
        (data / f"Part_{index:04d}.bi4").write_bytes(
            fixture.make_part(
                index,
                time_s,
                positions=current_positions,
                velocities=current_velocities,
                densities=current_densities,
            )
        )
    (data / "PartOut_000.obi4").write_bytes(fixture.make_partout())
    (data / "PartMotionRef.ibi4").write_bytes(fixture.make_motion_ref(TIMES))
    (data / "Part_Head.ibi4").write_bytes(b"opaque Part_Head fixture\n")
    (data / "PartInfo.ibi4").write_bytes(b"opaque PartInfo fixture\n")
    (root / "CfgInit_Domain.vtk").write_bytes(b"# vtk fixture domain\n")
    (root / "CfgInit_MapCells.vtk").write_bytes(b"# vtk fixture map cells\n")
    (root / "Run.csv").write_bytes(_run_csv(TIMES[-1]))
    (root / "RunPARTs.csv").write_bytes(
        _runparts(TIMES, mutate_last_npsave=mutate_last_npsave)
    )
    (root / "Run.out").write_bytes(
        f"""DualSPHysics test
TimeMax={effective_tmax}
TimePart=0.05
[Simulation finished  test]
Excluded particles...............: 0
Finished execution (code=0).
""".encode()
    )


class U3SolverColdASettleQcV1Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-u3-cold-a-qc-")
        self.base = Path(self.temporary.name)
        self.case_bi4 = self.base / "C1M_zero.bi4"
        self.case_xml = self.base / "C1M_zero.xml"
        self.case_bi4.write_bytes(
            fixture.make_part(0, 0.0, root_overrides={"MassFluid": 8e-6})
        )
        self.case_xml.write_bytes(fixture.CASE_XML)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def report(self, run: Path) -> dict:
        return cold_qc.build_report(run, self.case_bi4, self.case_xml)

    def test_exact_quiescent_campaign_only_allows_cold_b(self) -> None:
        run = self.base / "cold-a-pass"
        write_cold_run(run)
        report = self.report(run)
        self.assertEqual(
            report["verdict"]["status"],
            "PASS_U3_COLD_A_DEVELOPMENT_SETTLE_QC_COLD_B_REQUIRED",
        )
        self.assertEqual(
            report["verdict"]["claim_ceiling"], "COLD_A_ONLY_NOT_SETTLED"
        )
        self.assertTrue(report["verdict"]["cold_a_pass"])
        self.assertEqual(report["verdict"]["decision"], "COLD_A_PASS_COLD_B_ONLY")
        self.assertTrue(report["verdict"]["cold_b_execution_eligible"])
        self.assertFalse(report["verdict"]["restart_execution_eligible"])
        self.assertFalse(report["verdict"]["settled_state_freeze_eligible"])
        self.assertFalse(report["verdict"]["u4_authorized"])
        self.assertEqual(report["inventory"]["file_count"], 171)
        self.assertEqual(len(report["inventory"]["files"]), 171)
        self.assertEqual(report["runparts"]["row_count"], 162)
        self.assertTrue(report["position_tail"]["pass"])
        self.assertTrue(report["density_tail"]["pass"])
        self.assertTrue(report["base_qc_v3_run"]["structural_pass"])

    def test_velocity_position_and_density_thresholds_are_pre_run_failures(self) -> None:
        run = self.base / "cold-a-unsettled"
        write_cold_run(
            run,
            final_velocity=0.02,
            final_position_shift=0.001,
            final_density=1060.0,
        )
        report = self.report(run)
        self.assertEqual(
            report["verdict"]["status"],
            "FAIL_U3_COLD_A_DEVELOPMENT_SETTLE_QC",
        )
        self.assertFalse(report["base_qc_v3_run"]["tail_pass"])
        self.assertFalse(report["position_tail"]["pass"])
        self.assertFalse(report["density_tail"]["pass"])
        self.assertFalse(report["verdict"]["cold_b_execution_eligible"])

    def test_nonfinite_velocity_fails_inherited_structural_qc(self) -> None:
        run = self.base / "cold-a-nan"
        write_cold_run(run, final_velocity_nan=True)
        report = self.report(run)
        self.assertFalse(report["base_qc_v3_run"]["structural_pass"])
        self.assertFalse(
            report["base_qc_v3_run"]["checks"]["structural"][
                "all_particle_arrays_finite"
            ]
        )
        self.assertFalse(report["verdict"]["cold_a_pass"])

    def test_inventory_is_exact_and_rejects_an_extra_file(self) -> None:
        run = self.base / "cold-a-extra"
        write_cold_run(run)
        (run / "unexpected.txt").write_text("no\n", encoding="utf-8")
        with self.assertRaisesRegex(cold_qc.ColdSettleQcError, "inventory differs"):
            self.report(run)

    def test_effective_tmax_and_runparts_counts_fail_closed(self) -> None:
        wrong_time = self.base / "cold-a-wrong-time"
        write_cold_run(wrong_time, effective_tmax=8.0)
        time_report = self.report(wrong_time)
        self.assertFalse(time_report["effective_solver_time_parameters"]["pass"])
        self.assertFalse(time_report["verdict"]["cold_a_pass"])

        wrong_counts = self.base / "cold-a-wrong-counts"
        write_cold_run(wrong_counts, mutate_last_npsave=True)
        count_report = self.report(wrong_counts)
        self.assertFalse(count_report["runparts"]["pass"])
        self.assertFalse(
            count_report["runparts"]["checks"]["particle_counts_match_case"]
        )
        self.assertFalse(count_report["verdict"]["cold_a_pass"])

    def test_report_publication_is_exclusive_read_only_and_bounded(self) -> None:
        output = self.base / "qc.json"
        publication = cold_qc.write_json_exclusive(
            output, {"finite": 1.0, "status": "test"}
        )
        original = output.read_bytes()
        self.assertEqual(publication["sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual(stat.S_IMODE(os.lstat(output).st_mode), 0o400)
        with self.assertRaisesRegex(cold_qc.ColdSettleQcError, "exclusive"):
            cold_qc.write_json_exclusive(output, {"status": "replacement"})
        self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
