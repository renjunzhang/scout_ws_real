#!/usr/bin/env python3
"""Static boundary tests for the non-runnable R7/R1 archive.

These tests deliberately inspect only this simulation package.  They make the
reviewed CMake allowlists, archive location, and R8/H0 import boundary
machine-checkable without importing an archived module or starting ROS.
"""

from __future__ import annotations

import importlib.util
import re
import stat
import subprocess
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[3]
CMAKE = PACKAGE / "CMakeLists.txt"
SCRIPTS = PACKAGE / "scripts"
SCRIPT_TESTS = SCRIPTS / "tests"
ARCHIVE = PACKAGE / "historical_quarantine" / "r7_r1"
SOURCE_GATE = SCRIPTS / "smpcc_sim_source_separation.py"
QUARANTINE_SENTINEL = "QUARANTINED_R7_R1_ARCHIVE"
RUNTIME_WRAPPER_ROOTS = (
    WORKSPACE / "devel/lib/spmpc_sim_local_planner",
    WORKSPACE / "install/lib/spmpc_sim_local_planner",
    Path("/data/a/scout_sim_replacement/r8_controller_ws/devel/lib/spmpc_sim_local_planner"),
    Path("/data/a/scout_sim_replacement/r8_controller_ws/install/lib/spmpc_sim_local_planner"),
)

ACTIVE_RUNTIME_SCRIPTS = frozenset((
    "smpcc_sim_controller_gate.py",
    "smpcc_sim_formal_campaign_runner.py",
    "smpcc_sim_formal_freeze_intake.py",
    "smpcc_sim_formal_runtime_adapter.py",
    "smpcc_sim_frozen_path_replay.py",
    "smpcc_sim_h0_fixed_path_publisher.py",
    "smpcc_sim_h0_runtime_adapter.py",
    "smpcc_sim_h_proxy_monitor.py",
    "smpcc_sim_r8_release.py",
    "smpcc_sim_source_separation.py",
    "smpcc_sim_toolchain.py",
))

ACTIVE_LAUNCH_FILES = frozenset((
    "smpcc_sim_environment.launch",
    "smpcc_sim_h0_fixed_path_publisher.launch",
    "smpcc_sim_h_proxy_monitor.launch",
    "smpcc_sim_localization.launch",
    "smpcc_sim_mechanism_r8.launch",
    "smpcc_sim_only_bslosh_r8.launch",
))

ACTIVE_SHELL_SCRIPTS = frozenset((
    "build_sim_controller_workspace.sh",
    "launch_h0_sim_controller.sh",
    "launch_sim_environment.sh",
))

LEGACY_SCRIPTS = frozenset((
    "smpcc_sim_dev_r1_pilot_assets.py",
    "smpcc_sim_development_candidate_assets.py",
    "smpcc_sim_mechanism_analysis.py",
    "smpcc_sim_mechanism_campaign_control.py",
    "smpcc_sim_mechanism_crn_contract_builder.py",
    "smpcc_sim_mechanism_crn_runtime.py",
    "smpcc_sim_mechanism_endpoint_gate.py",
    "smpcc_sim_mechanism_fixed_profile_tracker.py",
    "smpcc_sim_mechanism_freeze.py",
    "smpcc_sim_mechanism_matrix.py",
    "smpcc_sim_mechanism_release.py",
    "smpcc_sim_mechanism_runner.py",
    "smpcc_sim_mechanism_runtime_qc.py",
    "smpcc_sim_mechanism_timing_pilot.py",
    "smpcc_sim_only_bslosh_r1_runner.py",
    "smpcc_sim_only_bslosh_release.py",
    "smpcc_sim_physical_command_alignment_audit.py",
))

LEGACY_TESTS = frozenset((
    "test_smpcc_sim_dev_r1_pilot_assets.py",
    "test_smpcc_sim_development_candidate_assets.py",
    "test_smpcc_sim_mechanism_analysis.py",
    "test_smpcc_sim_mechanism_campaign_control.py",
    "test_smpcc_sim_mechanism_crn_contract_builder.py",
    "test_smpcc_sim_mechanism_crn_runtime.py",
    "test_smpcc_sim_mechanism_endpoint_gate.py",
    "test_smpcc_sim_mechanism_fixed_profile_tracker.py",
    "test_smpcc_sim_mechanism_freeze.py",
    "test_smpcc_sim_mechanism_launch_contract.py",
    "test_smpcc_sim_mechanism_launch_runtime.py",
    "test_smpcc_sim_mechanism_matrix.py",
    "test_smpcc_sim_mechanism_release.py",
    "test_smpcc_sim_mechanism_runner.py",
    "test_smpcc_sim_mechanism_runtime_qc.py",
    "test_smpcc_sim_mechanism_timing_pilot.py",
    "test_smpcc_sim_only_bslosh_r1_runner.py",
    "test_smpcc_sim_only_bslosh_release.py",
    "test_smpcc_sim_physical_command_alignment_audit.py",
))

LEGACY_PROFILE_GENERATORS = frozenset((
    "common/advanced_profile_common.py",
    "common/path_profile_utils.py",
    "hamaguchi/generate_profile.py",
))
LEGACY_CONFIG_FIXTURES = frozenset(("historical_0705_weights_fixture.json",))


def cmake_list(name: str) -> frozenset[str]:
    text = CMAKE.read_text(encoding="utf-8")
    match = re.search(rf"set\({re.escape(name)}\s*(.*?)\n\)", text, re.DOTALL)
    if match is None:
        raise AssertionError(f"CMake list {name} is missing")
    return frozenset(
        line.strip()
        for line in match.group(1).splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )


def load_source_gate():
    spec = importlib.util.spec_from_file_location("smpcc_sim_source_separation_quarantine_test", SOURCE_GATE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SOURCE_GATE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LegacyExecutionQuarantineTest(unittest.TestCase):
    def test_cmake_installs_only_explicit_reviewed_entrypoints(self) -> None:
        text = CMAKE.read_text(encoding="utf-8")
        self.assertNotIn("file(GLOB SPMPC_SIM_RUNTIME_SCRIPTS", text)
        self.assertEqual(
            {f"scripts/{name}" for name in ACTIVE_RUNTIME_SCRIPTS},
            set(cmake_list("SPMPC_SIM_ACTIVE_RUNTIME_SCRIPTS")),
        )
        self.assertEqual(
            {f"launch/{name}" for name in ACTIVE_LAUNCH_FILES},
            set(cmake_list("SPMPC_SIM_ACTIVE_LAUNCH_FILES")),
        )
        self.assertEqual(
            {f"scripts/{name}" for name in ACTIVE_SHELL_SCRIPTS},
            set(cmake_list("SPMPC_SIM_ACTIVE_SHELL_SCRIPTS")),
        )
        for legacy in LEGACY_SCRIPTS:
            self.assertNotIn(legacy, text)
        self.assertNotIn("install(DIRECTORY config launch generated/acados", text)

    def test_legacy_entrypoints_and_tests_are_non_executable_archives(self) -> None:
        self.assertTrue((PACKAGE / "historical_quarantine/README.md").is_file())
        for name in LEGACY_SCRIPTS:
            self.assertFalse((SCRIPTS / name).exists(), name)
            archived = ARCHIVE / "scripts" / f"{name}.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(0, stat.S_IMODE(archived.stat().st_mode) & 0o111, archived)
        for name in LEGACY_TESTS:
            self.assertFalse((SCRIPT_TESTS / name).exists(), name)
            archived = ARCHIVE / "tests" / f"{name}.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(0, stat.S_IMODE(archived.stat().st_mode) & 0o111, archived)
        legacy_launch = ARCHIVE / "launch/smpcc_sim_mechanism_fixed_profile.launch.disabled"
        self.assertFalse((PACKAGE / "launch/smpcc_sim_mechanism_fixed_profile.launch").exists())
        self.assertTrue(legacy_launch.is_file())
        self.assertEqual(0, stat.S_IMODE(legacy_launch.stat().st_mode) & 0o111)
        for relative in LEGACY_PROFILE_GENERATORS:
            self.assertFalse((SCRIPTS / "fixed_profile" / relative).exists(), relative)
            archived = ARCHIVE / "fixed_profile" / f"{relative}.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(0, stat.S_IMODE(archived.stat().st_mode) & 0o111, archived)
        for name in LEGACY_CONFIG_FIXTURES:
            self.assertFalse((PACKAGE / "config" / name).exists(), name)
            archived = ARCHIVE / "config" / f"{name}.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(0, stat.S_IMODE(archived.stat().st_mode) & 0o111, archived)
        for name in LEGACY_SCRIPTS:
            stem = name[:-3]
            self.assertFalse(list((SCRIPTS / "__pycache__").glob(f"{stem}.*.pyc")), stem)
            archived = ARCHIVE / "bytecode/scripts" / f"{stem}.cpython-38.pyc.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(b"\0\0\0\0", archived.read_bytes()[:4], archived)
        for name in LEGACY_TESTS:
            stem = name[:-3]
            self.assertFalse(list((SCRIPT_TESTS / "__pycache__").glob(f"{stem}.*.pyc")), stem)
            archived = ARCHIVE / "bytecode/tests" / f"{stem}.cpython-38.pyc.disabled"
            self.assertTrue(archived.is_file(), archived)
            self.assertEqual(b"\0\0\0\0", archived.read_bytes()[:4], archived)
        archived_python = sorted(ARCHIVE.rglob("*.py.disabled"))
        self.assertTrue(archived_python)
        for path in archived_python:
            self.assertIn(QUARANTINE_SENTINEL, path.read_text(encoding="utf-8"), path)

    def test_explicit_python_cannot_bypass_historical_archive_quarantine(self) -> None:
        # A non-executable suffix alone is insufficient: Python accepts an
        # arbitrary filename.  Exercise the most consequential former entry
        # point and require its sentinel to abort before any imports/builds.
        legacy_freeze = ARCHIVE / "scripts/smpcc_sim_mechanism_freeze.py.disabled"
        completed = subprocess.run(
            [sys.executable, str(legacy_freeze)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertNotEqual(0, completed.returncode)
        self.assertIn(QUARANTINE_SENTINEL, completed.stderr)

        archived_bytecode = ARCHIVE / "bytecode/scripts/smpcc_sim_mechanism_freeze.cpython-38.pyc.disabled"
        bytecode = subprocess.run(
            [sys.executable, str(archived_bytecode)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertNotEqual(0, bytecode.returncode)

        for root in RUNTIME_WRAPPER_ROOTS:
            wrapper_archive = root / "historical_quarantine_r7_r1"
            if not wrapper_archive.is_dir():
                continue
            for name in LEGACY_SCRIPTS:
                wrapper = wrapper_archive / f"{name}.disabled"
                self.assertTrue(wrapper.is_file(), wrapper)
                self.assertIn(QUARANTINE_SENTINEL, wrapper.read_text(encoding="utf-8"))

    def test_existing_devel_or_install_space_has_no_stale_legacy_wrapper(self) -> None:
        # Catkin's devel-space wrappers are generated files and can survive a
        # historical glob-install build.  When such a space exists, reject
        # the stale executable surface rather than treating the new CMake
        # list as sufficient evidence by itself.
        for root in RUNTIME_WRAPPER_ROOTS:
            if not root.exists():
                continue
            for name in LEGACY_SCRIPTS:
                self.assertFalse((root / name).exists(), root / name)

    def test_active_r8_h0_import_graph_never_names_legacy_modules(self) -> None:
        active_text = "\n".join(
            (SCRIPTS / name).read_text(encoding="utf-8")
            for name in sorted(ACTIVE_RUNTIME_SCRIPTS)
        )
        for legacy in LEGACY_SCRIPTS:
            self.assertNotIn(legacy[:-3], active_text)

    def test_active_code_and_launch_surface_never_addresses_real_stack_source(self) -> None:
        active_text = "\n".join(
            (SCRIPTS / name).read_text(encoding="utf-8")
            for name in sorted(ACTIVE_RUNTIME_SCRIPTS)
        ) + "\n" + "\n".join(
            (PACKAGE / "launch" / name).read_text(encoding="utf-8")
            for name in sorted(ACTIVE_LAUNCH_FILES)
        )
        for forbidden in (
            "src/scout_apps/control/",
            "control/slosh_models",
            "find_package(spmpc_local_planner",
            "find_package(slosh_models",
        ):
            self.assertNotIn(forbidden, active_text)

        # The two shell launchers may name forbidden packages only inside
        # explicit deny guards.  They must never launch/source any real-stack
        # controller, experiment, or liquid-model package.
        shell_text = "\n".join(
            (SCRIPTS / name).read_text(encoding="utf-8")
            for name in sorted(ACTIVE_SHELL_SCRIPTS)
        )
        for forbidden_command in (
            "roslaunch spmpc_local_planner",
            "roslaunch spmpc_experiments",
            "roslaunch slosh_models",
            "rosrun spmpc_local_planner",
            "rosrun spmpc_experiments",
            "rosrun slosh_models",
            "source /home/a/scout_ws/src/scout_apps/control/",
        ):
            self.assertNotIn(forbidden_command, shell_text)

    def test_r8_source_registry_explicitly_excludes_historical_quarantine(self) -> None:
        gate = load_source_gate()
        sample = ARCHIVE / "scripts/smpcc_sim_mechanism_freeze.py.disabled"
        self.assertFalse(gate._include_tree_file(sample))
        self.assertNotIn(
            gate.HISTORICAL_QUARANTINE_ROOT,
            {root for root, _files in gate.PACKAGE_TREE_ROOTS.values()},
        )


if __name__ == "__main__":
    unittest.main()
