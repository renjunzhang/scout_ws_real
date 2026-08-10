#!/usr/bin/env python3
"""Root lifecycle supervisor for one fresh U3 C1M cold-A settle v7.

Workspace commands are read-only reviewers and snapshot-byte producers.  The
hidden production ``run`` entry works only from a root-owned immutable snapshot.
It loads two fresh transient AppArmor labels, hands a fixed stdin frame to the
UID-1000/NNP gate, closes a zero-unexpected-logged-denial journal window while
preserving two exact explicit unlogged deny rules, removes all labelled tasks and both
profiles, and only then permits a separate UID-1000/NNP process to export the
exact 171-file solver tree with dirfd/O_NOFOLLOW/O_EXCL.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import importlib.util
import json
import os
import signal
import stat
import struct
import subprocess
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
WORKSPACE_PACKAGE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v7.py"
CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v7_20260809T051659Z"
CAMPAIGN_ID = "u3_c1m_solver_cpu_settle_ab_campaign_20260809T051659Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v7-20260809t051659z"
RUNTIME_PROFILE = "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v7-20260809t051659z"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V7_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V7_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V7_ADMISSION_SHA256"
EXPORT_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V7_EXPORT_FD"
EXPORT_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V7_EXPORT_SHA256"
ADMISSION_TOKEN = (
    f"{CAMPAIGN_ID}:{CASE_ID}:single-cpu-solver-settle-cold-a-v7-attempt"
)
POLICY_STATUS = "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION"
CACHE_QUIET_DENY_RULE = "deny /etc/ld.so.cache r,"
CAPABILITY_QUIET_DENY_RULE = "deny capability dac_override,"
KNOWN_QUIET_DENY_RULES = (CACHE_QUIET_DENY_RULE, CAPABILITY_QUIET_DENY_RULE)
KNOWN_QUIET_DENY_ORIGINS = {
    CACHE_QUIET_DENY_RULE: "carried_forward_from_v2_after_v1_observed_bwrap_dynamic_loader_probe",
    CAPABILITY_QUIET_DENY_RULE: "carried_forward_from_v2_preexisting_v1_bwrap_capability_rule",
}
DENIAL_VISIBILITY_BOUNDARY = (
    "closed_zero_unexpected_logged_DENIED_records_with_two_explicit_unlogged_deny_rules;"
    "both_carried_from_successful_v2;"
    "does_not_claim_zero_denied_operations"
)

POLICY_NAME = "liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v7.json"
SCHEMA_NAME = "target_host_u3_solver_cpu_settle_cold_a_execution_policy_v7.json"
PROFILE_NAME = "r8-liquid-u3-solver-cpu-settle-cold-a-v7.profile"
GATE_NAME = "r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v7.py"
HELPER_NAME = "r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v7.py"
SUPERVISOR_NAME = "r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v7.py"
HARNESS_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"

IN_SNAPSHOT = SCRIPT_PATH.parent == SNAPSHOT_ROOT
BASE = SNAPSHOT_ROOT if IN_SNAPSHOT else WORKSPACE_PACKAGE_ROOT
POLICY_PATH = BASE / POLICY_NAME if IN_SNAPSHOT else BASE / "config/target_hosts" / POLICY_NAME
SCHEMA_PATH = BASE / SCHEMA_NAME if IN_SNAPSHOT else BASE / "schema" / SCHEMA_NAME
PROFILE_PATH = BASE / PROFILE_NAME if IN_SNAPSHOT else BASE / "config/apparmor_drafts" / PROFILE_NAME
GATE_PATH = BASE / GATE_NAME if IN_SNAPSHOT else BASE / "scripts" / GATE_NAME
HELPER_PATH = BASE / HELPER_NAME if IN_SNAPSHOT else BASE / "scripts" / HELPER_NAME
HARNESS_PATH = BASE / HARNESS_NAME if IN_SNAPSHOT else BASE / "scripts" / HARNESS_NAME

ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.start.json")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")

BUILD_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_source_cpu_build_20260807T023724Z_cpu_build.json")
BUILD_RECEIPT_SHA256 = "d407233107d4bc9eeea81b0c5a95cbfc98ad6529f1cb7494b68ae5ecc4b1d604"
SEED_ID = "u3_c1m_gencase_seed_v3_20260808T150802Z"
SEED_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json")
SEED_RECEIPT_SHA256 = "79f43b54594843e0107aa78ad01db34937fa97f0a34a16d96e74ac9cd90095ed"
SEED_RECEIPT_SIZE = 12_382
SEED_RECEIPT_STATUS = "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION"
GENCASE_EXECUTION_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gencase_v8_20260808T153753Z.execution.json")
GENCASE_EXECUTION_SHA256 = "113ce7d570a15abe415cdd3e703604870c0980eaad37f845fe1b71a2242aac32"
GENCASE_LIFECYCLE_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gencase_v8_20260808T153753Z.lifecycle.json")
GENCASE_LIFECYCLE_SHA256 = "d595dc8665175f0b6e2dca0e88db41c9bcba4c9dbb63cec97c827a20459a7eea"

V1_CASE_ID = "u3_c1_solver_cpu_smoke_v1_20260808T124816Z"
V1_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.start.json")
V1_START_RECEIPT_SHA256 = "779feb5bdccdb2c833a4c04dd06ebf6241434b3343a3506753a9b82544758238"
V1_START_RECEIPT_SIZE = 10_274
V1_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.execution.json")
V1_EXECUTION_RECEIPT_SHA256 = "6c8a2600bcb40c1039976061ec296509c4a2d6d6e89e941de9b7bfb4edf48494"
V1_EXECUTION_RECEIPT_SIZE = 7_614
V1_INCOMPLETE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.lifecycle_incomplete.json")
V1_INCOMPLETE_RECEIPT_SHA256 = "412d7e8a5c1adfc340707f6af608ab29c23f2a9446dfa6c32453a12b48caa090"
V1_INCOMPLETE_RECEIPT_SIZE = 5_037

V2_CASE_ID = "u3_c1_solver_cpu_smoke_v2_20260808T134452Z"
V2_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.start.json")
V2_START_RECEIPT_SHA256 = "6050a28d4f1093761d94a502df8635bb5964eaa161aa3f69c20e63c056799a63"
V2_START_RECEIPT_SIZE = 10_966
V2_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.execution.json")
V2_EXECUTION_RECEIPT_SHA256 = "bfa560359e045158a25da9c75850f4a3c40f0fb22012e410f2abab46f36c88d0"
V2_EXECUTION_RECEIPT_SIZE = 42_463
V2_LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.lifecycle.json")
V2_LIFECYCLE_RECEIPT_SHA256 = "e01f81c864fa444b312dbe68b6d5490d45f53d8f99d180e3c71600fbcf6bad39"
V2_LIFECYCLE_RECEIPT_SIZE = 6_337

V3_CASE_ID = "u3_c1m_solver_cpu_smoke_v3_20260808T160108Z"
V3_POLICY = WORKSPACE_PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_execution_policy_v3.json"
V3_POLICY_SHA256 = "afef0482dabdde27c3552aa766a426a5c842c0c8d33ebb9f2c947c898d1fa6a0"
V3_POLICY_SIZE = 19_495
V3_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.start.json")
V3_START_RECEIPT_SHA256 = "8a93606fa656bbd301ec6214809ea4f315a5e9c53863fcbe12af57d57d6e95cc"
V3_START_RECEIPT_SIZE = 11_050
V3_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.execution.json")
V3_EXECUTION_RECEIPT_SHA256 = "8ff89dd45a6548d684e75b18ef4443f63728c5604a850e81da890df7c74d916f"
V3_EXECUTION_RECEIPT_SIZE = 43_646
V3_LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.lifecycle.json")
V3_LIFECYCLE_RECEIPT_SHA256 = "8d4541cfbba652dc0849dfaceb6ef1be2a876a0ee36136247365decef5f2ffe9"
V3_LIFECYCLE_RECEIPT_SIZE = 6_409
V3_VISUALIZATION_ROOT = Path(f"/home/zrj/scout_liquid_lab/visualizations/{V3_CASE_ID}_v2")
V3_QC_REPORT = V3_VISUALIZATION_ROOT / "reports/solver_output_qc_v3.json"
V3_QC_REPORT_SHA256 = "e008fe0201802899a77e2f50bf273ef18c7cef92ca6d9a4a6d33c17adc47ee3e"
V3_QC_REPORT_SIZE = 114_699
V3_VISUALIZATION_MANIFEST = V3_VISUALIZATION_ROOT / "artifact_manifest.json"
V3_VISUALIZATION_MANIFEST_SHA256 = "d096e6663352b464646b81cea138797c98d86c8b6d351b2595a0b157cb003636"
V3_VISUALIZATION_MANIFEST_SIZE = 6_720

V4_CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v4_20260808T174116Z"
V4_POLICY = WORKSPACE_PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v4.json"
V4_POLICY_SHA256 = "df10e6a2f9075f4f4a11911bd341626009143af37d047ce97bb3b2eff7d4ab14"
V4_POLICY_SIZE = 27_078
V4_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.start.json")
V4_START_RECEIPT_SHA256 = "2902ec385337b3a5f386ee05d94e55be02a0b1a9710d562245bc9dc3f7c10f42"
V4_START_RECEIPT_SIZE = 11_531
V4_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.execution.json")
V4_EXECUTION_RECEIPT_SHA256 = "710086a44417cb41646c57dcdebcca6b8625b0619bf07e1459d9ed653392979e"
V4_EXECUTION_RECEIPT_SIZE = 5_197
V4_INCOMPLETE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.lifecycle_incomplete.json")
V4_INCOMPLETE_RECEIPT_SHA256 = "4a9ff90fb7754c93ac0cb79a27bb9b2699b543cb0e638683130a8a8059fc33e7"
V4_INCOMPLETE_RECEIPT_SIZE = 6_123
V4_WORKSPACE_FREEZE: dict[str, tuple[Path, str, int]] = {
    "schema": (
        WORKSPACE_PACKAGE_ROOT / "schema/target_host_u3_solver_cpu_settle_cold_a_execution_policy_v4.json",
        "d5c16dc0bc3783e23bd19a2f07059f9d9e9d44a13e8ae4e485c4c533cad9754a",
        2_266,
    ),
    "profile": (
        WORKSPACE_PACKAGE_ROOT / "config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v4.profile",
        "9e6b73a257ec791a6e4e06489a620d353248ccddc7cd22fcf948347e45c7896c",
        3_233,
    ),
    "gate": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v4.py",
        "c6c06e062484df1e7605826ad604ba9fd0a2fac233accfbec6941adffd0c6bb3",
        42_540,
    ),
    "helper": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v4.py",
        "8c52ee47941cf7ed59f466cb03edd174b82e4c34bdca758c085573f217b8fdeb",
        26_466,
    ),
    "supervisor": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v4.py",
        "23ad114bdfdd80141aa785144a9a392e03fe2b9f28d58968b28d4b5e41938ccc",
        74_640,
    ),
    "test": (
        WORKSPACE_PACKAGE_ROOT / "tests/test_target_u3_solver_cpu_settle_cold_a_gate_v4.py",
        "1a5bbe0d9e415d33b77d94b275642e92cd7da5d916980e444c69fc98f2e3d99b",
        34_228,
    ),
}
V5_CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v5_20260808T203712Z"
V5_CAMPAIGN_ID = "u3_c1m_solver_cpu_settle_ab_campaign_20260808T203712Z"
V5_POLICY = WORKSPACE_PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v5.json"
V5_POLICY_SHA256 = "d558833403082e9959c145d563fdecbbb7de740cbf6d3a9d0117fdb4d9d5b866"
V5_POLICY_SIZE = 32_340
V5_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.start.json")
V5_START_RECEIPT_SHA256 = "017774ba99487451b6e6a3dbaec58c54ee521829d0f9f486424963deefad5b90"
V5_START_RECEIPT_SIZE = 11_634
V5_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.execution.json")
V5_EXECUTION_RECEIPT_SHA256 = "80e4cd66e8dabac2b80b18050a851a23b9cc5942ab073f1a2e3e6f093c028a0e"
V5_EXECUTION_RECEIPT_SIZE = 13_701
V5_INCOMPLETE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.lifecycle_incomplete.json")
V5_INCOMPLETE_RECEIPT_SHA256 = "d246d6a266198a253ee476da42f91344e2a106480b680140b6727e28cb1397c1"
V5_INCOMPLETE_RECEIPT_SIZE = 6_239
V5_WORKSPACE_FREEZE: dict[str, tuple[Path, str, int]] = {
    "schema": (
        WORKSPACE_PACKAGE_ROOT / "schema/target_host_u3_solver_cpu_settle_cold_a_execution_policy_v5.json",
        "280a3500acb3a2cd8bee06f03cb8e508f2a8f734150305dc2201a0d913b94dca",
        2_266,
    ),
    "profile": (
        WORKSPACE_PACKAGE_ROOT / "config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v5.profile",
        "70802ebedca241b5033ae84bec24ba98699489f8df77d067711b7a86fa2948d0",
        3_383,
    ),
    "gate": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v5.py",
        "a7d44ce2a037744b62b20904dcf78fc2e5ef520a09806afc46bef60a43f3e1f8",
        59_086,
    ),
    "helper": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v5.py",
        "815a019d240119c62b6d5146306ebd82f4dcd13b5ce8689f435ca4e1f3eb8f24",
        36_293,
    ),
    "supervisor": (
        WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v5.py",
        "9a8034caebd18809d37be55bd94085ac7bdff65cd5a4e556f018ff80b4a5b875",
        82_704,
    ),
    "test": (
        WORKSPACE_PACKAGE_ROOT / "tests/test_target_u3_solver_cpu_settle_cold_a_gate_v5.py",
        "970e87c13110b8221fe65188ee737c6ef547c1d8c2999ae11e3a0f6772cfb91c",
        63_284,
    ),
}

INPUT_SOURCES: tuple[tuple[str, str, str, int], ...] = (
    (
        "DualSPHysics5.4CPU_linux64",
        "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260807T023724Z.partial/output/artifacts/DualSPHysics5.4CPU_linux64",
        "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202",
        32_649_520,
    ),
    (
        "DsphConfig.xml",
        "/home/zrj/scout_liquid_lab/dependency/runtime/u3_c1m_gencase_seed_v3_20260808T150802Z.partial/input/DsphConfig.xml",
        "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
        293,
    ),
    (
        "C1M_zero.xml",
        "/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.xml",
        "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb",
        7_800,
    ),
    (
        "C1M_zero.bi4",
        "/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.bi4",
        "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7",
        400_842,
    ),
    (
        "ld-linux-x86-64.so.2",
        "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2",
        "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81",
        236_616,
    ),
    (
        "libgomp.so.1",
        "/usr/lib/x86_64-linux-gnu/libgomp.so.1.0.0",
        "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1",
        352_304,
    ),
    (
        "libstdc++.so.6",
        "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33",
        "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11",
        2_592_224,
    ),
    (
        "libm.so.6",
        "/usr/lib/x86_64-linux-gnu/libm.so.6",
        "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91",
        952_616,
    ),
    (
        "libgcc_s.so.1",
        "/usr/lib/x86_64-linux-gnu/libgcc_s.so.1",
        "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb",
        183_024,
    ),
    (
        "libc.so.6",
        "/usr/lib/x86_64-linux-gnu/libc.so.6",
        "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161",
        2_125_328,
    ),
)

EXPECTED_ROOT_FILES = (
    "CfgInit_Domain.vtk",
    "CfgInit_MapCells.vtk",
    "Run.csv",
    "Run.out",
    "RunPARTs.csv",
)
EXPECTED_DATA_FILES = (
    "PartInfo.ibi4",
    "PartOut_000.obi4",
    "Part_Head.ibi4",
    "PartMotionRef.ibi4",
    *(f"Part_{index:04d}.bi4" for index in range(162)),
)
EXPECTED_PATHS = tuple(sorted(EXPECTED_ROOT_FILES + tuple(f"data/{name}" for name in EXPECTED_DATA_FILES)))

OUTPUT_MAGIC = b"R8SOLVEROUTV7\0\0\0"
OUTPUT_VERSION = 7
OUTPUT_HEADER = struct.Struct(">16sIIQQQ")
OUTPUT_METADATA_LIMIT = 262_144
OUTPUT_TOTAL_LIMIT = 83_886_080
OUTPUT_CONSOLE_LIMIT = 4_194_304
OUTPUT_FRAME_LIMIT = OUTPUT_HEADER.size + OUTPUT_METADATA_LIMIT + OUTPUT_TOTAL_LIMIT + OUTPUT_CONSOLE_LIMIT
OUTER_WALL_TIMEOUT_SECONDS = 10_800
EXPORT_TIMEOUT_SECONDS = 300

V11_HARNESS_SHA256 = "72b7f1bf287b4fe750b46dccdb7a1eae9379230c7bdc30dbc6a6cb33a403ef5b"
V11_HARNESS_SIZE = 159_126
SYSCTL_PATHS = (
    Path("/proc/sys/user/max_user_namespaces"),
    Path("/proc/sys/kernel/unprivileged_userns_clone"),
    Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"),
)

SNAPSHOT_SOURCE_PAIRS = (
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / GATE_NAME), GATE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HELPER_NAME), HELPER_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / SUPERVISOR_NAME), SUPERVISOR_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/apparmor_drafts" / PROFILE_NAME), PROFILE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "schema" / SCHEMA_NAME), SCHEMA_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/target_hosts" / POLICY_NAME), POLICY_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HARNESS_NAME), HARNESS_NAME),
    *((source, name) for name, source, _digest, _size in INPUT_SOURCES),
)

CONSUMED_ATTEMPT_SOURCE_MARKERS = (
    V4_CASE_ID,
    V5_CASE_ID,
    "/run/r8-liquid-",
)
CONSUMED_ATTEMPT_SNAPSHOT_NAME_MARKERS = ("cold_a_v4", "cold_a_v5")


def verify_fresh_source_references(
    sources: tuple[tuple[str, str], ...],
    *,
    context: str,
    forbidden_name_markers: tuple[str, ...] = (),
) -> None:
    """Reject consumed-attempt or transient paths before any source is read."""
    for source, name in sources:
        if not isinstance(source, str) or not isinstance(name, str):
            raise SupervisorError(f"v7 {context} source reference type differs")
        if any(marker in source for marker in CONSUMED_ATTEMPT_SOURCE_MARKERS) or any(
            marker in name for marker in forbidden_name_markers
        ):
            raise SupervisorError(
                f"consumed v4/v5 identity or transient snapshot appeared in v7 {context}"
            )

SNAPSHOT_BOOTSTRAP_SOURCE = f'''import hashlib,json,os,stat,sys
ROOT="/run"
NAME="r8-liquid-{CASE_ID}.snapshot"
SOURCES={SNAPSHOT_SOURCE_PAIRS!r}
def open_abs(path):
 parts=[p for p in path.split("/") if p]
 d=os.open("/",os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:
  for part in parts[:-1]:
   n=os.open(part,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
   os.close(d); d=n
  return os.open(parts[-1],os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
 finally: os.close(d)
def read_source(path,size,digest):
 f=open_abs(path)
 try:
  m=os.fstat(f)
  if not stat.S_ISREG(m.st_mode) or m.st_nlink!=1 or m.st_size!=size: raise SystemExit(71)
  b=bytearray()
  while len(b)<size:
   q=os.read(f,min(1048576,size-len(b)))
   if not q: raise SystemExit(72)
   b.extend(q)
  if os.read(f,1)!=b"" or hashlib.sha256(b).hexdigest()!=digest: raise SystemExit(73)
  return bytes(b)
 finally: os.close(f)
if os.geteuid()!=0 or len(sys.argv)!=1+2*len(SOURCES): raise SystemExit(70)
payloads=[]
for i,(source,name) in enumerate(SOURCES):
 digest=sys.argv[1+2*i]; size=int(sys.argv[2+2*i])
 if len(digest)!=64 or size<1 or size>83886080: raise SystemExit(74)
 payloads.append((name,read_source(source,size,digest),digest,size))
runfd=os.open(ROOT,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
try:
 os.mkdir(NAME,0o700,dir_fd=runfd)
 snapfd=os.open(NAME,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=runfd)
 try:
  manifest=[]
  for name,raw,digest,size in payloads:
   f=os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o400,dir_fd=snapfd)
   try:
    view=memoryview(raw)
    while view:
     count=os.write(f,view)
     if count<=0: raise SystemExit(75)
     view=view[count:]
    os.fsync(f); os.fchmod(f,0o444); os.fsync(f); m=os.fstat(f)
    if m.st_uid!=0 or m.st_gid!=0 or m.st_nlink!=1 or stat.S_IMODE(m.st_mode)!=0o444: raise SystemExit(76)
   finally: os.close(f)
   manifest.append({{"name":name,"sha256":digest,"size_bytes":size}})
  os.fchmod(snapfd,0o555); os.fsync(snapfd)
 finally: os.close(snapfd)
 os.fsync(runfd)
finally: os.close(runfd)
os.write(1,(json.dumps({{"status":"ROOT_SOLVER_CPU_SETTLE_COLD_A_V7_SNAPSHOT_CREATED_NOT_EXECUTED","files":manifest}},sort_keys=True,separators=(",",":"))+"\\n").encode("ascii"))
'''
SNAPSHOT_BOOTSTRAP_BYTES = SNAPSHOT_BOOTSTRAP_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_BYTES).hexdigest()
SNAPSHOT_BOOTSTRAP_LOADER_SOURCE = f'''import hashlib,sys
B=sys.stdin.buffer
n={len(SNAPSHOT_BOOTSTRAP_BYTES)}
p=bytearray()
while len(p)<n:
 q=B.read(n-len(p))
 if not q: raise SystemExit(81)
 p.extend(q)
p=bytes(p)
if B.read(1)!=b"" or hashlib.sha256(p).hexdigest()!="{SNAPSHOT_BOOTSTRAP_SHA256}": raise SystemExit(82)
g={{"__name__":"__main__"}}
exec(compile(p,"<r8-liquid-solver-cpu-settle-cold-a-v7-root-snapshot-bootstrap>","exec",dont_inherit=True,optimize=2),g,g)
'''
SNAPSHOT_BOOTSTRAP_LOADER_BYTES = SNAPSHOT_BOOTSTRAP_LOADER_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_LOADER_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_LOADER_BYTES).hexdigest()

sys.dont_write_bytecode = True


class SupervisorError(RuntimeError):
    """A fail-closed supervisor, frame, export or lifecycle error."""


def read_regular_bytes(path: Path, *, limit: int = 80 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= limit:
            raise SupervisorError(f"unsafe regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise SupervisorError(f"short read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError(f"file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def read_regular_bytes_allow_empty(path: Path, *, limit: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 <= metadata.st_size <= limit:
            raise SupervisorError(f"unsafe bounded regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise SupervisorError(f"short bounded read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError(f"bounded file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path, limit=2 * 1024 * 1024).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON root is not an object: {path}")
    return value


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SupervisorError(f"cannot load reviewed module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def artifact_paths() -> dict[str, Path]:
    return {
        "gate": GATE_PATH,
        "helper": HELPER_PATH,
        "supervisor": SCRIPT_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def verify_receipt(path: Path, digest: str, status: str, *, expected_size: int | None = None) -> dict[str, Any]:
    raw = read_regular_bytes(path, limit=256 * 1024)
    if sha256_bytes(raw) != digest or (expected_size is not None and len(raw) != expected_size):
        raise SupervisorError(f"frozen receipt byte identity differs: {path.name}")
    try:
        value = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"frozen receipt is invalid: {path.name}") from exc
    if not isinstance(value, dict) or value.get("status") != status:
        raise SupervisorError(f"frozen receipt status differs: {path.name}")
    return {"path": str(path), "sha256": digest, "size_bytes": len(raw), "status": status}


def verify_seed_receipt() -> dict[str, Any]:
    """Close the provenance chain for the directly consumed DsphConfig.xml."""

    try:
        raw = read_regular_bytes(SEED_RECEIPT, limit=256 * 1024)
    except OSError as exc:
        raise SupervisorError("cannot read frozen C1M seed v3 receipt") from exc
    if len(raw) != SEED_RECEIPT_SIZE or sha256_bytes(raw) != SEED_RECEIPT_SHA256:
        raise SupervisorError("frozen C1M seed v3 receipt byte identity differs")
    try:
        value = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("frozen C1M seed v3 receipt is invalid") from exc
    if not isinstance(value, dict):
        raise SupervisorError("frozen C1M seed v3 receipt root differs")
    if value.get("status") != SEED_RECEIPT_STATUS or value.get("seed_id") != SEED_ID:
        raise SupervisorError("frozen C1M seed v3 receipt identity/status differs")
    for flag in (
        "compiled_artifact_executed",
        "precompiled_binary_executed",
        "upstream_code_executed",
        "network_used",
        "gpu_device_exposed",
        "sudo_used",
        "system_packages_changed",
        "source_checkout_created",
        "predecessor_seed_used_as_source",
    ):
        if value.get(flag) is not False:
            raise SupervisorError(f"C1M seed v3 non-execution flag differs: {flag}")
    seed_input = value.get("seed_input", {})
    files = seed_input.get("files", {}) if isinstance(seed_input, dict) else {}
    dsph = files.get("DsphConfig.xml", {}) if isinstance(files, dict) else {}
    expected_source = next(item for item in INPUT_SOURCES if item[0] == "DsphConfig.xml")
    if (
        seed_input.get("root") != str(Path(expected_source[1]).parent)
        or dsph
        != {
            "mode": "0400",
            "sha256": expected_source[2],
            "size_bytes": expected_source[3],
        }
    ):
        raise SupervisorError("C1M seed v3 DsphConfig inventory differs")
    return {
        "path": str(SEED_RECEIPT),
        "sha256": SEED_RECEIPT_SHA256,
        "size_bytes": len(raw),
        "status": SEED_RECEIPT_STATUS,
        "seed_id": SEED_ID,
        "direct_input": "DsphConfig.xml",
        "non_execution_flags_verified": True,
    }


def verify_v3_completion_provenance() -> dict[str, Any]:
    """Pin the consumed v3 lifecycle plus its non-settled QC/viz decision."""

    policy_record = verify_receipt(
        V3_POLICY,
        V3_POLICY_SHA256,
        "REVIEWED_FRESH_SINGLE_C1M_CPU_SOLVER_SMOKE_V3_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
        expected_size=V3_POLICY_SIZE,
    )
    start_record = verify_receipt(
        V3_START_RECEIPT,
        V3_START_RECEIPT_SHA256,
        "V3_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        expected_size=V3_START_RECEIPT_SIZE,
    )
    execution_record = verify_receipt(
        V3_EXECUTION_RECEIPT,
        V3_EXECUTION_RECEIPT_SHA256,
        "PASS_U3_C1M_CPU_SOLVER_V3_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING",
        expected_size=V3_EXECUTION_RECEIPT_SIZE,
    )
    lifecycle_record = verify_receipt(
        V3_LIFECYCLE_RECEIPT,
        V3_LIFECYCLE_RECEIPT_SHA256,
        "PASS_U3_C1M_CPU_SOLVER_V3_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
        expected_size=V3_LIFECYCLE_RECEIPT_SIZE,
    )
    qc_raw = read_regular_bytes(V3_QC_REPORT, limit=256 * 1024)
    if len(qc_raw) != V3_QC_REPORT_SIZE or sha256_bytes(qc_raw) != V3_QC_REPORT_SHA256:
        raise SupervisorError("frozen v3 QC report byte identity differs")
    try:
        qc = json.loads(qc_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("frozen v3 QC report is invalid") from exc
    run = qc.get("run", {}) if isinstance(qc, dict) else {}
    verdict = qc.get("verdict", {}) if isinstance(qc, dict) else {}
    if (
        run.get("status") != "C1M_ZERO_MOTION_SMOKE_PASS"
        or run.get("structural_pass") is not True
        or run.get("duration_eligible_for_settle_qc") is not False
        or run.get("tail_pass") is not False
        or run.get("numeric_settle_qc_pass") is not False
        or verdict.get("settled_state_freeze_eligible") is not False
        or verdict.get("settled_state_claim_allowed") is not False
    ):
        raise SupervisorError("frozen v3 QC non-settled verdict differs")
    manifest_record = verify_receipt(
        V3_VISUALIZATION_MANIFEST,
        V3_VISUALIZATION_MANIFEST_SHA256,
        "PASS_U3_C1M_SOLVER_SMOKE_VISUALIZED_NOT_SETTLED",
        expected_size=V3_VISUALIZATION_MANIFEST_SIZE,
    )
    return {
        "case_id": V3_CASE_ID,
        "identity_consumed": True,
        "retry_forbidden": True,
        "source_use": "provenance_only_no_v3_solver_output_reuse",
        "policy": policy_record,
        "receipts": {
            "start": start_record,
            "execution": execution_record,
            "lifecycle": lifecycle_record,
        },
        "output_qc": {
            "path": str(V3_QC_REPORT),
            "sha256": V3_QC_REPORT_SHA256,
            "size_bytes": len(qc_raw),
            "status": run["status"],
            "structural_pass": True,
            "duration_eligible_for_settle_qc": False,
            "tail_pass": False,
            "numeric_settle_qc_pass": False,
            "settled_state_freeze_eligible": False,
        },
        "accepted_visualization": manifest_record,
    }


def verify_v4_failure_provenance() -> dict[str, Any]:
    """Pin the consumed v4 failure without reading or reusing its snapshot."""

    policy_record = verify_receipt(
        V4_POLICY,
        V4_POLICY_SHA256,
        "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V4_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
        expected_size=V4_POLICY_SIZE,
    )
    start_record = verify_receipt(
        V4_START_RECEIPT,
        V4_START_RECEIPT_SHA256,
        "V4_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        expected_size=V4_START_RECEIPT_SIZE,
    )
    execution_record = verify_receipt(
        V4_EXECUTION_RECEIPT,
        V4_EXECUTION_RECEIPT_SHA256,
        "V4_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        expected_size=V4_EXECUTION_RECEIPT_SIZE,
    )
    incomplete_record = verify_receipt(
        V4_INCOMPLETE_RECEIPT,
        V4_INCOMPLETE_RECEIPT_SHA256,
        "V4_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        expected_size=V4_INCOMPLETE_RECEIPT_SIZE,
    )
    workspace_freeze: dict[str, Any] = {}
    for name, (path, digest, size) in V4_WORKSPACE_FREEZE.items():
        raw = read_regular_bytes(path, limit=256 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"frozen v4 workspace artifact differs: {name}")
        workspace_freeze[name] = {
            "path": str(path),
            "sha256": digest,
            "size_bytes": size,
        }

    start = read_json_object(V4_START_RECEIPT)
    execution = read_json_object(V4_EXECUTION_RECEIPT)
    incomplete = read_json_object(V4_INCOMPLETE_RECEIPT)
    gate = execution.get("gate")
    if (
        start.get("case_id") != V4_CASE_ID
        or execution.get("case_id") != V4_CASE_ID
        or incomplete.get("case_id") != V4_CASE_ID
        or not isinstance(gate, dict)
        or gate.get("returncode") != 2
        or gate.get("stdin_fully_written") is not True
        or gate.get("stdin_size_bytes") != 39_527_065
        or gate.get("stdout_sha256")
        != "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        or gate.get("stdout_size_bytes") != 0
        or gate.get("stderr_sha256")
        != "cef2979e52eca41b2921183cfa675add76e9a3d95c16ec4e32ed0b9fb167d460"
        or gate.get("stderr_size_bytes") != 108
        or gate.get("stderr_utf8_prefix")
        != '{"error": "outer guest process group remains after cleanup", "status": "SOLVER_CPU_SETTLE_COLD_A_V4_NO_GO"}\n'
        or execution.get("primary_error")
        != "SupervisorError: UID1000 solver gate did not return one clean successful frame"
        or execution.get("cleanup_errors") != []
        or incomplete.get("cleanup_errors") != []
        or incomplete.get("next_allowed_stage")
        != "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION"
    ):
        raise SupervisorError("consumed cold-A v4 failure evidence differs")
    for document in (execution, incomplete):
        if (
            document.get("cold_b_admission_authorized") is not False
            or document.get("settled_state_authorized") is not False
            or document.get("production_authorized") is not False
        ):
            raise SupervisorError("consumed cold-A v4 authorization ceiling differs")

    return {
        "case_id": V4_CASE_ID,
        "identity_consumed": True,
        "retry_forbidden": True,
        "output_exported": False,
        "policy": policy_record,
        "workspace_freeze": workspace_freeze,
        "receipts": {
            "start": start_record,
            "execution": execution_record,
            "lifecycle_incomplete": incomplete_record,
        },
        "observed_failure": {
            "gate_returncode": 2,
            "stdin_fully_written": True,
            "stdin_size_bytes": 39_527_065,
            "stdout_sha256": gate["stdout_sha256"],
            "stdout_size_bytes": 0,
            "stderr_sha256": gate["stderr_sha256"],
            "stderr_size_bytes": 108,
            "stderr_utf8_prefix": gate["stderr_utf8_prefix"],
            "primary_error": execution["primary_error"],
            "cleanup_errors": [],
            "next_allowed_stage": incomplete["next_allowed_stage"],
        },
        "cold_b_admission_authorized": False,
        "settled_state_authorized": False,
        "u4_authorized": False,
        "production_authorized": False,
        "source_use": "provenance_only_no_v4_snapshot_or_runtime_output_reuse",
    }


def verify_v5_failure_provenance() -> dict[str, Any]:
    """Pin the consumed v5 denial failure without reading its snapshot/output."""

    policy_record = verify_receipt(
        V5_POLICY,
        V5_POLICY_SHA256,
        "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V5_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
        expected_size=V5_POLICY_SIZE,
    )
    start_record = verify_receipt(
        V5_START_RECEIPT,
        V5_START_RECEIPT_SHA256,
        "V5_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        expected_size=V5_START_RECEIPT_SIZE,
    )
    execution_record = verify_receipt(
        V5_EXECUTION_RECEIPT,
        V5_EXECUTION_RECEIPT_SHA256,
        "V5_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        expected_size=V5_EXECUTION_RECEIPT_SIZE,
    )
    incomplete_record = verify_receipt(
        V5_INCOMPLETE_RECEIPT,
        V5_INCOMPLETE_RECEIPT_SHA256,
        "V5_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        expected_size=V5_INCOMPLETE_RECEIPT_SIZE,
    )
    workspace_freeze: dict[str, Any] = {}
    for name, (path, digest, size) in V5_WORKSPACE_FREEZE.items():
        raw = read_regular_bytes(path, limit=256 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"frozen v5 workspace artifact differs: {name}")
        workspace_freeze[name] = {
            "path": str(path),
            "sha256": digest,
            "size_bytes": size,
        }

    start = read_json_object(V5_START_RECEIPT)
    execution = read_json_object(V5_EXECUTION_RECEIPT)
    incomplete = read_json_object(V5_INCOMPLETE_RECEIPT)
    gate = execution.get("gate")
    audit = execution.get("audit")
    expected_stderr = (
        "{\"error\":\"[Errno 13] Permission denied: '/proc'; solver cleanup failure: "
        "[Errno 13] Permission denied: '/proc'\",\"status\":\"GUEST_SOLVER_V5_COLD_A_NO_GO\"}\n"
    )
    if (
        start.get("case_id") != V5_CASE_ID
        or execution.get("case_id") != V5_CASE_ID
        or incomplete.get("case_id") != V5_CASE_ID
        or start.get("campaign_id") != V5_CAMPAIGN_ID
        or execution.get("campaign_id") != V5_CAMPAIGN_ID
        or incomplete.get("campaign_id") != V5_CAMPAIGN_ID
        or any(document.get("campaign_role") != "cold_a" for document in (start, execution, incomplete))
        or start.get("policy_sha256") != V5_POLICY_SHA256
        or not isinstance(gate, dict)
        or gate.get("returncode") != 2
        or gate.get("stdin_fully_written") is not True
        or gate.get("stdin_size_bytes") != 39_536_892
        or gate.get("stdout_sha256")
        != "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        or gate.get("stdout_size_bytes") != 0
        or gate.get("stderr_sha256")
        != "aee2529d45339ce39f1ac66d9517021dd73f32c93882edf72edbca2f755d264d"
        or gate.get("stderr_size_bytes") != 153
        or gate.get("stderr_utf8_prefix") != expected_stderr
        or execution.get("primary_error")
        != "SupervisorError: AppArmor journal window is not closed with zero unexpected logged denials"
        or incomplete.get("primary_error") != execution.get("primary_error")
        or execution.get("cleanup_errors") != []
        or incomplete.get("cleanup_errors") != []
        or incomplete.get("next_allowed_stage")
        != "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION"
    ):
        raise SupervisorError("consumed cold-A v5 gate or receipt evidence differs")
    if not isinstance(audit, dict):
        raise SupervisorError("consumed cold-A v5 denial audit is absent")
    denials = audit.get("sanitized_denials")
    expected_records = [
        ("4df54d03c54691460bb1a8f8e34833a2829db73afc82063bffe6538fd7e6b087", 314),
        ("93a9109c233a8eefe1abe135c8d75286ceddbb8ec9cd5cd28b3f9aa8e88ab55b", 314),
    ]
    expected_fields = {
        "apparmor": "DENIED",
        "operation": "open",
        "class": "file",
        "profile": "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v5-20260808t203712z",
        "name": "/proc/",
        "comm": "python3.12",
        "requested_mask": "r",
        "denied_mask": "r",
        "fsuid": "1000",
        "ouid": "0",
    }
    if (
        audit.get("capture_valid") is not True
        or audit.get("capture_errors") != []
        or audit.get("matching_total") != 2
        or audit.get("unexpected_total") != 2
        or audit.get("stored_count") != 2
        or audit.get("dropped_count") != 0
        or audit.get("storage_overflow") is not False
        or audit.get("raw_stdout_sha256")
        != "e14f421aada1708dcfc92e37dcdf793a363bae4d401077bad65e392110dea94b"
        or audit.get("raw_stdout_size_bytes") != 1_289
        or not isinstance(denials, list)
        or len(denials) != 2
        or audit.get("unexpected_denials") != denials
    ):
        raise SupervisorError("consumed cold-A v5 denial audit summary differs")
    for denial, (line_digest, line_size) in zip(denials, expected_records, strict=True):
        fields = denial.get("fields") if isinstance(denial, dict) else None
        if not isinstance(fields, list):
            raise SupervisorError("consumed cold-A v5 denial fields differ")
        values = {
            field.get("key"): field.get("value")
            for field in fields
            if isinstance(field, dict) and field.get("key") != "pid"
        }
        if (
            values != expected_fields
            or denial.get("line_sha256") != line_digest
            or denial.get("line_size_bytes") != line_size
            or denial.get("line_truncated") is not False
            or denial.get("parse_status") != "PARSED_UNIQUE_KV"
            or denial.get("parse_error") is not None
        ):
            raise SupervisorError("consumed cold-A v5 exact denial record differs")
    for document in (execution, incomplete):
        if (
            document.get("cold_b_admission_authorized") is not False
            or document.get("settled_state_authorized") is not False
            or document.get("production_authorized") is not False
        ):
            raise SupervisorError("consumed cold-A v5 authorization ceiling differs")
    cleanup = incomplete.get("cleanup_observation")
    profiles_after = incomplete.get("profiles_after_observation")
    prequery_cleanup = audit.get("prequery_label_cleanup")
    expected_execution_link = {
        "creation": "O_EXCL_NOFOLLOW_DIRFD_ANCHORED",
        "mode": "0440",
        "path": str(V5_EXECUTION_RECEIPT),
        "sha256": V5_EXECUTION_RECEIPT_SHA256,
        "size_bytes": V5_EXECUTION_RECEIPT_SIZE,
    }
    if (
        incomplete.get("execution_receipt") != expected_execution_link
        or not isinstance(prequery_cleanup, dict)
        or prequery_cleanup.get("initial") != []
        or prequery_cleanup.get("after_term") != []
        or prequery_cleanup.get("term_sent") != []
        or prequery_cleanup.get("kill_sent") != []
        or prequery_cleanup.get("stable_zero_scans") != [[], [], []]
        or audit.get("postquery_stable_zero_labels") != [[], [], []]
        or start.get("initial_stable_zero_labels") != [[], [], []]
        or not isinstance(cleanup, dict)
        or cleanup.get("initial") != []
        or cleanup.get("after_term") != []
        or cleanup.get("term_sent") != []
        or cleanup.get("kill_sent") != []
        or cleanup.get("stable_zero_scans") != [[], [], []]
        or cleanup.get("post_unload_stable_zero_scans") != [[], [], []]
        or not isinstance(profiles_after, dict)
        or profiles_after.get("kernel_exact_counts")
        != {
            "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v5-20260808t203712z": 0,
            "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v5-20260808t203712z": 0,
        }
        or profiles_after.get("aa_status_exact_presence")
        != {
            "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v5-20260808t203712z": False,
            "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v5-20260808t203712z": False,
        }
        or incomplete.get("sysctls_after_observation") != start.get("sysctls_before")
    ):
        raise SupervisorError("consumed cold-A v5 cleanup or sysctl evidence differs")

    return {
        "case_id": V5_CASE_ID,
        "campaign_id": V5_CAMPAIGN_ID,
        "campaign_role": "cold_a",
        "identity_consumed": True,
        "retry_forbidden": True,
        "output_exported": False,
        "policy": policy_record,
        "workspace_freeze": workspace_freeze,
        "receipts": {
            "start": start_record,
            "execution": execution_record,
            "lifecycle_incomplete": incomplete_record,
        },
        "observed_failure": {
            "gate_returncode": 2,
            "stdin_fully_written": True,
            "stdin_size_bytes": 39_536_892,
            "stdout_sha256": gate["stdout_sha256"],
            "stdout_size_bytes": 0,
            "stderr_sha256": gate["stderr_sha256"],
            "stderr_size_bytes": 153,
            "stderr_utf8_prefix": gate["stderr_utf8_prefix"],
            "primary_error": execution["primary_error"],
            "cleanup_errors": [],
            "unexpected_logged_denials": {
                "matching_total": 2,
                "unexpected_total": 2,
                "stored_count": 2,
                "storage_overflow": False,
                "raw_stdout_sha256": audit["raw_stdout_sha256"],
                "raw_stdout_size_bytes": 1_289,
                "common_fields": expected_fields,
                "records": [
                    {"line_sha256": digest, "line_size_bytes": size}
                    for digest, size in expected_records
                ],
            },
            "required_profile_delta": "/proc/ r,",
            "cleanup_evidence": {
                "prequery_stable_zero_labels": [[], [], []],
                "postquery_stable_zero_labels": [[], [], []],
                "initial_stable_zero_labels": [[], [], []],
                "cleanup_stable_zero_labels": [[], [], []],
                "post_unload_stable_zero_labels": [[], [], []],
                "profile_exact_counts": {
                    "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v5-20260808t203712z": 0,
                    "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v5-20260808t203712z": 0,
                },
                "sysctls_unchanged": True,
                "execution_receipt_linked": True,
            },
            "next_allowed_stage": incomplete["next_allowed_stage"],
        },
        "cold_b_admission_authorized": False,
        "settled_state_authorized": False,
        "u4_authorized": False,
        "production_authorized": False,
        "source_use": "provenance_only_no_v5_snapshot_or_runtime_output_reuse",
    }


def verify_provenance_and_inputs() -> tuple[dict[str, Any], dict[str, bytes]]:
    verify_fresh_source_references(
        tuple((source, name) for name, source, _digest, _size in INPUT_SOURCES),
        context="input sources",
    )
    receipts = {
        "gencase_seed_v3": verify_seed_receipt(),
        "cpu_build": verify_receipt(BUILD_RECEIPT, BUILD_RECEIPT_SHA256, "PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT"),
        "gencase_execution": verify_receipt(
            GENCASE_EXECUTION_RECEIPT,
            GENCASE_EXECUTION_SHA256,
            "PASS_U3_C1M_GENCASE_V8_CASE_EXPORTED_CLEANUP_PENDING",
        ),
        "gencase_lifecycle": verify_receipt(
            GENCASE_LIFECYCLE_RECEIPT,
            GENCASE_LIFECYCLE_SHA256,
            "PASS_U3_C1M_GENCASE_V8_LIFECYCLE_CLEANUP_AND_CASE_EXPORT",
        ),
        "consumed_v1_start": verify_receipt(
            V1_START_RECEIPT,
            V1_START_RECEIPT_SHA256,
            "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
            expected_size=V1_START_RECEIPT_SIZE,
        ),
        "consumed_v1_execution": verify_receipt(
            V1_EXECUTION_RECEIPT,
            V1_EXECUTION_RECEIPT_SHA256,
            "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
            expected_size=V1_EXECUTION_RECEIPT_SIZE,
        ),
        "consumed_v1_lifecycle_incomplete": verify_receipt(
            V1_INCOMPLETE_RECEIPT,
            V1_INCOMPLETE_RECEIPT_SHA256,
            "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
            expected_size=V1_INCOMPLETE_RECEIPT_SIZE,
        ),
        "completed_v2_start": verify_receipt(
            V2_START_RECEIPT,
            V2_START_RECEIPT_SHA256,
            "V2_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
            expected_size=V2_START_RECEIPT_SIZE,
        ),
        "completed_v2_execution": verify_receipt(
            V2_EXECUTION_RECEIPT,
            V2_EXECUTION_RECEIPT_SHA256,
            "PASS_U3_C1_CPU_SOLVER_V2_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING",
            expected_size=V2_EXECUTION_RECEIPT_SIZE,
        ),
        "completed_v2_lifecycle": verify_receipt(
            V2_LIFECYCLE_RECEIPT,
            V2_LIFECYCLE_RECEIPT_SHA256,
            "PASS_U3_C1_CPU_SOLVER_V2_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
            expected_size=V2_LIFECYCLE_RECEIPT_SIZE,
        ),
        "completed_v3_smoke_qc_visualization": verify_v3_completion_provenance(),
        "consumed_cold_a_v4": verify_v4_failure_provenance(),
        "consumed_cold_a_v5": verify_v5_failure_provenance(),
    }
    build = read_json_object(BUILD_RECEIPT)
    artifact = build.get("artifact", {})
    if (
        artifact.get("sha256") != INPUT_SOURCES[0][2]
        or artifact.get("size_bytes") != INPUT_SOURCES[0][3]
        or artifact.get("needed") != ["libgomp.so.1", "libstdc++.so.6", "libm.so.6", "libgcc_s.so.1", "libc.so.6"]
        or artifact.get("rpath") is not False
        or artifact.get("runpath") is not False
        or artifact.get("mode_after_audit") != "0o400"
        or build.get("compiled_artifact_executed") is not False
    ):
        raise SupervisorError("CPU build receipt artifact contract differs")
    inputs: dict[str, bytes] = {}
    observed: dict[str, Any] = {}
    for name, source, digest, size in INPUT_SOURCES:
        raw = read_regular_bytes(Path(source), limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"solver source input differs: {name}")
        inputs[name] = raw
        metadata = os.stat(source, follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise SupervisorError(f"solver source input inode differs: {name}")
        observed[name] = {"source": source, "sha256": digest, "size_bytes": size}
    solver = inputs["DualSPHysics5.4CPU_linux64"]
    if solver[:4] != b"\x7fELF" or solver[4:6] != b"\x02\x01":
        raise SupervisorError("CPU solver is not ELF64 little-endian")
    if b"libcuda.so" in solver or b"libnvidia" in solver:
        raise SupervisorError("CPU solver contains a forbidden dynamic GPU dependency")
    if not inputs["C1M_zero.bi4"].startswith(b"#FileJBD JPartDataBi4"):
        raise SupervisorError("C1M input BI4 code differs")
    return {"receipts": receipts, "inputs": observed}, inputs


def verify_policy_static() -> dict[str, Any]:
    verify_fresh_source_references(
        SNAPSHOT_SOURCE_PAIRS,
        context="snapshot sources",
        forbidden_name_markers=CONSUMED_ATTEMPT_SNAPSHOT_NAME_MARKERS,
    )
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("solver policy/schema contract differs")
    if policy.get("status") != POLICY_STATUS:
        raise SupervisorError("solver policy status differs")
    trusted = policy.get("trusted_artifacts", {})
    if not isinstance(trusted, dict) or set(trusted) != set(artifact_paths()):
        raise SupervisorError("solver trusted artifact set differs")
    observed_artifacts: dict[str, Any] = {}
    for name, path in artifact_paths().items():
        raw = read_regular_bytes(path, limit=4 * 1024 * 1024)
        entry = trusted[name]
        if sha256_bytes(raw) != entry.get("sha256") or len(raw) != entry.get("size_bytes"):
            raise SupervisorError(f"solver trusted artifact differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}
    harness_raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    if sha256_bytes(harness_raw) != V11_HARNESS_SHA256 or len(harness_raw) != V11_HARNESS_SIZE:
        raise SupervisorError("frozen v11 lifecycle harness bytes differ")
    gate = load_module(GATE_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_static_gate")
    helper_module = load_module(
        HELPER_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_static_helper"
    )
    if (
        gate.CASE_ID != CASE_ID
        or gate.CAMPAIGN_ID != CAMPAIGN_ID
        or gate.ADMISSION_TOKEN != ADMISSION_TOKEN
        or helper_module.CASE_ID != CASE_ID
        or helper_module.CAMPAIGN_ID != CAMPAIGN_ID
    ):
        raise SupervisorError("v7 campaign identity differs across runtime artifacts")
    gate_review = gate.verify_review_artifacts(verify_tools=True)
    provenance, _inputs = verify_provenance_and_inputs()
    policy_raw = read_regular_bytes(POLICY_PATH)
    snapshot_sources: list[dict[str, Any]] = []
    for source, name in SNAPSHOT_SOURCE_PAIRS:
        raw = read_regular_bytes(Path(source), limit=80 * 1024 * 1024)
        snapshot_sources.append({"source": source, "name": name, "sha256": sha256_bytes(raw), "size_bytes": len(raw)})
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_bytes(policy_raw), "size_bytes": len(policy_raw)},
        "artifacts": observed_artifacts,
        "gate_review": gate_review,
        "provenance": provenance,
        "lifecycle_harness": {"path": str(HARNESS_PATH), "sha256": V11_HARNESS_SHA256, "size_bytes": V11_HARNESS_SIZE},
        "snapshot_sources": snapshot_sources,
        "denial_accounting": denial_accounting_evidence(),
        "cold_a_one_shot_static_admission_ready": True,
        "production_authorized": False,
        "execution_performed": False,
    }


def snapshot_manifest_arguments(review: Mapping[str, Any]) -> list[str]:
    sources = review.get("snapshot_sources")
    if not isinstance(sources, list) or len(sources) != len(SNAPSHOT_SOURCE_PAIRS):
        raise SupervisorError("snapshot source manifest differs")
    arguments: list[str] = []
    for expected, observed in zip(SNAPSHOT_SOURCE_PAIRS, sources, strict=True):
        if observed.get("source") != expected[0] or observed.get("name") != expected[1]:
            raise SupervisorError("snapshot source order differs")
        arguments.extend((observed["sha256"], str(observed["size_bytes"])))
    return arguments


def verify_snapshot_producer_identity() -> dict[str, Any]:
    if SCRIPT_PATH != WORKSPACE_SUPERVISOR_PATH or os.geteuid() != HOST_UID or os.getuid() != HOST_UID:
        raise SupervisorError("snapshot producer must be the exact unprivileged supervisor")
    status = Path("/proc/self/status").read_text(encoding="ascii")
    fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip() for line in status.splitlines() if ":" in line}
    uid = [int(value) for value in fields["Uid"].split()]
    gid = [int(value) for value in fields["Gid"].split()]
    caps = [int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapAmb")]
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or any(caps):
        raise SupervisorError("snapshot producer identity differs")
    return {"uid": uid, "gid": gid, "active_capabilities_zero": True}


def parse_success_frame(raw: bytes) -> dict[str, Any]:
    if not OUTPUT_HEADER.size <= len(raw) <= OUTPUT_FRAME_LIMIT:
        raise SupervisorError("solver success frame total length is out of bounds")
    magic, version, metadata_size, file_count, payload_size, console_size = OUTPUT_HEADER.unpack_from(raw)
    if magic != OUTPUT_MAGIC or version != OUTPUT_VERSION:
        raise SupervisorError("solver success frame magic/version differs")
    if not 1 <= metadata_size <= OUTPUT_METADATA_LIMIT or file_count != 171:
        raise SupervisorError("solver success frame metadata/count differs")
    if not 1 <= payload_size <= OUTPUT_TOTAL_LIMIT or not 0 <= console_size <= OUTPUT_CONSOLE_LIMIT:
        raise SupervisorError("solver success frame payload/console length differs")
    expected_total = OUTPUT_HEADER.size + metadata_size + payload_size + console_size
    if len(raw) != expected_total:
        raise SupervisorError("solver success frame is truncated or has trailing bytes")
    metadata_raw = raw[OUTPUT_HEADER.size : OUTPUT_HEADER.size + metadata_size]
    payload_raw = raw[OUTPUT_HEADER.size + metadata_size : OUTPUT_HEADER.size + metadata_size + payload_size]
    console = raw[-console_size:] if console_size else b""
    try:
        metadata = json.loads(metadata_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("solver frame metadata is invalid") from exc
    canonical = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if not isinstance(metadata, dict) or canonical != metadata_raw:
        raise SupervisorError("solver frame metadata is not a canonical object")
    required_keys = {
        "document_type",
        "status",
        "solver_argv",
        "environment",
        "guest_inputs",
        "guest_identity",
        "guest_label",
        "candidate_label",
        "guest_work_tmpfs",
        "output_audit",
        "output_manifest",
        "console",
        "stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe",
        "host_writable_bind_count",
    }
    if set(metadata) != required_keys:
        raise SupervisorError("solver frame metadata key set differs")
    if (
        metadata["document_type"] != "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_GUEST_FRAME_V7"
        or metadata["status"] != "GUEST_SOLVER_V7_COLD_A_SETTLE_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT"
        or metadata["stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe"] is not True
        or metadata["host_writable_bind_count"] != 0
    ):
        raise SupervisorError("solver frame status or transport evidence differs")
    gate = load_module(GATE_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_frame_gate")
    if metadata["solver_argv"] != gate.SOLVER_ARGV or metadata["environment"] != gate.ENVIRONMENT:
        raise SupervisorError("solver frame argv/environment differs")
    identity = metadata["guest_identity"]
    if (
        identity.get("uid") != [0, 0, 0, 0]
        or identity.get("gid") != [0, 0, 0, 0]
        or identity.get("groups") != []
        or identity.get("no_new_privs") != 1
        or set(identity.get("capabilities", {})) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        or any(identity["capabilities"].values())
    ):
        raise SupervisorError("solver frame guest identity differs")
    expected_label = BOOTSTRAP_PROFILE + " (enforce)"
    if metadata["guest_label"] != expected_label or metadata["candidate_label"] != expected_label:
        raise SupervisorError("solver frame AppArmor inheritance differs")
    tmpfs = metadata["guest_work_tmpfs"]
    if tmpfs.get("mountpoint") != "/work" or tmpfs.get("filesystem") != "tmpfs" or tmpfs.get("total_bytes") != 268_435_456:
        raise SupervisorError("solver frame tmpfs evidence differs")
    expected_inputs = {}
    helper = load_module(HELPER_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_frame_helper")
    for name, guest_path, digest, size, mode in helper.INPUTS:
        expected_inputs[name] = {"guest_path": guest_path, "sha256": digest, "size_bytes": size, "mode": f"{mode:04o}"}
    if metadata["guest_inputs"] != expected_inputs:
        raise SupervisorError("solver frame input evidence differs")
    manifest = metadata["output_manifest"]
    if not isinstance(manifest, list) or len(manifest) != 171:
        raise SupervisorError("solver frame output manifest count differs")
    paths = [entry.get("path") for entry in manifest if isinstance(entry, dict)]
    if len(paths) != 171 or tuple(paths) != EXPECTED_PATHS:
        raise SupervisorError("solver frame output path set/order differs")
    payloads: dict[str, bytes] = {}
    offset = 0
    for entry in manifest:
        path = entry["path"]
        pure = PurePosixPath(path)
        size = entry.get("size_bytes")
        digest = entry.get("sha256")
        if pure.is_absolute() or ".." in pure.parts or len(pure.parts) not in (1, 2):
            raise SupervisorError("solver frame contains an unsafe relative path")
        if not isinstance(size, int) or not 1 <= size <= 16_777_216 or not isinstance(digest, str) or len(digest) != 64:
            raise SupervisorError("solver frame manifest entry differs")
        end = offset + size
        if end > len(payload_raw):
            raise SupervisorError("solver frame payload is truncated")
        payload = payload_raw[offset:end]
        if sha256_bytes(payload) != digest:
            raise SupervisorError(f"solver frame payload digest differs: {path}")
        payloads[path] = payload
        offset = end
    if offset != len(payload_raw):
        raise SupervisorError("solver frame payload has trailing bytes")
    if metadata["console"] != {"sha256": sha256_bytes(console), "size_bytes": len(console)}:
        raise SupervisorError("solver frame console evidence differs")
    output_audit = metadata["output_audit"]
    if output_audit.get("file_count") != 171 or output_audit.get("total_bytes") != len(payload_raw):
        raise SupervisorError("solver frame output audit differs")
    run_out = payloads["Run.out"].decode("utf-8", "strict")
    required_run_out = (
        "Finished execution (code=0).",
        "[Simulation finished",
        "CaseNfixed=0",
        "CaseNmoving=2,669",
        'Shifting="NoBound"',
        "DtAllParticles=True",
    )
    if any(marker not in run_out for marker in required_run_out):
        raise SupervisorError("host re-audit rejected Run.out completion")
    motion_ref = payloads["data/PartMotionRef.ibi4"]
    if (
        len(motion_ref) < 64
        or motion_ref[:58].decode("ascii", "strict").rstrip(" ")
        != "#FileJBD JPartMotRefBi4"
    ):
        raise SupervisorError("host re-audit rejected moving-reference BI4")
    return {
        "metadata": metadata,
        "payloads": payloads,
        "console": console,
        "frame_sha256": sha256_bytes(raw),
        "frame_size_bytes": len(raw),
    }


def _assert_absent(path: Path) -> None:
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise SupervisorError(f"one-shot path already exists: {path}")


def assert_one_shot_paths_absent() -> None:
    for path in (ATTEMPT_ROOT, START_RECEIPT, EXECUTION_RECEIPT, LIFECYCLE_RECEIPT, LIFECYCLE_FAILURE_RECEIPT, CONSOLE_LOG):
        _assert_absent(path)


def open_directory_chain_nofollow(path: Path) -> int:
    if not path.is_absolute():
        raise SupervisorError("directory chain must be absolute")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for component in path.parts[1:]:
            child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def mkdir_new_at(parent_fd: int, name: str, mode: int) -> int:
    if not name or "/" in name or name in (".", ".."):
        raise SupervisorError("unsafe output directory basename")
    try:
        os.mkdir(name, mode, dir_fd=parent_fd)
    except FileExistsError as exc:
        raise SupervisorError(f"one-shot output directory exists: {name}") from exc
    descriptor = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=parent_fd)
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        os.close(descriptor)
        raise SupervisorError(f"new output directory mode differs: {name}")
    return descriptor


def verify_uid1000_nnp() -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in Path("/proc/self/status").read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    uid = [int(value) for value in fields["Uid"].split()]
    gid = [int(value) for value in fields["Gid"].split()]
    groups = [int(value) for value in fields.get("Groups", "").split()]
    caps = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}
    nnp = int(fields["NoNewPrivs"])
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or groups or any(caps.values()) or nnp != 1:
        raise SupervisorError("UID1000 exporter identity differs")
    return {"uid": uid, "gid": gid, "groups": groups, "capabilities": caps, "no_new_privs": nnp}


def consume_capability(fd_env: str, digest_env: str) -> dict[str, Any]:
    try:
        descriptor = int(os.environ[fd_env])
        expected = os.environ[digest_env]
    except (KeyError, ValueError) as exc:
        raise SupervisorError("root one-use capability is absent") from exc
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (
            descriptor < 3
            or len(expected) != 64
            or not stat.S_ISFIFO(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or stat.S_IMODE(metadata.st_mode) != 0o600
            or not os.get_inheritable(descriptor)
        ):
            raise SupervisorError("root one-use capability metadata differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise SupervisorError("root one-use capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError("root one-use capability has trailing bytes")
    finally:
        os.close(descriptor)
    if sha256_bytes(payload) != expected:
        raise SupervisorError("root one-use capability digest differs")
    os.environ.pop(fd_env, None)
    os.environ.pop(digest_env, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "sha256": expected, "size_bytes": 32}


def write_payload_new(parent_fd: int, name: str, payload: bytes, mode: int) -> dict[str, Any]:
    descriptor = os.open(
        name,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o400,
        dir_fd=parent_fd,
    )
    try:
        view = memoryview(payload)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise SupervisorError(f"short O_EXCL write: {name}")
            view = view[count:]
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise SupervisorError(f"O_EXCL output inode differs: {name}")
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
        return {
            "sha256": sha256_bytes(payload),
            "size_bytes": len(payload),
            "mode": f"{mode:04o}",
            "device": metadata.st_dev,
            "inode": metadata.st_ino,
            "nlink": metadata.st_nlink,
        }
    finally:
        os.close(descriptor)


def export_frame_o_excl(frame: bytes) -> dict[str, Any]:
    identity = verify_uid1000_nnp()
    capability = consume_capability(EXPORT_FD_ENV, EXPORT_SHA256_ENV)
    parsed = parse_success_frame(frame)
    cases_fd = open_directory_chain_nofollow(ATTEMPT_ROOT.parent)
    audits_fd = open_directory_chain_nofollow(CONSOLE_LOG.parent)
    attempt_fd = output_fd = data_fd = -1
    exported: dict[str, Any] = {}
    try:
        attempt_fd = mkdir_new_at(cases_fd, ATTEMPT_ROOT.name, 0o700)
        output_fd = mkdir_new_at(attempt_fd, OUTPUT_ROOT.name, 0o700)
        data_fd = mkdir_new_at(output_fd, "data", 0o700)
        for relative in EXPECTED_PATHS:
            payload = parsed["payloads"][relative]
            if relative.startswith("data/"):
                parent_fd, name = data_fd, relative.split("/", 1)[1]
            else:
                parent_fd, name = output_fd, relative
            record = write_payload_new(parent_fd, name, payload, 0o440)
            exported[relative] = {"path": str(OUTPUT_ROOT / relative), **record}
        if sorted(os.listdir(output_fd)) != sorted((*EXPECTED_ROOT_FILES, "data")):
            raise SupervisorError("host solver output root set differs")
        if sorted(os.listdir(data_fd)) != sorted(EXPECTED_DATA_FILES):
            raise SupervisorError("host solver data file set differs")
        console = write_payload_new(audits_fd, CONSOLE_LOG.name, parsed["console"], 0o440)
        for descriptor in (data_fd, output_fd, attempt_fd):
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o550)
            os.fsync(descriptor)
        os.fsync(audits_fd)
        os.fsync(cases_fd)
    finally:
        for descriptor in (data_fd, output_fd, attempt_fd, audits_fd, cases_fd):
            if descriptor >= 0:
                os.close(descriptor)
    return {
        "files": exported,
        "console_log": {"path": str(CONSOLE_LOG), **console},
        "frame_sha256": parsed["frame_sha256"],
        "export_identity": identity,
        "root_cleanup_capability": capability,
        "classification": "SIM_ONLY_UNVALIDATED",
    }


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> dict[str, Any]:
    if path.parent != Path("/home/zrj/scout_liquid_lab/audits") or os.geteuid() != 0:
        raise SupervisorError("receipt creation requires root and exact audit directory")
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    directory = open_directory_chain_nofollow(path.parent)
    try:
        descriptor = os.open(
            path.name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o400,
            dir_fd=directory,
        )
        try:
            view = memoryview(raw)
            while view:
                count = os.write(descriptor, view)
                if count <= 0:
                    raise SupervisorError("short append-only receipt write")
                view = view[count:]
            os.fsync(descriptor)
            os.fchown(descriptor, HOST_UID, HOST_GID)
            os.fchmod(descriptor, mode)
            os.fsync(descriptor)
            metadata = os.fstat(descriptor)
            if metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != mode:
                raise SupervisorError("append-only receipt inode differs")
        finally:
            os.close(descriptor)
        os.fsync(directory)
    finally:
        os.close(directory)
    return {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": f"{mode:04o}", "creation": "O_EXCL_NOFOLLOW_DIRFD_ANCHORED"}


def load_lifecycle_harness() -> Any:
    raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    if sha256_bytes(raw) != V11_HARNESS_SHA256 or len(raw) != V11_HARNESS_SIZE:
        raise SupervisorError("frozen v11 lifecycle harness differs")
    module = load_module(HARNESS_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_lifecycle_harness")
    module.LABELS = LABELS
    module.BOOTSTRAP_PROFILE = BOOTSTRAP_PROFILE
    module.RUNTIME_PROFILE = RUNTIME_PROFILE
    module.HOST_UID = HOST_UID
    module.HOST_GID = HOST_GID
    module.SYSCTL_PATHS = SYSCTL_PATHS
    return module


def verify_root_snapshot(policy_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if os.geteuid() != 0 or not IN_SNAPSHOT or SCRIPT_PATH != SNAPSHOT_ROOT / SUPERVISOR_NAME:
        raise SupervisorError("run requires the exact root-owned solver snapshot")
    root = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0 or stat.S_IMODE(root.st_mode) != 0o555:
        raise SupervisorError("solver snapshot root metadata differs")
    names = sorted(os.listdir(SNAPSHOT_ROOT))
    expected_names = sorted(name for _source, name in SNAPSHOT_SOURCE_PAIRS)
    if names != expected_names:
        raise SupervisorError("solver snapshot file set differs")
    policy_raw = read_regular_bytes(POLICY_PATH)
    if len(policy_sha256) != 64 or sha256_bytes(policy_raw) != policy_sha256:
        raise SupervisorError("externally frozen solver policy digest differs")
    policy = json.loads(policy_raw.decode("utf-8"))
    expected_by_name: dict[str, tuple[str, int]] = {}
    trusted = policy["trusted_artifacts"]
    for kind, path in artifact_paths().items():
        entry = trusted[kind]
        expected_by_name[path.name] = (entry["sha256"], entry["size_bytes"])
    expected_by_name[POLICY_NAME] = (policy_sha256, len(policy_raw))
    expected_by_name[HARNESS_NAME] = (V11_HARNESS_SHA256, V11_HARNESS_SIZE)
    for name, _source, digest, size in INPUT_SOURCES:
        expected_by_name[name] = (digest, size)
    observed: dict[str, Any] = {}
    for name in expected_names:
        path = SNAPSHOT_ROOT / name
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444:
            raise SupervisorError(f"solver snapshot file metadata differs: {name}")
        digest, size = expected_by_name[name]
        raw = read_regular_bytes(path, limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"solver snapshot file identity differs: {name}")
        observed[name] = {"path": str(path), "sha256": digest, "size_bytes": size, "mode": "0444"}
    return policy, {"root": str(SNAPSHOT_ROOT), "mode": "0555", "files": observed}


def read_snapshot_inputs() -> dict[str, bytes]:
    if not IN_SNAPSHOT:
        raise SupervisorError("solver inputs require exact root snapshot")
    result: dict[str, bytes] = {}
    for name, _source, digest, size in INPUT_SOURCES:
        raw = read_regular_bytes(SNAPSHOT_ROOT / name, limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"snapshot solver input differs: {name}")
        result[name] = raw
    return result


def create_root_capability() -> tuple[int, dict[str, Any]]:
    if os.geteuid() != 0:
        raise SupervisorError("root capability creation requires euid 0")
    read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
    try:
        nonce = os.getrandom(32)
        view = memoryview(nonce)
        while view:
            count = os.write(write_fd, view)
            if count <= 0:
                raise SupervisorError("short root capability write")
            view = view[count:]
    except BaseException:
        os.close(read_fd)
        raise
    finally:
        os.close(write_fd)
    return read_fd, {"transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ", "sha256": sha256_bytes(nonce), "size_bytes": 32}


def gate_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv",
        "--reuid=1000",
        "--regid=1000",
        "--clear-groups",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
        "--no-new-privs",
        "--",
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-S",
        str(GATE_PATH),
        "internal-run",
    ]


def export_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv",
        "--reuid=1000",
        "--regid=1000",
        "--clear-groups",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
        "--no-new-privs",
        "--",
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-S",
        str(SCRIPT_PATH),
        "export-frame",
    ]


def command_evidence(result: Mapping[str, Any], *, include_stderr_prefix: bool = False) -> dict[str, Any]:
    stdout = result.get("stdout", b"")
    stderr = result.get("stderr", b"")
    evidence = {
        "argv": result.get("argv"),
        "returncode": result.get("returncode"),
        "stdin_size_bytes": result.get("stdin_size_bytes"),
        "stdin_fully_written": result.get("stdin_fully_written"),
        "stdout_size_bytes": len(stdout),
        "stdout_sha256": sha256_bytes(stdout),
        "stderr_size_bytes": len(stderr),
        "stderr_sha256": sha256_bytes(stderr),
        "failure": result.get("failure"),
        "start_new_session": result.get("start_new_session"),
    }
    if include_stderr_prefix:
        evidence["stderr_utf8_prefix"] = stderr[:4096].decode("utf-8", "replace")
    return evidence


def denial_accounting_evidence() -> dict[str, Any]:
    return {
        "success_claim": "CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_WITH_TWO_EXPLICIT_QUIET_DENY_RULES",
        "known_quiet_deny_rules": list(KNOWN_QUIET_DENY_RULES),
        "known_quiet_deny_origins": dict(KNOWN_QUIET_DENY_ORIGINS),
        "visibility_boundary": DENIAL_VISIBILITY_BOUNDARY,
        "zero_denied_operations_claimed": False,
    }


def require_closed_zero_unexpected_logged_denial_audit(audit: Mapping[str, Any]) -> None:
    if (
        audit.get("capture_valid") is not True
        or audit.get("capture_errors") != []
        or audit.get("matching_total") != 0
        or audit.get("stored_count") != 0
        or audit.get("dropped_count") != 0
        or audit.get("storage_overflow") is not False
        or audit.get("unexpected_total") != 0
        or audit.get("sanitized_denials") != []
        or not audit.get("start_cursor")
        or not audit.get("end_cursor")
        or audit.get("boot_id_before") != audit.get("boot_id_after")
        or audit.get("denial_accounting") != denial_accounting_evidence()
    ):
        raise SupervisorError("AppArmor journal window is not closed with zero unexpected logged denials")


def verify_exported_outputs(parsed: Mapping[str, Any]) -> dict[str, Any]:
    attempt = os.lstat(ATTEMPT_ROOT)
    output = os.lstat(OUTPUT_ROOT)
    data = os.lstat(OUTPUT_ROOT / "data")
    for metadata, label in ((attempt, "attempt"), (output, "output"), (data, "data")):
        if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != 0o550:
            raise SupervisorError(f"exported {label} directory metadata differs")
    if sorted(os.listdir(OUTPUT_ROOT)) != sorted((*EXPECTED_ROOT_FILES, "data")) or sorted(os.listdir(OUTPUT_ROOT / "data")) != sorted(EXPECTED_DATA_FILES):
        raise SupervisorError("exported solver output tree differs")
    observed: dict[str, Any] = {}
    for relative in EXPECTED_PATHS:
        path = OUTPUT_ROOT / relative
        raw = read_regular_bytes(path, limit=16_777_216)
        metadata = os.lstat(path)
        expected = parsed["payloads"][relative]
        if raw != expected or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != 0o440:
            raise SupervisorError(f"exported solver file identity differs: {relative}")
        observed[relative] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": "0440"}
    console_raw = read_regular_bytes_allow_empty(CONSOLE_LOG, limit=OUTPUT_CONSOLE_LIMIT)
    console_metadata = os.lstat(CONSOLE_LOG)
    if (
        console_raw != parsed["console"]
        or console_metadata.st_uid != HOST_UID
        or console_metadata.st_gid != HOST_GID
        or console_metadata.st_nlink != 1
        or stat.S_IMODE(console_metadata.st_mode) != 0o440
    ):
        raise SupervisorError("exported solver console identity differs")
    return {
        "files": observed,
        "file_count": len(observed),
        "console_log": {
            "path": str(CONSOLE_LOG),
            "sha256": sha256_bytes(console_raw),
            "size_bytes": len(console_raw),
            "mode": "0440",
            "nlink": console_metadata.st_nlink,
        },
    }


def run_once(*, policy_sha256: str, admission_token: str) -> dict[str, Any]:
    if admission_token != ADMISSION_TOKEN:
        raise SupervisorError("explicit one-shot solver admission token differs")
    policy, snapshot = verify_root_snapshot(policy_sha256)
    assert_one_shot_paths_absent()
    harness = load_lifecycle_harness()
    gate = load_module(GATE_PATH, "_r8_c1m_solver_cpu_settle_cold_a_v7_runtime_gate")
    with harness.TerminationGuard() as termination_guard:
        termination_guard.checkpoint()
        sudo_admission = harness.clear_invoking_user_sudo_timestamp()
        harness.validate_sudo_cleanup_evidence(sudo_admission)
        sysctls_before = harness.read_sysctls()
        if policy.get("profile_lifecycle", {}).get("expected_sysctls") != sysctls_before:
            raise SupervisorError("current sysctls differ from frozen solver baseline")
        profiles_before = harness.profile_state()
        harness.require_profile_counts(profiles_before, 0)
        initial_zero = harness.require_stable_zero_labels()
        audit_anchor = harness.capture_journal_anchor()
        start_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_START_RECEIPT",
            "case_id": CASE_ID,
            "campaign_id": CAMPAIGN_ID,
            "campaign_role": "cold_a",
            "status": "V7_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256,
            "snapshot": snapshot,
            "sysctls_before": sysctls_before,
            "profiles_before": profiles_before,
            "initial_stable_zero_labels": initial_zero,
            "journal_anchor": audit_anchor,
            "denial_accounting": denial_accounting_evidence(),
            "sudo_timestamp_admission": sudo_admission,
        }
        start_record = write_json_new(START_RECEIPT, start_document)
        load_attempted = False
        gate_result: dict[str, Any] | None = None
        parsed: dict[str, Any] | None = None
        audit: dict[str, Any] | None = None
        export_result: dict[str, Any] | None = None
        export_verification: dict[str, Any] | None = None
        primary_error: str | None = None
        cleanup_errors: list[str] = []
        cleanup: dict[str, Any] = {}
        profiles_after: dict[str, Any] = {}
        sysctls_after: dict[str, Any] = {}
        sudo_clear: dict[str, Any] = {}
        execution_status = "V7_COLD_A_ATTEMPT_ABORTED_CLEANUP_PENDING"
        try:
            harness.run_checked(["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", str(PROFILE_PATH)], timeout_seconds=15, require_silent=True)
            termination_guard.checkpoint()
            load_attempted = True
            harness.run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", str(PROFILE_PATH)], timeout_seconds=15, require_silent=True)
            harness.require_profile_counts(harness.profile_state(), 1)
            helper_bytes = read_regular_bytes(HELPER_PATH, limit=4 * 1024 * 1024)
            frame = gate.build_input_frame(helper_bytes, read_snapshot_inputs())
            admission_fd, admission_capability = create_root_capability()
            gate_env = {
                "HOME": "/nonexistent",
                "PATH": "/usr/bin:/usr/sbin",
                "LC_ALL": "C",
                "LANG": "C",
                "TZ": "UTC0",
                SNAPSHOT_ENV: str(SNAPSHOT_ROOT),
                ADMISSION_FD_ENV: str(admission_fd),
                ADMISSION_SHA256_ENV: admission_capability["sha256"],
            }
            try:
                gate_result = harness.run_bounded_command(
                    gate_handoff_argv(),
                    stdin_bytes=frame,
                    stdout_limit=OUTPUT_FRAME_LIMIT,
                    stderr_limit=4_718_592,
                    timeout_seconds=OUTER_WALL_TIMEOUT_SECONDS,
                    env=gate_env,
                    pass_fds=(admission_fd,),
                )
            finally:
                os.close(admission_fd)
            termination_guard.checkpoint()
            prequery_cleanup = harness.terminate_labeled_processes()
            if prequery_cleanup.get("initial") != [] or prequery_cleanup.get("after_term") != [] or prequery_cleanup.get("stable_zero_scans") != [[], [], []]:
                raise SupervisorError("solver gate left AppArmor-labelled task residue")
            harness.require_profile_counts(harness.profile_state(), 1)
            harness.run_checked(["/usr/bin/journalctl", "--sync"], timeout_seconds=15, require_silent=True)
            audit = harness.capture_apparmor_denials(audit_anchor)
            audit["prequery_label_cleanup"] = prequery_cleanup
            audit["postquery_stable_zero_labels"] = harness.require_stable_zero_labels()
            audit["denial_accounting"] = denial_accounting_evidence()
            require_closed_zero_unexpected_logged_denial_audit(audit)
            if gate_result.get("returncode") != 0 or gate_result.get("failure") is not None or gate_result.get("stdin_fully_written") is not True or gate_result.get("stderr") != b"":
                raise SupervisorError("UID1000 solver gate did not return one clean successful frame")
            parsed = parse_success_frame(gate_result["stdout"])
            execution_status = "PASS_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_FRAME_VALID_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING"
        except BaseException as exc:
            primary_error = f"{type(exc).__name__}: {exc}"[:4096]
        finally:
            termination_guard.begin_cleanup()
            try:
                cleanup = harness.terminate_labeled_processes()
            except BaseException as exc:
                cleanup_errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
            pre_unload_zero = cleanup.get("stable_zero_scans") == [[], [], []]
            if not pre_unload_zero:
                try:
                    cleanup = {**cleanup, "stable_zero_scans": harness.require_stable_zero_labels(), "fallback_zero_scan_used": True}
                    pre_unload_zero = True
                except BaseException as exc:
                    cleanup_errors.append(f"pre_unload_zero_scan: {type(exc).__name__}: {exc}"[:4096])
            try:
                if not pre_unload_zero:
                    raise SupervisorError("profile unload forbidden before three zero-label scans")
                counts = harness.read_kernel_profile_counts()
                if load_attempted or any(counts.values()):
                    unload = harness.run_bounded_command(
                        ["/usr/sbin/apparmor_parser", "-R", "-K", str(PROFILE_PATH)],
                        stdout_limit=65_536,
                        stderr_limit=65_536,
                        timeout_seconds=15,
                    )
                    if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                        raise SupervisorError("transient solver profiles failed to unload")
                profiles_after = harness.profile_state()
                harness.require_profile_counts(profiles_after, 0)
                cleanup["post_unload_stable_zero_scans"] = harness.require_stable_zero_labels()
            except BaseException as exc:
                cleanup_errors.append(f"profile_cleanup: {type(exc).__name__}: {exc}"[:4096])
            try:
                sysctls_after = harness.read_sysctls()
                if sysctls_after != sysctls_before:
                    raise SupervisorError("host sysctls changed across solver lifecycle")
            except BaseException as exc:
                cleanup_errors.append(f"sysctl_postcondition: {type(exc).__name__}: {exc}"[:4096])

        if parsed is not None and primary_error is None and not cleanup_errors:
            try:
                export_fd, export_capability = create_root_capability()
                export_env = {
                    "HOME": "/nonexistent",
                    "PATH": "/usr/bin:/usr/sbin",
                    "LC_ALL": "C",
                    "LANG": "C",
                    "TZ": "UTC0",
                    EXPORT_FD_ENV: str(export_fd),
                    EXPORT_SHA256_ENV: export_capability["sha256"],
                }
                try:
                    export_command = harness.run_bounded_command(
                        export_handoff_argv(),
                        stdin_bytes=gate_result["stdout"],
                        stdout_limit=262_144,
                        stderr_limit=65_536,
                        timeout_seconds=EXPORT_TIMEOUT_SECONDS,
                        env=export_env,
                        pass_fds=(export_fd,),
                    )
                finally:
                    os.close(export_fd)
                if export_command["returncode"] != 0 or export_command["failure"] is not None or export_command["stderr"] or not export_command["stdin_fully_written"]:
                    raise SupervisorError("UID1000 solver O_EXCL exporter failed")
                message = json.loads(export_command["stdout"].decode("utf-8"))
                if message.get("status") != "PASS_UID1000_SOLVER_SETTLE_COLD_A_TREE_O_EXCL_EXPORT_V7" or not isinstance(message.get("export"), dict):
                    raise SupervisorError("solver exporter receipt differs")
                export_result = {"command": command_evidence(export_command), "capability": export_capability, "guest_report": message["export"]}
                export_verification = verify_exported_outputs(parsed)
                execution_status = "PASS_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING"
            except BaseException as exc:
                primary_error = f"{type(exc).__name__}: {exc}"[:4096]

        try:
            sudo_clear = harness.clear_invoking_user_sudo_timestamp()
            harness.validate_sudo_cleanup_evidence(sudo_clear)
        except BaseException as exc:
            cleanup_errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])

        execution_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_EXECUTION_RECEIPT",
            "case_id": CASE_ID,
            "campaign_id": CAMPAIGN_ID,
            "campaign_role": "cold_a",
            "status": execution_status if primary_error is None and not cleanup_errors else "V7_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256,
            "start_receipt": start_record,
            "gate": None if gate_result is None else command_evidence(gate_result, include_stderr_prefix=True),
            "frame": None if parsed is None else {
                "sha256": parsed["frame_sha256"],
                "size_bytes": parsed["frame_size_bytes"],
                "metadata": parsed["metadata"],
                "payloads": {path: {"sha256": sha256_bytes(raw), "size_bytes": len(raw)} for path, raw in parsed["payloads"].items()},
            },
            "audit": audit,
            "denial_accounting": denial_accounting_evidence(),
            "export": export_result,
            "export_verification": export_verification,
            "primary_error": primary_error,
            "cleanup_errors": cleanup_errors,
            "production_authorized": False,
            "classification": "SIM_ONLY_UNVALIDATED",
            "settled_state_authorized": False,
            "cold_b_admission_authorized": False,
        }
        execution_record = write_json_new(EXECUTION_RECEIPT, execution_document)
        success = execution_document["status"] == "PASS_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING" and parsed is not None and export_verification is not None
        if success:
            lifecycle_document = {
                "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_LIFECYCLE_RECEIPT",
                "case_id": CASE_ID,
                "campaign_id": CAMPAIGN_ID,
                "campaign_role": "cold_a",
                "status": "PASS_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_LIFECYCLE_CLEANUP_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
                "execution_receipt": execution_record,
                "cleanup": cleanup,
                "profiles_after": profiles_after,
                "sysctls": {"before": sysctls_before, "after": sysctls_after, "unchanged": True},
                "sudo_timestamp": sudo_clear,
                "denial_accounting": denial_accounting_evidence(),
                "next_allowed_stage": "SEPARATE_COLD_A_OUTPUT_QC_THEN_IF_PASS_FRESH_COLD_B_ADMISSION",
                "settled_state_authorized": False,
                "cold_b_admission_authorized": False,
                "production_authorized": False,
            }
            lifecycle_record = write_json_new(LIFECYCLE_RECEIPT, lifecycle_document)
            return {"execution_status": execution_document["status"], "execution_receipt": execution_record, "lifecycle_receipt": lifecycle_record}
        failure_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_V7_LIFECYCLE_INCOMPLETE_RECEIPT",
            "case_id": CASE_ID,
            "campaign_id": CAMPAIGN_ID,
            "campaign_role": "cold_a",
            "status": "V7_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
            "execution_receipt": execution_record,
            "primary_error": primary_error,
            "cleanup_errors": cleanup_errors,
            "cleanup_observation": cleanup,
            "profiles_after_observation": profiles_after,
            "sysctls_after_observation": sysctls_after,
            "sudo_timestamp_observation": sudo_clear,
            "denial_accounting": denial_accounting_evidence(),
            "production_authorized": False,
            "settled_state_authorized": False,
            "cold_b_admission_authorized": False,
            "next_allowed_stage": "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION",
        }
        failure_record = write_json_new(LIFECYCLE_FAILURE_RECEIPT, failure_document)
        raise SupervisorError(f"solver cold A v7 failed closed; preserved {failure_record['path']}")


def export_frame_entry() -> dict[str, Any]:
    if os.geteuid() != HOST_UID or not IN_SNAPSHOT:
        raise SupervisorError("export-frame requires UID1000 and exact root snapshot")
    frame = sys.stdin.buffer.read(OUTPUT_FRAME_LIMIT + 1)
    if len(frame) > OUTPUT_FRAME_LIMIT or sys.stdin.buffer.read(1) != b"":
        raise SupervisorError("solver export frame exceeds fixed ceiling")
    return export_frame_o_excl(frame)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("emit-bootstrap")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--policy-sha256", required=True)
    run_parser.add_argument("--admission-token", required=True)
    subparsers.add_parser("export-frame", help=argparse.SUPPRESS)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = verify_policy_static()
            print(json.dumps({
                "status": "PASS_SOLVER_CPU_SETTLE_COLD_A_V7_SUPERVISOR_STATIC_EXECUTION_NOT_PERFORMED",
                "review": review,
                "snapshot_bootstrap": {
                    "source_sha256": SNAPSHOT_BOOTSTRAP_SHA256,
                    "source_size_bytes": len(SNAPSHOT_BOOTSTRAP_BYTES),
                    "loader_sha256": SNAPSHOT_BOOTSTRAP_LOADER_SHA256,
                    "loader_size_bytes": len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES),
                    "loader_source": SNAPSHOT_BOOTSTRAP_LOADER_SOURCE,
                    "manifest_arguments": snapshot_manifest_arguments(review),
                },
                "admission_token": ADMISSION_TOKEN,
            }, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "emit-bootstrap":
            verify_snapshot_producer_identity()
            verify_policy_static()
            sys.stdout.buffer.write(SNAPSHOT_BOOTSTRAP_BYTES)
            sys.stdout.buffer.flush()
            return 0
        if arguments.command == "export-frame":
            result = export_frame_entry()
            print(json.dumps({"status": "PASS_UID1000_SOLVER_SETTLE_COLD_A_TREE_O_EXCL_EXPORT_V7", "export": result}, ensure_ascii=False, sort_keys=True))
            return 0
        result = run_once(policy_sha256=arguments.policy_sha256, admission_token=arguments.admission_token)
        print(json.dumps({"status": "SOLVER_CPU_SETTLE_COLD_A_V7_ONE_SHOT_LIFECYCLE_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
        return 0
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        target = sys.stderr if arguments.command in ("emit-bootstrap", "export-frame") else sys.stdout
        print(json.dumps({"status": "SOLVER_CPU_SETTLE_COLD_A_V7_NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=target)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
