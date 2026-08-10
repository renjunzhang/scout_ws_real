#!/usr/bin/env python3
"""Static gate and UID-1000 conduit for one U3 C1M cold-A settle v6.

``self-check`` is read-only and never executes an ELF.  ``internal-run`` is a
hidden one-shot entry accepted only from the exact root-owned snapshot, with a
root-created nonce FD and a fixed stdin frame.  The bwrap guest receives no host
descriptor except its bounded stdio pipes and has no host-writable bind.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import selectors
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, NamedTuple


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z"
CAMPAIGN_ID = "u3_c1m_solver_cpu_settle_ab_campaign_20260808T212044Z"
ADMISSION_TOKEN = (
    f"{CAMPAIGN_ID}:{CASE_ID}:single-cpu-solver-settle-cold-a-v6-attempt"
)
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v6-20260808t212044z"
RUNTIME_PROFILE = "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v6-20260808t212044z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V6_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V6_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_SETTLE_COLD_A_V6_ADMISSION_SHA256"

POLICY_NAME = "liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v6.json"
SCHEMA_NAME = "target_host_u3_solver_cpu_settle_cold_a_execution_policy_v6.json"
PROFILE_NAME = "r8-liquid-u3-solver-cpu-settle-cold-a-v6.profile"
GATE_NAME = "r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v6.py"
HELPER_NAME = "r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v6.py"
SUPERVISOR_NAME = "r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v6.py"
HARNESS_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"

IN_SNAPSHOT = SCRIPT_PATH.parent == SNAPSHOT_ROOT
BASE = SNAPSHOT_ROOT if IN_SNAPSHOT else PACKAGE_ROOT
POLICY_PATH = BASE / POLICY_NAME if IN_SNAPSHOT else BASE / "config/target_hosts" / POLICY_NAME
SCHEMA_PATH = BASE / SCHEMA_NAME if IN_SNAPSHOT else BASE / "schema" / SCHEMA_NAME
PROFILE_PATH = BASE / PROFILE_NAME if IN_SNAPSHOT else BASE / "config/apparmor_drafts" / PROFILE_NAME
HELPER_PATH = BASE / HELPER_NAME if IN_SNAPSHOT else BASE / "scripts" / HELPER_NAME
SUPERVISOR_PATH = BASE / SUPERVISOR_NAME if IN_SNAPSHOT else BASE / "scripts" / SUPERVISOR_NAME
HARNESS_PATH = BASE / HARNESS_NAME if IN_SNAPSHOT else BASE / "scripts" / HARNESS_NAME

HELPER_MAGIC = b"R8SOLVERHELPV6\0\0"
INPUT_MAGIC = b"R8SOLVERINPUT6\0\0"
POLICY_STATUS = "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V6_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION"
SEED_ID = "u3_c1m_gencase_seed_v3_20260808T150802Z"
SEED_RECEIPT_PROVENANCE = {
    "path": f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json",
    "sha256": "79f43b54594843e0107aa78ad01db34937fa97f0a34a16d96e74ac9cd90095ed",
    "size_bytes": 12_382,
    "status": "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION",
    "seed_id": SEED_ID,
}
CACHE_QUIET_DENY_RULE = "deny /etc/ld.so.cache r,"
CAPABILITY_QUIET_DENY_RULE = "deny capability dac_override,"
KNOWN_QUIET_DENY_RULES = (CACHE_QUIET_DENY_RULE, CAPABILITY_QUIET_DENY_RULE)
PROC_DIRECTORY_READ_RULE = "/proc/ r,"
OWNER_PROC_RULES = (
    "owner /proc/** r,",
    "owner /proc/*/uid_map w,",
    "owner /proc/*/gid_map w,",
    "owner /proc/*/setgroups w,",
)
NON_OWNER_PROC_RULES = (
    PROC_DIRECTORY_READ_RULE,
    "/proc/filesystems r,",
    "/proc/sys/kernel/overflowuid r,",
    "/proc/sys/kernel/overflowgid r,",
    "/proc/sys/user/max_user_namespaces w,",
    "/proc/stat r,",
)
PROFILE_CANONICAL_SHA256 = "9537b0615eab9fac002b9575b440fc2d8303d1399dacd9b6dba605e8cb7fed79"
KNOWN_QUIET_DENY_ORIGINS = {
    CACHE_QUIET_DENY_RULE: "carried_forward_from_v2_after_v1_observed_bwrap_dynamic_loader_probe",
    CAPABILITY_QUIET_DENY_RULE: "carried_forward_from_v2_preexisting_v1_bwrap_capability_rule",
}
DENIAL_VISIBILITY_BOUNDARY = (
    "closed_zero_unexpected_logged_DENIED_records_with_two_explicit_unlogged_deny_rules;"
    "both_carried_from_successful_v2;"
    "does_not_claim_zero_denied_operations"
)
V1_CASE_ID = "u3_c1_solver_cpu_smoke_v1_20260808T124816Z"
V1_PREDECESSOR_RECEIPTS = {
    "start": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.start.json",
        "sha256": "779feb5bdccdb2c833a4c04dd06ebf6241434b3343a3506753a9b82544758238",
        "size_bytes": 10_274,
        "status": "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
    },
    "execution": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.execution.json",
        "sha256": "6c8a2600bcb40c1039976061ec296509c4a2d6d6e89e941de9b7bfb4edf48494",
        "size_bytes": 7_614,
        "status": "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
    },
    "lifecycle_incomplete": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.lifecycle_incomplete.json",
        "sha256": "412d7e8a5c1adfc340707f6af608ab29c23f2a9446dfa6c32453a12b48caa090",
        "size_bytes": 5_037,
        "status": "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
    },
}
V2_CASE_ID = "u3_c1_solver_cpu_smoke_v2_20260808T134452Z"
V2_PREDECESSOR_RECEIPTS = {
    "start": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.start.json",
        "sha256": "6050a28d4f1093761d94a502df8635bb5964eaa161aa3f69c20e63c056799a63",
        "size_bytes": 10_966,
        "status": "V2_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
    },
    "execution": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.execution.json",
        "sha256": "bfa560359e045158a25da9c75850f4a3c40f0fb22012e410f2abab46f36c88d0",
        "size_bytes": 42_463,
        "status": "PASS_U3_C1_CPU_SOLVER_V2_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING",
    },
    "lifecycle": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.lifecycle.json",
        "sha256": "e01f81c864fa444b312dbe68b6d5490d45f53d8f99d180e3c71600fbcf6bad39",
        "size_bytes": 6_337,
        "status": "PASS_U3_C1_CPU_SOLVER_V2_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
    },
}
V3_CASE_ID = "u3_c1m_solver_cpu_smoke_v3_20260808T160108Z"
V3_COMPLETION_PROVENANCE = {
    "case_id": V3_CASE_ID,
    "identity_consumed": True,
    "retry_forbidden": True,
    "output_exported": True,
    "policy": {
        "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_execution_policy_v3.json",
        "sha256": "afef0482dabdde27c3552aa766a426a5c842c0c8d33ebb9f2c947c898d1fa6a0",
        "size_bytes": 19_495,
        "status": "REVIEWED_FRESH_SINGLE_C1M_CPU_SOLVER_SMOKE_V3_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
    },
    "receipts": {
        "start": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.start.json",
            "sha256": "8a93606fa656bbd301ec6214809ea4f315a5e9c53863fcbe12af57d57d6e95cc",
            "size_bytes": 11_050,
            "status": "V3_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        },
        "execution": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.execution.json",
            "sha256": "8ff89dd45a6548d684e75b18ef4443f63728c5604a850e81da890df7c74d916f",
            "size_bytes": 43_646,
            "status": "PASS_U3_C1M_CPU_SOLVER_V3_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING",
        },
        "lifecycle": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V3_CASE_ID}.lifecycle.json",
            "sha256": "8d4541cfbba652dc0849dfaceb6ef1be2a876a0ee36136247365decef5f2ffe9",
            "size_bytes": 6_409,
            "status": "PASS_U3_C1M_CPU_SOLVER_V3_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
        },
    },
    "output_qc": {
        "path": f"/home/zrj/scout_liquid_lab/visualizations/{V3_CASE_ID}_v2/reports/solver_output_qc_v3.json",
        "sha256": "e008fe0201802899a77e2f50bf273ef18c7cef92ca6d9a4a6d33c17adc47ee3e",
        "size_bytes": 114_699,
        "status": "C1M_ZERO_MOTION_SMOKE_PASS",
        "structural_pass": True,
        "duration_eligible_for_settle_qc": False,
        "tail_pass": False,
        "numeric_settle_qc_pass": False,
        "settled_state_freeze_eligible": False,
    },
    "accepted_visualization": {
        "root": f"/home/zrj/scout_liquid_lab/visualizations/{V3_CASE_ID}_v2",
        "artifact_manifest": {
            "path": f"/home/zrj/scout_liquid_lab/visualizations/{V3_CASE_ID}_v2/artifact_manifest.json",
            "sha256": "d096e6663352b464646b81cea138797c98d86c8b6d351b2595a0b157cb003636",
            "size_bytes": 6_720,
            "status": "PASS_U3_C1M_SOLVER_SMOKE_VISUALIZED_NOT_SETTLED",
        },
        "accepted_revision": "v2",
        "settled_state_freeze_eligible": False,
    },
    "source_use": "provenance_only_no_v3_solver_output_reuse",
}
V4_CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v4_20260808T174116Z"
V4_FAILURE_PROVENANCE = {
    "case_id": V4_CASE_ID,
    "identity_consumed": True,
    "retry_forbidden": True,
    "output_exported": False,
    "policy": {
        "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v4.json",
        "sha256": "df10e6a2f9075f4f4a11911bd341626009143af37d047ce97bb3b2eff7d4ab14",
        "size_bytes": 27_078,
        "status": "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V4_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
    },
    "workspace_freeze": {
        "schema": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_solver_cpu_settle_cold_a_execution_policy_v4.json",
            "sha256": "d5c16dc0bc3783e23bd19a2f07059f9d9e9d44a13e8ae4e485c4c533cad9754a",
            "size_bytes": 2_266,
        },
        "profile": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v4.profile",
            "sha256": "9e6b73a257ec791a6e4e06489a620d353248ccddc7cd22fcf948347e45c7896c",
            "size_bytes": 3_233,
        },
        "gate": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v4.py",
            "sha256": "c6c06e062484df1e7605826ad604ba9fd0a2fac233accfbec6941adffd0c6bb3",
            "size_bytes": 42_540,
        },
        "helper": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v4.py",
            "sha256": "8c52ee47941cf7ed59f466cb03edd174b82e4c34bdca758c085573f217b8fdeb",
            "size_bytes": 26_466,
        },
        "supervisor": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v4.py",
            "sha256": "23ad114bdfdd80141aa785144a9a392e03fe2b9f28d58968b28d4b5e41938ccc",
            "size_bytes": 74_640,
        },
        "test": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_solver_cpu_settle_cold_a_gate_v4.py",
            "sha256": "1a5bbe0d9e415d33b77d94b275642e92cd7da5d916980e444c69fc98f2e3d99b",
            "size_bytes": 34_228,
        },
    },
    "receipts": {
        "start": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.start.json",
            "sha256": "2902ec385337b3a5f386ee05d94e55be02a0b1a9710d562245bc9dc3f7c10f42",
            "size_bytes": 11_531,
            "status": "V4_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        },
        "execution": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.execution.json",
            "sha256": "710086a44417cb41646c57dcdebcca6b8625b0619bf07e1459d9ed653392979e",
            "size_bytes": 5_197,
            "status": "V4_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        },
        "lifecycle_incomplete": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V4_CASE_ID}.lifecycle_incomplete.json",
            "sha256": "4a9ff90fb7754c93ac0cb79a27bb9b2699b543cb0e638683130a8a8059fc33e7",
            "size_bytes": 6_123,
            "status": "V4_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        },
    },
    "observed_failure": {
        "gate_returncode": 2,
        "stdin_fully_written": True,
        "stdin_size_bytes": 39_527_065,
        "stdout_sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        "stdout_size_bytes": 0,
        "stderr_sha256": "cef2979e52eca41b2921183cfa675add76e9a3d95c16ec4e32ed0b9fb167d460",
        "stderr_size_bytes": 108,
        "stderr_utf8_prefix": "{\"error\": \"outer guest process group remains after cleanup\", \"status\": \"SOLVER_CPU_SETTLE_COLD_A_V4_NO_GO\"}\n",
        "primary_error": "SupervisorError: UID1000 solver gate did not return one clean successful frame",
        "cleanup_errors": [],
        "next_allowed_stage": "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION",
    },
    "cold_b_admission_authorized": False,
    "settled_state_authorized": False,
    "u4_authorized": False,
    "production_authorized": False,
    "source_use": "provenance_only_no_v4_snapshot_or_runtime_output_reuse",
}
V5_CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v5_20260808T203712Z"
V5_CAMPAIGN_ID = "u3_c1m_solver_cpu_settle_ab_campaign_20260808T203712Z"
V5_FAILURE_PROVENANCE = {
    "case_id": V5_CASE_ID,
    "campaign_id": V5_CAMPAIGN_ID,
    "campaign_role": "cold_a",
    "identity_consumed": True,
    "retry_forbidden": True,
    "output_exported": False,
    "policy": {
        "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v5.json",
        "sha256": "d558833403082e9959c145d563fdecbbb7de740cbf6d3a9d0117fdb4d9d5b866",
        "size_bytes": 32_340,
        "status": "REVIEWED_FRESH_C1M_CPU_SOLVER_SETTLE_COLD_A_V5_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION",
    },
    "workspace_freeze": {
        "schema": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_solver_cpu_settle_cold_a_execution_policy_v5.json",
            "sha256": "280a3500acb3a2cd8bee06f03cb8e508f2a8f734150305dc2201a0d913b94dca",
            "size_bytes": 2_266,
        },
        "profile": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v5.profile",
            "sha256": "70802ebedca241b5033ae84bec24ba98699489f8df77d067711b7a86fa2948d0",
            "size_bytes": 3_383,
        },
        "gate": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v5.py",
            "sha256": "a7d44ce2a037744b62b20904dcf78fc2e5ef520a09806afc46bef60a43f3e1f8",
            "size_bytes": 59_086,
        },
        "helper": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v5.py",
            "sha256": "815a019d240119c62b6d5146306ebd82f4dcd13b5ce8689f435ca4e1f3eb8f24",
            "size_bytes": 36_293,
        },
        "supervisor": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v5.py",
            "sha256": "9a8034caebd18809d37be55bd94085ac7bdff65cd5a4e556f018ff80b4a5b875",
            "size_bytes": 82_704,
        },
        "test": {
            "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_solver_cpu_settle_cold_a_gate_v5.py",
            "sha256": "970e87c13110b8221fe65188ee737c6ef547c1d8c2999ae11e3a0f6772cfb91c",
            "size_bytes": 63_284,
        },
    },
    "receipts": {
        "start": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.start.json",
            "sha256": "017774ba99487451b6e6a3dbaec58c54ee521829d0f9f486424963deefad5b90",
            "size_bytes": 11_634,
            "status": "V5_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        },
        "execution": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.execution.json",
            "sha256": "80e4cd66e8dabac2b80b18050a851a23b9cc5942ab073f1a2e3e6f093c028a0e",
            "size_bytes": 13_701,
            "status": "V5_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        },
        "lifecycle_incomplete": {
            "path": f"/home/zrj/scout_liquid_lab/audits/{V5_CASE_ID}.lifecycle_incomplete.json",
            "sha256": "d246d6a266198a253ee476da42f91344e2a106480b680140b6727e28cb1397c1",
            "size_bytes": 6_239,
            "status": "V5_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        },
    },
    "observed_failure": {
        "gate_returncode": 2,
        "stdin_fully_written": True,
        "stdin_size_bytes": 39_536_892,
        "stdout_sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        "stdout_size_bytes": 0,
        "stderr_sha256": "aee2529d45339ce39f1ac66d9517021dd73f32c93882edf72edbca2f755d264d",
        "stderr_size_bytes": 153,
        "stderr_utf8_prefix": "{\"error\":\"[Errno 13] Permission denied: '/proc'; solver cleanup failure: [Errno 13] Permission denied: '/proc'\",\"status\":\"GUEST_SOLVER_V5_COLD_A_NO_GO\"}\n",
        "primary_error": "SupervisorError: AppArmor journal window is not closed with zero unexpected logged denials",
        "cleanup_errors": [],
        "unexpected_logged_denials": {
            "matching_total": 2,
            "unexpected_total": 2,
            "stored_count": 2,
            "storage_overflow": False,
            "raw_stdout_sha256": "e14f421aada1708dcfc92e37dcdf793a363bae4d401077bad65e392110dea94b",
            "raw_stdout_size_bytes": 1_289,
            "common_fields": {
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
            },
            "records": [
                {
                    "line_sha256": "4df54d03c54691460bb1a8f8e34833a2829db73afc82063bffe6538fd7e6b087",
                    "line_size_bytes": 314,
                },
                {
                    "line_sha256": "93a9109c233a8eefe1abe135c8d75286ceddbb8ec9cd5cd28b3f9aa8e88ab55b",
                    "line_size_bytes": 314,
                },
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
        "next_allowed_stage": "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION",
    },
    "cold_b_admission_authorized": False,
    "settled_state_authorized": False,
    "u4_authorized": False,
    "production_authorized": False,
    "source_use": "provenance_only_no_v5_snapshot_or_runtime_output_reuse",
}
INPUT_CONTRACT: tuple[tuple[str, str, int], ...] = (
    ("DualSPHysics5.4CPU_linux64", "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202", 32_649_520),
    ("DsphConfig.xml", "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd", 293),
    ("C1M_zero.xml", "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb", 7_800),
    ("C1M_zero.bi4", "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7", 400_842),
    ("ld-linux-x86-64.so.2", "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81", 236_616),
    ("libgomp.so.1", "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1", 352_304),
    ("libstdc++.so.6", "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11", 2_592_224),
    ("libm.so.6", "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91", 952_616),
    ("libgcc_s.so.1", "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb", 183_024),
    ("libc.so.6", "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161", 2_125_328),
)
C1M_CASE_SEMANTICS = {
    "particle_count": 9_078,
    "fixed_boundary": 0,
    "moving_boundary": 2_669,
    "fluid": 6_409,
    "floating": 0,
    "dp_m": 0.002,
    "moving_refmotion": 0,
    "zero_motion_copies": 2,
    "zero_motion_primitive": "objreal_ref0_begin_mov1_start0_mvnull_id1",
    "shifting": 1,
    "dt_all_particles": 1,
    "expected_motion_output": "data/PartMotionRef.ibi4",
}

SOLVER_ARGV = [
    "/work/runtime/ld-linux-x86-64.so.2",
    "--inhibit-cache",
    "--library-path",
    "/work/runtime/lib",
    "/work/runtime/DualSPHysics5.4CPU_linux64",
    "/work/case/C1M_zero",
    "/work/output",
    "-cpu",
    "-ompthreads:1",
    "-stable:1",
    "-vres:0",
    "-cellmode:full",
    "-tmax:8.05",
    "-tout:0.05",
    "-sv:binx,info",
    "-svres:1",
    "-svtimers:0",
    "-svdomainvtk:0",
    "-saveposdouble:1",
    "-nortimes:1",
    "-createdirs:1",
    "-csvsep:0",
]

ENVIRONMENT = {
    "HOME": "/nonexistent",
    "PATH": "/usr/bin",
    "LC_ALL": "C",
    "LANG": "C",
    "TZ": "UTC0",
    "TMPDIR": "/work/tmp",
    "OMP_NUM_THREADS": "1",
    "OMP_DYNAMIC": "FALSE",
    "OMP_PROC_BIND": "FALSE",
}

TMPFS_BYTES = 268_435_456
INPUT_FRAME_LIMIT = 50_331_648
OUTPUT_TOTAL_LIMIT = 83_886_080
OUTPUT_METADATA_LIMIT = 262_144
OUTPUT_CONSOLE_LIMIT = 4_194_304
OUTPUT_HEADER_BYTES = 48
GUEST_STDOUT_LIMIT = OUTPUT_HEADER_BYTES + OUTPUT_METADATA_LIMIT + OUTPUT_TOTAL_LIMIT + OUTPUT_CONSOLE_LIMIT
GUEST_STDERR_LIMIT = 4_718_592
BWRAP_TIMEOUT_SECONDS = 10_770
CONDUIT_TIMEOUT_SECONDS = 10_790
EOF_EXIT_GRACE_SECONDS = 0.2
TERM_GRACE_SECONDS = 10.0
KILL_GRACE_SECONDS = 3.0
GROUP_SCAN_INTERVAL_SECONDS = 0.05

TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sudo": (Path("/usr/bin/sudo"), "136f2e48b0295b9fc595b8259cf2411ac43f27ddbfe02b956649ddaa2e92b9fa"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed static, identity, transport or command error."""


class ProcessMember(NamedTuple):
    pid: int
    state: str
    pgrp: int
    session: int
    starttime: int


class ProcessGroupIdentity(NamedTuple):
    pid: int
    pgid: int
    session: int
    starttime: int
    pidfd: int


def canonical_hash(value: Any) -> str:
    raw = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def read_regular_bytes(path: Path, *, limit: int = 80 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= limit:
            raise GateError(f"unsafe regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise GateError(f"short regular-file read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError(f"regular file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path, limit=2 * 1024 * 1024).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def sha256_file(path: Path, *, limit: int = 80 * 1024 * 1024) -> str:
    return hashlib.sha256(read_regular_bytes(path, limit=limit)).hexdigest()


def guest_loader(*, helper_size: int, helper_sha256: str) -> str:
    return (
        "import hashlib,os\n"
        "def _r(n):\n"
        " b=bytearray()\n"
        " while len(b)<n:\n"
        "  q=os.read(0,n-len(b))\n"
        "  if not q: raise SystemExit(90)\n"
        "  b.extend(q)\n"
        " return bytes(b)\n"
        f"m=_r({len(HELPER_MAGIC)})\n"
        f"h=_r({helper_size})\n"
        f"if m!={HELPER_MAGIC!r} or hashlib.sha256(h).hexdigest()!={helper_sha256!r}: raise SystemExit(91)\n"
        "g={'__name__':'__main__','__file__':'<r8-liquid-u3-c1m-solver-cpu-settle-cold-a-helper-v6>'}\n"
        "exec(compile(h,g['__file__'],'exec',dont_inherit=True,optimize=2),g,g)\n"
    )


def guest_command(*, helper_size: int, helper_sha256: str) -> list[str]:
    return [
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-S",
        "-c",
        guest_loader(helper_size=helper_size, helper_sha256=helper_sha256),
    ]


def bwrap_argv(*, helper_size: int, helper_sha256: str) -> list[str]:
    command = guest_command(helper_size=helper_size, helper_sha256=helper_sha256)
    argv = [
        "/usr/bin/timeout",
        "--foreground",
        "--signal=TERM",
        "--kill-after=10s",
        f"{BWRAP_TIMEOUT_SECONDS}s",
        "/usr/bin/aa-exec",
        "-p",
        BOOTSTRAP_PROFILE,
        "--",
        "/usr/bin/bwrap",
        "--die-with-parent",
        "--new-session",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--uid",
        "0",
        "--gid",
        "0",
        "--cap-drop",
        "ALL",
        "--hostname",
        "r8-liquid-cold-a-v6",
        "--clearenv",
    ]
    for key, value in ENVIRONMENT.items():
        argv.extend(("--setenv", key, value))
    argv.extend(
        (
            "--ro-bind",
            "/usr",
            "/usr",
            "--symlink",
            "usr/lib",
            "/lib",
            "--symlink",
            "usr/lib64",
            "/lib64",
            "--size",
            str(TMPFS_BYTES),
            "--tmpfs",
            "/work",
            "--proc",
            "/proc",
            "--chdir",
            "/work",
            "--",
            *command,
        )
    )
    verify_argv_contract(argv, expected_guest_command=command)
    return argv


def _pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == flag]


def verify_argv_contract(argv: list[str], *, expected_guest_command: list[str]) -> None:
    if _pairs(argv, "--ro-bind") != [["/usr", "/usr"]] or _pairs(argv, "--bind"):
        raise GateError("bwrap host bind surface differs")
    if _pairs(argv, "--bind-fd") or _pairs(argv, "--file") or _pairs(argv, "--dev-bind"):
        raise GateError("bwrap exposes a forbidden host descriptor or device")
    if [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == "--size"] != [[str(TMPFS_BYTES), "--tmpfs"]]:
        raise GateError("bwrap tmpfs size grammar differs")
    size_index = argv.index(str(TMPFS_BYTES))
    if argv[size_index : size_index + 3] != [str(TMPFS_BYTES), "--tmpfs", "/work"]:
        raise GateError("bwrap tmpfs ceiling is not attached to /work")
    for required in (
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--cap-drop",
    ):
        if argv.count(required) != 1:
            raise GateError(f"bwrap exact isolation option differs: {required}")
    if argv[argv.index("--cap-drop") + 1] != "ALL":
        raise GateError("bwrap child capabilities are not dropped")
    if [argv[index + 1] for index, value in enumerate(argv[:-1]) if value == "--proc"] != ["/proc"]:
        raise GateError("bwrap proc mount count differs")
    forbidden = (
        "--dev",
        "--share-net",
        "/dev",
        "/sys",
        "/home/zrj/scout_ws",
        "/home/zrj/scout_liquid_lab",
        "/dev/nvidia0",
    )
    if any(value in argv for value in forbidden):
        raise GateError("bwrap argv crosses a forbidden boundary")
    if argv[-len(expected_guest_command) :] != expected_guest_command:
        raise GateError("fixed guest helper command differs")


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def _canonical_profile(text: str) -> str:
    effective = _effective_profile(text)
    canonical = "\n".join(
        line.strip() for line in effective.splitlines() if line.strip()
    )
    return canonical.replace(BOOTSTRAP_PROFILE, "<BOOTSTRAP_PROFILE>").replace(
        RUNTIME_PROFILE, "<RUNTIME_PROFILE>"
    )


def _reject_profile_includes(text: str) -> None:
    for raw_line in text.splitlines():
        candidate = raw_line.lstrip()
        if candidate.startswith("#"):
            candidate = candidate[1:].lstrip()
        if not candidate.startswith("include"):
            continue
        suffix = candidate[len("include") :]
        if not suffix or suffix[0].isspace() or suffix[0] in '<"':
            raise GateError("solver AppArmor profile include directives are forbidden")


def verify_profile() -> dict[str, Any]:
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    text = raw.decode("utf-8", "strict")
    _reject_profile_includes(text)
    effective = _effective_profile(text)
    if hashlib.sha256(_canonical_profile(text).encode("utf-8")).hexdigest() != PROFILE_CANONICAL_SHA256:
        raise GateError("solver AppArmor profile differs from the exact v6 canonical allowlist")
    if effective.count(f"profile {BOOTSTRAP_PROFILE} ") != 1 or effective.count(f"profile {RUNTIME_PROFILE} ") != 1:
        raise GateError("fresh solver profile labels differ")
    bootstrap_body, runtime_section = effective.split(f"profile {RUNTIME_PROFILE}", 1)
    bootstrap_lines = [line.strip() for line in bootstrap_body.splitlines() if line.strip()]
    runtime_body = runtime_section.split("}", 1)[0]
    runtime_lines = [line.strip() for line in runtime_body.splitlines() if line.strip()]
    deny_lines = [line for line in bootstrap_lines if "deny " in line]
    cache_authority_lines = [line for line in bootstrap_lines if "/etc/ld.so" in line]
    capability_authority_lines = [line for line in bootstrap_lines if "capability dac_override" in line]
    owner_proc_lines = [line for line in bootstrap_lines if line.startswith("owner /proc")]
    non_owner_proc_lines = [line for line in bootstrap_lines if line.startswith("/proc")]
    if deny_lines != list(KNOWN_QUIET_DENY_RULES):
        raise GateError("bootstrap explicit deny set is not the exact two-rule contract")
    if cache_authority_lines != [CACHE_QUIET_DENY_RULE]:
        raise GateError("bootstrap ld.so cache authority is not the exact quiet deny")
    if capability_authority_lines != [CAPABILITY_QUIET_DENY_RULE]:
        raise GateError("bootstrap dac_override authority is not the exact quiet deny")
    if owner_proc_lines != list(OWNER_PROC_RULES):
        raise GateError("bootstrap owner /proc authority differs from the v5 baseline")
    if non_owner_proc_lines != list(NON_OWNER_PROC_RULES):
        raise GateError("bootstrap non-owner /proc authority exceeds the exact v6 allowlist")
    if any(
        "deny " in line or "/etc/ld.so" in line or "capability dac_override" in line
        for line in runtime_lines
    ):
        raise GateError("unreachable runtime profile contains a quiet deny or related authority")
    required = (
        "/usr/bin/bwrap rix,",
        "/usr/bin/python3.12 rix,",
        "/work/runtime/ld-linux-x86-64.so.2 rix,",
        "/work/runtime/ld-linux-x86-64.so.2 mr,",
        "/work/runtime/DualSPHysics5.4CPU_linux64 mr,",
        "/work/runtime/lib/** mr,",
        PROC_DIRECTORY_READ_RULE,
        "/proc/stat r,",
        CACHE_QUIET_DENY_RULE,
        CAPABILITY_QUIET_DENY_RULE,
        "userns create,",
        "mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/ ,".replace("/ ,", "/,"),
        "mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/ ,".replace("/ ,", "/,"),
        f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
    )
    if any(marker not in effective for marker in required):
        raise GateError("solver AppArmor profile lacks a required narrow rule")
    if "/work/runtime/DualSPHysics5.4CPU_linux64 rix," in effective:
        raise GateError("solver must be mapped only by the copied loader")
    for forbidden in (
        "/home/zrj/scout_ws",
        "/home/zrj/scout_liquid_lab",
        "/dev/nvidia",
        "change_profile",
        "flags=(unconfined)",
    ):
        if forbidden in effective:
            raise GateError(f"solver profile exposes forbidden authority: {forbidden}")
    authority_tokens = (" /work", " /usr", " userns ", " mount ", " capability ", " network ", " signal ", " ptrace ", " rix,", " mr,", " rw,")
    if any(token in runtime_body for token in authority_tokens):
        raise GateError("unreachable runtime profile is not empty")
    return {"path": str(PROFILE_PATH), "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}


def _artifact_paths() -> dict[str, Path]:
    return {
        "gate": SCRIPT_PATH,
        "helper": HELPER_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def verify_review_artifacts(*, verify_tools: bool = True) -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("solver policy/schema top-level contract differs")
    if policy.get("status") != POLICY_STATUS:
        raise GateError("solver policy status differs")
    if policy.get("allowed_gate_commands") != ["self-check", "internal-run"]:
        raise GateError("solver gate command surface differs")
    expected_artifacts = policy.get("trusted_artifacts")
    if not isinstance(expected_artifacts, dict) or set(expected_artifacts) != set(_artifact_paths()):
        raise GateError("trusted solver artifact set differs")
    observed_artifacts: dict[str, Any] = {}
    for name, path in _artifact_paths().items():
        raw = read_regular_bytes(path, limit=4 * 1024 * 1024)
        observed = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}
        entry = expected_artifacts[name]
        if observed["sha256"] != entry.get("sha256") or observed["size_bytes"] != entry.get("size_bytes"):
            raise GateError(f"trusted solver artifact differs: {name}")
        observed_artifacts[name] = observed
    profile = verify_profile()

    if (
        policy.get("provenance", {}).get("gencase_seed_v3_receipt")
        != SEED_RECEIPT_PROVENANCE
    ):
        raise GateError("C1M GenCase seed v3 provenance differs")

    predecessor = policy.get("provenance", {}).get("consumed_v1_attempt", {})
    if (
        predecessor.get("case_id") != V1_CASE_ID
        or predecessor.get("identity_consumed") is not True
        or predecessor.get("retry_forbidden") is not True
        or predecessor.get("output_exported") is not False
        or predecessor.get("receipts") != V1_PREDECESSOR_RECEIPTS
    ):
        raise GateError("consumed v1 solver predecessor provenance differs")

    completed_v2 = policy.get("provenance", {}).get("completed_c1_v2_attempt", {})
    if (
        completed_v2.get("case_id") != V2_CASE_ID
        or completed_v2.get("identity_consumed") is not True
        or completed_v2.get("retry_forbidden") is not True
        or completed_v2.get("output_exported") is not True
        or completed_v2.get("receipts") != V2_PREDECESSOR_RECEIPTS
    ):
        raise GateError("completed C1 v2 solver predecessor provenance differs")

    if policy.get("provenance", {}).get("completed_c1m_v3_smoke") != V3_COMPLETION_PROVENANCE:
        raise GateError("consumed C1M v3 smoke/QC/visualization provenance differs")

    if policy.get("provenance", {}).get("consumed_cold_a_v4_attempt") != V4_FAILURE_PROVENANCE:
        raise GateError("consumed cold-A v4 failure provenance differs")
    if policy.get("provenance", {}).get("consumed_cold_a_v5_attempt") != V5_FAILURE_PROVENANCE:
        raise GateError("consumed cold-A v5 failure provenance differs")

    frozen_attempt = policy.get("frozen_attempt", {})
    if (
        frozen_attempt.get("case_id") != CASE_ID
        or frozen_attempt.get("campaign_id") != CAMPAIGN_ID
        or frozen_attempt.get("campaign_role") != "cold_a"
        or frozen_attempt.get("cold_b_identity_allocated") is not False
        or frozen_attempt.get("supervisor_admission_token") != ADMISSION_TOKEN
        or frozen_attempt.get("attempts_per_identity") != 1
        or frozen_attempt.get("same_identity_retry") != "forbidden"
    ):
        raise GateError("cold-A v6 campaign or one-shot identity differs")

    expected_inputs = {
        name: {"sha256": digest, "size_bytes": size}
        for name, digest, size in INPUT_CONTRACT
    }
    if policy.get("input_contract", {}).get("files") != expected_inputs:
        raise GateError("solver fixed input contract differs")
    if policy.get("input_contract", {}).get("c1m_case_semantics") != C1M_CASE_SEMANTICS:
        raise GateError("solver C1M case semantics contract differs")
    command = policy.get("fixed_guest_command", {})
    if command.get("solver_argv") != SOLVER_ARGV or command.get("environment") != ENVIRONMENT:
        raise GateError("solver argv or environment differs")
    helper = expected_artifacts["helper"]
    expected_guest = guest_command(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    if command.get("guest_loader_argv_canonical_sha256") != canonical_hash(expected_guest):
        raise GateError("guest loader argv digest differs")
    isolation = policy.get("isolation", {})
    if (
        isolation.get("host_writable_bind_count") != 0
        or isolation.get("gpu_device_nodes") != []
        or isolation.get("network") != "new_empty_network_namespace"
        or isolation.get("guest_work_tmpfs_bytes") != TMPFS_BYTES
    ):
        raise GateError("solver isolation contract differs")
    transport = policy.get("immutable_transport", {})
    input_frame = transport.get("input_frame", {})
    output_frame = transport.get("output_frame", {})
    if (
        input_frame.get("helper_magic") != "R8SOLVERHELPV6\\0\\0"
        or input_frame.get("input_magic") != "R8SOLVERINPUT6\\0\\0"
        or output_frame.get("magic") != "R8SOLVEROUTV6\\0\\0\\0"
        or output_frame.get("version") != 6
        or output_frame.get("payload_total_max_bytes") != OUTPUT_TOTAL_LIMIT
        or output_frame.get("frame_total_max_bytes") != GUEST_STDOUT_LIMIT
        or transport.get("abi_baseline") != "cold_a_v5_normalized"
        or transport.get("abi_delta_from_v5") != "version_discriminator_only"
    ):
        raise GateError("cold-A v6 immutable transport contract differs")
    output = policy.get("output_contract", {})
    expected_data_files = [
        "PartInfo.ibi4",
        "PartOut_000.obi4",
        "Part_Head.ibi4",
        "PartMotionRef.ibi4",
        *(f"Part_{index:04d}.bi4" for index in range(162)),
    ]
    expected_exact_sizes = {
        "CfgInit_Domain.vtk": 1_010,
        "CfgInit_MapCells.vtk": 2_386,
        "data/PartInfo.ibi4": 115_257,
        "data/PartMotionRef.ibi4": 40_833,
        "data/PartOut_000.obi4": 584,
        "data/Part_Head.ibi4": 1_554,
        "part_series": {
            "first": "data/Part_0000.bi4",
            "last": "data/Part_0161.bi4",
            "count": 162,
            "each_size_bytes": 401_318,
        },
    }
    if (
        output.get("exact_file_count") != 171
        or output.get("exact_data_files") != expected_data_files
        or output.get("expected_exact_sizes_bytes") != expected_exact_sizes
        or output.get("text_file_limit_bytes") != 65_536
        or output.get("host_export") != "post_cleanup_uid1000_dirfd_nofollow_o_excl_only"
        or output.get("settled_state_authorized") is not False
    ):
        raise GateError("solver output/export contract differs")
    resources = policy.get("resources", {})
    expected_resources = {
        "guest_runtime_timeout_seconds": 10_710,
        "bwrap_timeout_seconds": 10_770,
        "gate_conduit_timeout_seconds": 10_790,
        "outer_wall_timeout_seconds": 10_800,
        "cpu_seconds": 10_200,
        "cpu_hard_seconds": 10_210,
        "address_space_bytes": 2_147_483_648,
        "process_limit": 16,
        "open_file_limit": 256,
        "file_size_limit_bytes": 16_777_216,
        "core_dump_bytes": 0,
        "guest_work_tmpfs_bytes": TMPFS_BYTES,
        "guest_output_total_limit_bytes": OUTPUT_TOTAL_LIMIT,
        "guest_console_limit_bytes": OUTPUT_CONSOLE_LIMIT,
        "export_timeout_seconds": 300,
    }
    if resources != expected_resources:
        raise GateError("solver resource contract differs")
    invariants = policy.get("invariants", {})
    if (
        invariants.get("cold_a_is_fresh_identity") is not True
        or invariants.get("v5_identity_consumed_and_never_retried") is not True
        or invariants.get("v5_snapshot_and_runtime_outputs_are_provenance_only") is not True
        or invariants.get("v6_profile_delta_is_exact_proc_directory_read_only") is not True
        or invariants.get("no_broad_non_owner_proc_glob_or_write_authority") is not True
        or invariants.get("v4_identity_consumed_and_never_retried") is not True
        or invariants.get("v4_snapshot_and_runtime_outputs_are_provenance_only") is not True
        or invariants.get("v3_identity_consumed_and_never_retried") is not True
        or invariants.get("v3_outputs_are_provenance_only_and_never_runtime_inputs") is not True
        or invariants.get("cold_a_failure_forbids_cold_b_policy_creation_or_execution") is not True
        or invariants.get("cold_b_admission_requires_cold_a_lifecycle_and_separate_output_qc_pass") is not True
        or invariants.get("settled_state_authorized") is not False
        or invariants.get("u4_authorized") is not False
        or policy.get("next_allowed_stage")
        != "SEPARATE_COLD_A_OUTPUT_QC_THEN_IF_PASS_FRESH_COLD_B_ADMISSION"
    ):
        raise GateError("cold-A sequencing or non-settled invariant differs")
    lifecycle = policy.get("profile_lifecycle", {})
    if (
        lifecycle.get("known_quiet_deny_rules") != list(KNOWN_QUIET_DENY_RULES)
        or lifecycle.get("known_quiet_deny_origins") != KNOWN_QUIET_DENY_ORIGINS
        or lifecycle.get("denial_visibility_boundary") != DENIAL_VISIBILITY_BOUNDARY
        or lifecycle.get("journal_success") != "closed_zero_unexpected_logged_DENIED_records_same_boot_no_overflow"
        or lifecycle.get("proc_directory_read_rule") != PROC_DIRECTORY_READ_RULE
        or lifecycle.get("proc_authority_baseline") != "cold_a_v5_profile_normalized"
        or lifecycle.get("proc_authority_delta_from_v5")
        != "one_exact_non_owner_proc_directory_read_rule_only"
    ):
        raise GateError("solver known quiet deny or visibility boundary differs")
    observed_tools: dict[str, Any] = {}
    if verify_tools:
        for name, (path, digest) in TOOLS.items():
            observed = sha256_file(path, limit=8 * 1024 * 1024)
            if observed != digest:
                raise GateError(f"trusted system tool differs: {name}")
            observed_tools[name] = {"path": str(path), "sha256": observed}
    harness_raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    harness = policy.get("lifecycle_harness", {})
    if (
        hashlib.sha256(harness_raw).hexdigest() != harness.get("sha256")
        or len(harness_raw) != harness.get("size_bytes")
    ):
        raise GateError("frozen v11 lifecycle harness differs")
    bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_file(POLICY_PATH)},
        "artifacts": observed_artifacts,
        "profile": profile,
        "tools": observed_tools,
        "lifecycle_harness": {"path": str(HARNESS_PATH), "sha256": harness["sha256"], "size_bytes": len(harness_raw)},
        "execution_performed": False,
    }


def read_identity_status(path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    return {
        "uid": [int(value) for value in fields["Uid"].split()],
        "gid": [int(value) for value in fields["Gid"].split()],
        "groups": [int(value) for value in fields.get("Groups", "").split()],
        "capabilities": {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
        "no_new_privs": int(fields["NoNewPrivs"]),
    }


def verify_child_identity(status: Mapping[str, Any] | None = None) -> dict[str, Any]:
    observed = dict(read_identity_status() if status is None else status)
    if observed.get("uid") != [HOST_UID] * 4 or observed.get("gid") != [HOST_GID] * 4 or observed.get("groups") != []:
        raise GateError("host gate UID/GID/group identity differs")
    caps = observed.get("capabilities")
    if not isinstance(caps, dict) or set(caps) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"} or any(caps.values()):
        raise GateError("host gate capabilities are not all zero")
    if observed.get("no_new_privs") != 1:
        raise GateError("host gate NoNewPrivs differs")
    return observed


def verify_snapshot_runtime() -> dict[str, Any]:
    if not IN_SNAPSHOT or SCRIPT_PATH.parent != SNAPSHOT_ROOT or os.environ.get(SNAPSHOT_ENV) != str(SNAPSHOT_ROOT):
        raise GateError("internal-run requires the exact solver snapshot")
    root = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0 or stat.S_IMODE(root.st_mode) != 0o555:
        raise GateError("solver snapshot directory metadata differs")
    for path in (SCRIPT_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH, HELPER_PATH, SUPERVISOR_PATH, HARNESS_PATH):
        metadata = os.lstat(path)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o444
        ):
            raise GateError(f"solver snapshot artifact metadata differs: {path.name}")
    return {"root": str(SNAPSHOT_ROOT), "uid": 0, "gid": 0, "mode": "0555"}


def consume_root_admission_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[ADMISSION_FD_ENV])
        expected = os.environ[ADMISSION_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise GateError("root admission FD capability is absent") from exc
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
            raise GateError("root admission pipe metadata differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise GateError("root admission capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError("root admission capability has trailing bytes")
    finally:
        os.close(descriptor)
    observed = hashlib.sha256(payload).hexdigest()
    if observed != expected:
        raise GateError("root admission capability digest differs")
    os.environ.pop(ADMISSION_FD_ENV, None)
    os.environ.pop(ADMISSION_SHA256_ENV, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "sha256": observed, "size_bytes": 32}


def build_input_frame(helper_bytes: bytes, inputs: Mapping[str, bytes]) -> bytes:
    expected_names = [name for name, _digest, _size in INPUT_CONTRACT]
    if set(inputs) != set(expected_names):
        raise GateError("solver input frame set differs")
    policy = read_json_object(POLICY_PATH)
    helper = policy["trusted_artifacts"]["helper"]
    if len(helper_bytes) != helper["size_bytes"] or hashlib.sha256(helper_bytes).hexdigest() != helper["sha256"]:
        raise GateError("solver helper identity differs")
    parts = [HELPER_MAGIC, helper_bytes, INPUT_MAGIC]
    for name, digest, size in INPUT_CONTRACT:
        raw = inputs[name]
        if len(raw) != size or hashlib.sha256(raw).hexdigest() != digest:
            raise GateError(f"solver framed input differs: {name}")
        parts.append(raw)
    frame = b"".join(parts)
    if len(frame) > INPUT_FRAME_LIMIT:
        raise GateError("solver input frame exceeds fixed ceiling")
    return frame


def _read_process_member(
    pid: int, *, skip_ungrouped_kernel_task: bool = False
) -> ProcessMember | None:
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="ascii") as source:
            raw = source.read(4_097)
    except FileNotFoundError:
        return None
    if len(raw) > 4_096 or not raw.endswith("\n"):
        raise GateError(f"process stat is malformed for pid {pid}")
    close = raw.rfind(")")
    if close < 2 or not raw[:close].startswith(f"{pid} ("):
        raise GateError(f"process stat identity is malformed for pid {pid}")
    fields = raw[close + 2 :].split()
    if len(fields) < 20 or len(fields[0]) != 1:
        raise GateError(f"process stat fields are malformed for pid {pid}")
    try:
        pgrp = int(fields[2])
        session_id = int(fields[3])
        starttime = int(fields[19])
    except ValueError as exc:
        raise GateError(f"process stat numeric fields are malformed for pid {pid}") from exc
    if pgrp < 0 or session_id < 0 or starttime < 1:
        raise GateError(f"process stat contains a non-positive identity for pid {pid}")
    if pgrp == 0 and session_id == 0 and skip_ungrouped_kernel_task:
        return None
    if pgrp == 0 or session_id == 0:
        raise GateError(f"process stat contains a non-positive identity for pid {pid}")
    return ProcessMember(pid, fields[0], pgrp, session_id, starttime)


def _scan_group_members_once(pgid: int) -> list[ProcessMember]:
    members: list[ProcessMember] = []
    with os.scandir("/proc") as entries:
        for entry in entries:
            if not entry.name.isascii() or not entry.name.isdecimal():
                continue
            member = _read_process_member(
                int(entry.name), skip_ungrouped_kernel_task=True
            )
            if member is not None and member.pgrp == pgid:
                members.append(member)
    return sorted(members)


def _group_members(pgid: int) -> list[ProcessMember]:
    """Return a conservative union of two process-group scans.

    A member seen in either scan remains present.  A state transition between
    scans is represented as ``?`` and therefore can never be mistaken for the
    one safe terminal state: the original leader as the sole zombie member.
    """

    observed: dict[tuple[int, int, int, int], ProcessMember] = {}
    for scan in (_scan_group_members_once(pgid), _scan_group_members_once(pgid)):
        for member in scan:
            key = (member.pid, member.pgrp, member.session, member.starttime)
            previous = observed.get(key)
            if previous is None:
                observed[key] = member
            elif previous.state != member.state:
                observed[key] = member._replace(state="?")
    return sorted(observed.values())


def _capture_process_group(process: subprocess.Popen[bytes]) -> ProcessGroupIdentity:
    if not hasattr(os, "pidfd_open"):
        raise GateError("pidfd_open is unavailable for outer guest ownership")
    pidfd = os.pidfd_open(process.pid, 0)
    try:
        first = _read_process_member(process.pid)
        second = _read_process_member(process.pid)
        if first is None or second is None:
            raise GateError("outer guest leader disappeared before identity freeze")
        first_identity = (first.pid, first.pgrp, first.session, first.starttime)
        second_identity = (second.pid, second.pgrp, second.session, second.starttime)
        if first_identity != second_identity:
            raise GateError("outer guest leader identity changed during freeze")
        pgid = os.getpgid(process.pid)
        session_id = os.getsid(process.pid)
        if (
            process.pid != pgid
            or process.pid != session_id
            or first.pgrp != pgid
            or first.session != session_id
        ):
            raise GateError("outer guest leader is not its frozen session and group leader")
        return ProcessGroupIdentity(process.pid, pgid, session_id, first.starttime, pidfd)
    except BaseException:
        os.close(pidfd)
        raise


def _validate_leader_anchor(identity: ProcessGroupIdentity) -> ProcessMember:
    first = _read_process_member(identity.pid)
    second = _read_process_member(identity.pid)
    if first is None or second is None:
        raise GateError("outer guest leader disappeared before explicit reap")
    expected = (identity.pid, identity.pgid, identity.session, identity.starttime)
    if (
        (first.pid, first.pgrp, first.session, first.starttime) != expected
        or (second.pid, second.pgrp, second.session, second.starttime) != expected
    ):
        raise GateError("outer guest leader birth, PGID, or SID changed")
    return first if first.state == second.state else first._replace(state="?")


def _preflight_group_scan(identity: ProcessGroupIdentity) -> list[ProcessMember]:
    """Exercise the full /proc scan before allowing a long-running guest."""

    members = _group_members(identity.pgid)
    expected = (identity.pid, identity.pgid, identity.session, identity.starttime)
    if not any(
        (member.pid, member.pgrp, member.session, member.starttime) == expected
        for member in members
    ):
        raise GateError("outer guest leader is absent from the initial process-group scan")
    return members


def _only_owned_leader_zombie(
    members: list[ProcessMember], identity: ProcessGroupIdentity
) -> bool:
    return members == [
        ProcessMember(
            identity.pid,
            "Z",
            identity.pgid,
            identity.session,
            identity.starttime,
        )
    ]


def _wait_only_owned_leader_zombie(
    identity: ProcessGroupIdentity, timeout_seconds: float
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_seconds)
    while True:
        if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(GROUP_SCAN_INTERVAL_SECONDS, remaining))


def _signal_owned_group(identity: ProcessGroupIdentity, sig: signal.Signals) -> None:
    _validate_leader_anchor(identity)
    try:
        os.killpg(identity.pgid, sig)
    except ProcessLookupError:
        if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
            return
        raise GateError("outer guest process group disappeared inconsistently during signal")


def _reap_owned_leader(
    process: subprocess.Popen[bytes], identity: ProcessGroupIdentity
) -> int:
    members = _group_members(identity.pgid)
    if not _only_owned_leader_zombie(members, identity):
        raise GateError("refusing to reap outer guest leader before all same-PGID descendants are gone")
    returncode = process.wait()
    if process.poll() != returncode:
        raise GateError("outer guest leader return code changed after explicit reap")
    residue = _group_members(identity.pgid)
    if residue:
        raise GateError(f"outer guest process group remains after leader reap: {residue!r}")
    return returncode


def _stop_group(
    process: subprocess.Popen[bytes], identity: ProcessGroupIdentity
) -> int:
    members = _group_members(identity.pgid)
    if _only_owned_leader_zombie(members, identity):
        return _reap_owned_leader(process, identity)
    _signal_owned_group(identity, signal.SIGTERM)
    if not _wait_only_owned_leader_zombie(identity, TERM_GRACE_SECONDS):
        _signal_owned_group(identity, signal.SIGKILL)
        if not _wait_only_owned_leader_zombie(identity, KILL_GRACE_SECONDS):
            raise GateError("outer guest process group remains after TERM/KILL cleanup")
    return _reap_owned_leader(process, identity)


def _finish_after_pipe_eof(
    process: subprocess.Popen[bytes],
    identity: ProcessGroupIdentity,
    conduit_deadline: float,
) -> int:
    remaining = max(0.0, conduit_deadline - time.monotonic())
    grace = min(EOF_EXIT_GRACE_SECONDS, remaining)
    if not _wait_only_owned_leader_zombie(identity, grace):
        raise GateError("outer guest leader or same-PGID descendant remained alive after clean pipe EOF grace")
    return _reap_owned_leader(process, identity)


def _raise_primary_or_cleanup(
    primary: BaseException | None, cleanup: BaseException | None
) -> None:
    if primary is not None:
        if cleanup is not None:
            if not isinstance(primary, Exception):
                primary.add_note(f"outer guest cleanup failure: {cleanup}")
                raise primary
            raise GateError(f"{primary}; outer guest cleanup failure: {cleanup}") from primary
        raise primary
    if cleanup is not None:
        raise cleanup


def run_bounded_guest(argv: list[str], input_frame: bytes) -> tuple[int, bytes, bytes]:
    process = subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C", "LANG": "C", "TZ": "UTC0"},
        close_fds=True,
        start_new_session=True,
    )
    identity: ProcessGroupIdentity | None = None
    selector: selectors.BaseSelector | None = None
    position = 0
    stdout = bytearray()
    stderr = bytearray()
    deadline = time.monotonic() + CONDUIT_TIMEOUT_SECONDS
    primary_error: BaseException | None = None
    cleanup_error: BaseException | None = None
    returncode: int | None = None
    try:
        identity = _capture_process_group(process)
        _preflight_group_scan(identity)
        if process.stdin is None or process.stdout is None or process.stderr is None:
            raise GateError("guest conduit pipes are unavailable")
        descriptors = {
            "stdin": process.stdin.fileno(),
            "stdout": process.stdout.fileno(),
            "stderr": process.stderr.fileno(),
        }
        for descriptor in descriptors.values():
            os.set_blocking(descriptor, False)
        selector = selectors.DefaultSelector()
        for name, descriptor in descriptors.items():
            if name == "stdin" and not input_frame:
                process.stdin.close()
                continue
            selector.register(
                descriptor,
                selectors.EVENT_WRITE if name == "stdin" else selectors.EVENT_READ,
                name,
            )
        while selector.get_map():
            if time.monotonic() >= deadline:
                primary_error = GateError(
                    f"solver guest conduit exceeded {CONDUIT_TIMEOUT_SECONDS} seconds"
                )
                break
            events = selector.select(timeout=0.2)
            for key, _mask in events:
                descriptor = key.fd
                channel = key.data
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, input_frame[position : position + 65_536])
                    except BrokenPipeError:
                        count = 0
                        selector.unregister(descriptor)
                        process.stdin.close()
                    if count:
                        position += count
                        if position == len(input_frame):
                            selector.unregister(descriptor)
                            process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 65_536)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    (process.stdout if channel == "stdout" else process.stderr).close()
                    continue
                target = stdout if channel == "stdout" else stderr
                ceiling = GUEST_STDOUT_LIMIT if channel == "stdout" else GUEST_STDERR_LIMIT
                room = max(0, ceiling + 1 - len(target))
                target.extend(block[:room])
                if len(block) > room or len(target) > ceiling:
                    primary_error = GateError(
                        f"guest {channel} exceeded hard byte ceiling"
                    )
                    break
            if primary_error is not None:
                break
        if primary_error is None:
            try:
                returncode = _finish_after_pipe_eof(process, identity, deadline)
            except BaseException as exc:
                primary_error = exc
        if primary_error is None and position != len(input_frame):
            primary_error = GateError("guest did not consume complete solver input frame")
    except BaseException as exc:
        primary_error = exc
    finally:
        if selector is not None:
            try:
                selector.close()
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
        if returncode is None and identity is not None:
            try:
                returncode = _stop_group(process, identity)
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
        elif returncode is None:
            try:
                # Popen returned only after start_new_session=True completed,
                # and the unreaped direct-child PID cannot be reused.  Thus
                # process.pid is still the unique PGID created by Popen even
                # when pidfd/stat capture itself failed.  Kill that initial
                # group so an early timeout/setpriv/aa-exec child cannot escape.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                returncode = process.wait(timeout=2)
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream is None:
                continue
            try:
                stream.close()
            except (BrokenPipeError, OSError, ValueError) as exc:
                cleanup_error = cleanup_error or exc
        if identity is not None:
            try:
                os.close(identity.pidfd)
            except OSError as exc:
                cleanup_error = cleanup_error or exc
    _raise_primary_or_cleanup(primary_error, cleanup_error)
    if returncode is None:
        raise GateError("outer guest return code is unavailable after cleanup")
    return returncode, bytes(stdout), bytes(stderr)


def internal_child(frame: bytes) -> tuple[int, bytes, bytes]:
    verify_snapshot_runtime()
    verify_child_identity()
    consume_root_admission_capability()
    verify_review_artifacts(verify_tools=True)
    policy = read_json_object(POLICY_PATH)
    helper = policy["trusted_artifacts"]["helper"]
    expected_size = len(HELPER_MAGIC) + helper["size_bytes"] + len(INPUT_MAGIC) + sum(size for _name, _digest, size in INPUT_CONTRACT)
    if len(frame) != expected_size:
        raise GateError("solver input frame total length differs")
    argv = bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    return run_bounded_guest(argv, frame)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("internal-run", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            review = verify_review_artifacts(verify_tools=True)
            print(json.dumps({"status": "PASS_SOLVER_CPU_SETTLE_COLD_A_V6_STATIC_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        frame = sys.stdin.buffer.read(INPUT_FRAME_LIMIT + 1)
        if len(frame) > INPUT_FRAME_LIMIT or sys.stdin.buffer.read(1) != b"":
            raise GateError("outer solver input frame exceeds fixed ceiling")
        returncode, stdout, stderr = internal_child(frame)
        if returncode != 0:
            sys.stderr.buffer.write(stderr)
            if stdout:
                sys.stderr.write("\nNO_GO: discarded nonzero-rc guest stdout bytes\n")
            sys.stderr.buffer.flush()
            return returncode
        if stderr or len(stdout) < OUTPUT_HEADER_BYTES:
            raise GateError("successful solver guest did not return one clean frame")
        sys.stdout.buffer.write(stdout)
        sys.stdout.buffer.flush()
        return 0
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        stream = sys.stderr if arguments.command == "internal-run" else sys.stdout
        print(json.dumps({"status": "SOLVER_CPU_SETTLE_COLD_A_V6_NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=stream)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
