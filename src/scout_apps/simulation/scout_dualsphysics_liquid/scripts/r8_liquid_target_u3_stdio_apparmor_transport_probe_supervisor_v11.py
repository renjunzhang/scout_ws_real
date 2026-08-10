#!/usr/bin/env python3
"""Root lifecycle supervisor for the one-shot harmless v11 successor probe.

The workspace copy supports read-only ``self-check`` only.  ``run`` refuses
unless this exact script and all peer artifacts are already in the fixed
root-owned snapshot.  Snapshot creation is performed by a separately reviewed
minimal Python ``-I -B -c`` bootstrap that copies bytes but never imports or
executes workspace code.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
import re
import selectors
import signal
import stat
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
WORKSPACE_PACKAGE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"
PROBE_ID = "u3_stdio_apparmor_transport_probe_v11_20260808T052940Z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_SNAPSHOT"
ADMISSION_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION"
ADMISSION_FD_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION_SHA256"
ADMISSION_TOKEN = f"{PROBE_ID}:single-harmless-attempt"
RECOVERY_TOKEN = f"{PROBE_ID}:cleanup-only-no-probe"
PREDECESSOR_NO_GO = {
    "probe_id": "u3_stdio_apparmor_transport_probe_v7_20260807T175042Z",
    "snapshot_root": "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v7_20260807T175042Z.snapshot",
    "status": "PRE_START_SUDO_CLEANUP_BOUNDING_SET_NO_GO",
    "no_retry_same_identity": True,
    "snapshot": {
        "created": True,
        "preserve": True,
        "directory_owner": [0, 0],
        "directory_mode": "0555",
        "file_owner": [0, 0],
        "file_mode": "0444",
        "workspace_bytes_match_snapshot": True,
    },
    "frozen_workspace_artifacts": {
        "gate": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v7.py", "sha256": "57265e792840d77ae02a79f0fc1de21e81acbb8df1cbb787af491ab3b3e40aa4", "size_bytes": 66509},
        "supervisor": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v7.py", "sha256": "c44714dc0669d93fcc1dc026add632e686adcb75eba5372326ef10265393c0f4", "size_bytes": 98694},
        "profile": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v7.profile", "sha256": "14912c90f752d027ab5fa4fe09c6c0f7d21fcb1887d4fe545fe2539705b03169", "size_bytes": 3630},
        "schema": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_stdio_apparmor_transport_probe_policy_v7.json", "sha256": "55dbc8c0b452014e8020e3f9ad1c06c5075195074ecf6d905ec63c76f7d449e5", "size_bytes": 30076},
        "policy": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v7.json", "sha256": "621585ec3b52a4f71d5e3951893fca1a07f68fc7a3655e65f5480fbe3ee50aef", "size_bytes": 23567},
        "tests": {"path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_stdio_apparmor_transport_probe_gate_v7.py", "sha256": "a7af75096d5da37faf05e547e6b971cec3ca9267e77865ea7a24c3079466c23c", "size_bytes": 84688},
    },
    "preflight_receipt": {
        "path": "/home/zrj/scout_liquid_lab/audits/u3_stdio_apparmor_transport_probe_v7_20260807T175042Z.preflight_incomplete.json",
        "sha256": "24a9605a1619005a677372929e8fa4acc2c4a5e65bbb8e3e7095fd7f5950ffa7",
        "size_bytes": 921,
        "owner": [1000, 1000],
        "mode": "0440",
        "status": "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANUP_INCOMPLETE_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED",
    },
    "other_receipts_absent": ["start", "execution", "lifecycle", "lifecycle_incomplete", "recovery"],
    "execution_boundary": {
        "start_receipt_created": False,
        "parser_invoked": False,
        "profile_load_attempted": False,
        "profile_loaded": False,
        "gate_executed": False,
        "bwrap_executed": False,
        "probe_payload_executed": False,
    },
    "v7_cleanup": {
        "attempted": True,
        "proven": False,
        "error": "SupervisorError: UID1000 sudo -K all-timestamp invalidation failed",
        "observation_empty": True,
        "frozen_argv_contract_contains": "--bounding-set=-all",
    },
    "independent_bounding_diagnostic": {
        "single_diagnostic_not_v7_retry": True,
        "persisted_receipt_created": False,
        "operator_transcribed_live_pty_only": True,
        "pty_combined_only": True,
        "stdout_stderr_separation_claimed": False,
        "prompt": {"utf8": "[sudo] password for zrj: ", "size_bytes": 25, "sha256": "60dee00584e33319a2c384208f72722145582a290af11358b630415578671361"},
        "a_with_drop_all_bounding": {
            "argv": ["/usr/bin/sudo", "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27", "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--", "/usr/bin/sudo", "-K"],
            "returncode": 1,
            "post_auth_pty_combined_utf8": "\r\nsudo: unable to change to root gid: Operation not permitted\r\nsudo: error initializing audit plugin sudoers_audit\r\n",
            "post_auth_pty_combined_size_bytes": 116,
            "post_auth_pty_combined_sha256": "eef20b8b5686fea157cd1874de2df2edd0d845a93ca7b76cf408f5ca33eb6ce1",
            "exact_error_lines": ["sudo: unable to change to root gid: Operation not permitted", "sudo: error initializing audit plugin sudoers_audit"],
            "full_pty_combined_size_bytes": 141,
            "full_pty_combined_sha256": "1baf7191649ac711ddd1690ddadfed8d7e636b3bbca0c78362a57fc312d60d38",
        },
        "b_preserve_host_bounding": {
            "argv": ["/usr/bin/sudo", "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27", "--inh-caps=-all", "--ambient-caps=-all", "--", "/usr/bin/sudo", "-K"],
            "only_difference_from_a": "removed_exact_--bounding-set=-all_token",
            "returncode": 0,
            "post_auth_pty_combined_hex": "0d0a",
            "post_auth_pty_combined_size_bytes": 2,
            "post_auth_pty_combined_sha256": "7eb70257593da06f682a3ddda54a9d260d4fc514f645237f5ca74b08f8da61a6",
            "full_pty_combined_size_bytes": 27,
            "full_pty_combined_sha256": "1d7494fa8e46a73ac403d90f2dc6bfa96c8855542b257972356fd40907961fca",
        },
        "post_b_noninteractive_true": {"argv": ["/usr/bin/sudo", "-n", "/usr/bin/true"], "returncode": 1, "pty_combined_utf8": "sudo: a password is required\r\n", "size_bytes": 30, "sha256": "a42b259ca32f8716aa70b7516558b63f261186cbbb0ddd85aa007794b4159dad"},
        "final_plain_user_cleanup": {
            "clear": {"argv": ["/usr/bin/sudo", "-K"], "returncode": 0, "pty_combined_size_bytes": 0, "pty_combined_sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
            "independent_noninteractive_true": {"argv": ["/usr/bin/sudo", "-n", "/usr/bin/true"], "returncode": 1, "pty_combined_utf8": "sudo: a password is required\r\n", "size_bytes": 30, "sha256": "a42b259ca32f8716aa70b7516558b63f261186cbbb0ddd85aa007794b4159dad"},
            "closed": True,
            "closed_scope": "independent_diagnostic_observation_only",
            "v7_lifecycle_cleanup_proven": False,
        },
    },
    "production_authorized": False,
    "identity_consumed": True,
    "preserve_snapshot": True,
    "workspace_files_preserved": True,
}
PREDECESSOR_V8 = {
    "probe_id": "u3_stdio_apparmor_transport_probe_v8_20260807T183304Z",
    "snapshot_root": "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v8_20260807T183304Z.snapshot",
    "status": "PASS_MKDIR_DENIAL_DISCOVERY_AND_COMPLETE_LIFECYCLE_NO_PRODUCTION_AUTHORITY",
    "identity_consumed": True,
    "no_retry_same_identity": True,
    "production_authorized": False,
    "workspace_artifacts": {
        "gate": {"sha256": "0f6366a5626f6c1de0c4678dc2f5f11944cbfb9a38d343262925af00c0374ca1", "size_bytes": 70337},
        "supervisor": {"sha256": "f6fc7c4d4b2eb82cc0b3a33b498eaf91ed4f74c195c7f2123522a1b7b1d666c8", "size_bytes": 103209},
        "profile": {"sha256": "a5e35f06655a760e3e8d3ae5e026c9983f9a74387176e9ca3d2b92e57e770450", "size_bytes": 3630},
        "schema": {"sha256": "34c9069e5a7198d208b1e4cad61d5c20d7f69a4c16f67d12330e3683949edaea", "size_bytes": 32699},
        "policy": {"sha256": "be99635844e66cac722dd4db23a460ac95168cfa825b92de2ce9e13b00191f1a", "size_bytes": 25912},
        "tests": {"sha256": "dd169a15d6436b4a12f0a4960b7e218fa5990891a9ef0a1a8370a485c6904b51", "size_bytes": 98839},
    },
    "snapshot": {
        "directory": {"owner": [0, 0], "mode": "0555", "nlink": 2},
        "file_owner": [0, 0],
        "file_mode": "0444",
        "file_nlink": 1,
        "artifacts": {
            "gate": {"sha256": "0f6366a5626f6c1de0c4678dc2f5f11944cbfb9a38d343262925af00c0374ca1", "size_bytes": 70337},
            "supervisor": {"sha256": "f6fc7c4d4b2eb82cc0b3a33b498eaf91ed4f74c195c7f2123522a1b7b1d666c8", "size_bytes": 103209},
            "profile": {"sha256": "a5e35f06655a760e3e8d3ae5e026c9983f9a74387176e9ca3d2b92e57e770450", "size_bytes": 3630},
            "schema": {"sha256": "34c9069e5a7198d208b1e4cad61d5c20d7f69a4c16f67d12330e3683949edaea", "size_bytes": 32699},
            "policy": {"sha256": "be99635844e66cac722dd4db23a460ac95168cfa825b92de2ce9e13b00191f1a", "size_bytes": 25912},
        },
    },
    "receipts": {
        "start": {"sha256": "fc947306b5581d0017b73bc070837339349d2a6639beb1b910858ae6bfdce1c2", "size_bytes": 6270, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "ONE_SHOT_STARTED_CLEANUP_REQUIRED"},
        "execution": {"sha256": "9937cd5312fb1a61e47974fe578884be5c55fe6edca2685fef8d92ab5cb9a1de", "size_bytes": 9576, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "PASS_FAIL_CLOSED_APPARMOR_DENIAL_DISCOVERY_CLEANUP_PENDING"},
        "lifecycle": {"sha256": "4448309a90f4ee8beb3c05cb7369b555a3f97863e4a8a70e30978e3ceda891cc", "size_bytes": 5825, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "PASS_V8_PROFILE_PROCESS_SYSCTL_SUDO_LIFECYCLE_CLEANUP"},
    },
    "discovery": {"matching_total": 1, "expected_mkdir_total": 1, "unexpected_total": 0, "storage_overflow": False, "profile": "r8-liquid-u3-stdio-transport-bootstrap-v8-20260807t183304z", "operation": "mkdir", "name": "/newroot/proc/"},
    "cleanup": {"pre_unload_zero_scans": 3, "post_unload_zero_scans": 3, "profiles_absent": True, "sysctls_unchanged": True, "sudo_timestamp_cleanup_proven": True},
}
PREDECESSOR_V9 = {
    "probe_id": "u3_stdio_apparmor_transport_probe_v9_20260808T034143Z",
    "snapshot_root": "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v9_20260808T034143Z.snapshot",
    "status": "V9_EXACT_PROC_MOUNT_DENIAL_STDERR_PATH_MISMATCH_COMPLETE_LIFECYCLE_NO_PRODUCTION_AUTHORITY",
    "identity_consumed": True,
    "no_retry_same_identity": True,
    "production_authorized": False,
    "workspace_artifacts": {
        "gate": {"sha256": "7b126b8abd555b95fbbc0ca9e56b2590c07f582eeac8b9317ad318ad9d162126", "size_bytes": 80584},
        "supervisor": {"sha256": "7e0678a0b5444c04d68c291276364a498db765b626bd5fc5737b15a5df3eb7d1", "size_bytes": 130328},
        "profile": {"sha256": "275a4627a5a7046b63c965b9bc8565b539429eaf087e537acc9bfb4a9939bf27", "size_bytes": 3789},
        "schema": {"sha256": "ba5bd4181294430a3d31168577d82626ba01420889d11344831705d7ff9792d4", "size_bytes": 39174},
        "policy": {"sha256": "aeb0c8c4b5ae32c4051a4e06e25cb5b826adbc11eaa642ce5ce37b1002afaac9", "size_bytes": 31304},
        "tests": {"sha256": "222464f124d79d990db2e5cf2395dee4750bd9f0271bed1e5a750cb12ca689c5", "size_bytes": 107282},
    },
    "snapshot": {
        "directory": {"owner": [0, 0], "mode": "0555", "nlink": 2},
        "file_owner": [0, 0],
        "file_mode": "0444",
        "file_nlink": 1,
        "artifacts": {
            "gate": {"sha256": "7b126b8abd555b95fbbc0ca9e56b2590c07f582eeac8b9317ad318ad9d162126", "size_bytes": 80584},
            "supervisor": {"sha256": "7e0678a0b5444c04d68c291276364a498db765b626bd5fc5737b15a5df3eb7d1", "size_bytes": 130328},
            "profile": {"sha256": "275a4627a5a7046b63c965b9bc8565b539429eaf087e537acc9bfb4a9939bf27", "size_bytes": 3789},
            "schema": {"sha256": "ba5bd4181294430a3d31168577d82626ba01420889d11344831705d7ff9792d4", "size_bytes": 39174},
            "policy": {"sha256": "aeb0c8c4b5ae32c4051a4e06e25cb5b826adbc11eaa642ce5ce37b1002afaac9", "size_bytes": 31304},
        },
    },
    "receipts": {
        "start": {"sha256": "8fdcfc89e8aa7274a5f1492a8fb95e22ea22738fe498e96d4e53479f6bb6b948", "size_bytes": 6826, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "ONE_SHOT_STARTED_CLEANUP_REQUIRED"},
        "execution": {"sha256": "67b7ee184ff11a922931c8c3046562f5705d84316243d74168c853e9da913a75", "size_bytes": 18172, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"},
        "lifecycle": {"sha256": "5c6a85fe6e53c03791869e414eb217a23af133d73651eff87773422a3d18e854", "size_bytes": 5827, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "PASS_V9_PROFILE_PROCESS_SYSCTL_SUDO_LIFECYCLE_CLEANUP"},
    },
    "other_receipts_absent": ["preflight_incomplete", "lifecycle_incomplete", "recovery"],
    "discovery": {
        "capture_valid": True,
        "matching_total": 1,
        "expected_mount_total": 1,
        "mkdir_total": 0,
        "unexpected_total": 0,
        "storage_overflow": False,
        "profile": "r8-liquid-u3-stdio-transport-bootstrap-v9-20260808t034143z",
        "operation": "mount",
        "class": "mount",
        "info": "failed mntpnt match",
        "error": "-13",
        "name": "/newroot/proc/",
        "comm": "bwrap",
        "fstype": "proc",
        "srcname": "proc",
        "flags": ["rw", "nosuid", "nodev", "noexec"],
        "audit_line_sha256": "8f323812be682cda9d9cba86f046591e6147fac2bd4359189e5752be815c322f",
        "gate_returncode": 1,
        "gate_stdout_size_bytes": 0,
        "gate_stderr_utf8": "bwrap: Can't mount proc on /newroot/proc: Permission denied\n",
        "gate_stderr_sha256": "7ac86c8b8a73b272d765c8a17ce43c608c2230b9d7031aed53f6c5c6bbfb7700",
        "classification_blocker": "frozen_v9_stderr_expected_/proc_but_observed_/newroot/proc",
    },
    "cleanup": {"pre_unload_zero_scans": 3, "post_unload_zero_scans": 3, "profiles_absent": True, "sysctls_unchanged": True, "sudo_timestamp_cleanup_proven": True},
}
POLICY_NAME = "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v11.json"
SCHEMA_NAME = "target_host_u3_stdio_apparmor_transport_probe_policy_v11.json"
PROFILE_NAME = "r8-liquid-u3-stdio-transport-probe-v11.profile"
GATE_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v11.py"
SUPERVISOR_NAME = SCRIPT_PATH.name
BOOTSTRAP_PROFILE = "r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z"
RUNTIME_PROFILE = "r8-liquid-u3-stdio-transport-runtime-v11-20260808t052940z"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)
HOST_UID = 1000
HOST_GID = 1000
SUDO_GROUP_GID = 27
SUDO_GROUP_PATH = Path("/etc/group")
SUDO_GROUP_RECORD = "sudo:x:27:zrj"
PREDECESSOR_V10 = {
    "probe_id": "u3_stdio_apparmor_transport_probe_v10_20260808T045130Z",
    "snapshot_root": "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v10_20260808T045130Z.snapshot",
    "status": "V10_NNP_RUNTIME_TRANSITION_AND_AUXILIARY_DENIALS_COMPLETE_LIFECYCLE_NO_PRODUCTION_AUTHORITY",
    "identity_consumed": True,
    "no_retry_same_identity": True,
    "production_authorized": False,
    "workspace_artifacts": {
        "gate": {"sha256": "009be0b30c814081216d99715e1ebdee1871e1b0b62c9124bb8a529b02078532", "size_bytes": 94655},
        "supervisor": {"sha256": "4ef6490a51efdfe227df5d00612e400853ce22772d0c1b379051e4798113a9b6", "size_bytes": 144469},
        "profile": {"sha256": "415488692f58c9d706a42a967649f7061fca3e13541a9df0ad4618f6bc1a0a62", "size_bytes": 3869},
        "schema": {"sha256": "64eeaca76c8468a280bbaa6aaafd64ac23719a357a8153de1da2d7f65ad433f8", "size_bytes": 44851},
        "policy": {"sha256": "ed4fa355e04fc7c3d4ed5d843129418868f60e3ebd14ca71c787e22669bad128", "size_bytes": 36466},
        "tests": {"sha256": "40bb191b393db45a93f6e4cdc84427e677abbb2f6047e4f625a41cea0b350166", "size_bytes": 110000},
    },
    "snapshot": {
        "directory": {"owner": [0, 0], "mode": "0555", "nlink": 2},
        "file_owner": [0, 0],
        "file_mode": "0444",
        "file_nlink": 1,
        "artifacts": {
            "gate": {"sha256": "009be0b30c814081216d99715e1ebdee1871e1b0b62c9124bb8a529b02078532", "size_bytes": 94655},
            "supervisor": {"sha256": "4ef6490a51efdfe227df5d00612e400853ce22772d0c1b379051e4798113a9b6", "size_bytes": 144469},
            "profile": {"sha256": "415488692f58c9d706a42a967649f7061fca3e13541a9df0ad4618f6bc1a0a62", "size_bytes": 3869},
            "schema": {"sha256": "64eeaca76c8468a280bbaa6aaafd64ac23719a357a8153de1da2d7f65ad433f8", "size_bytes": 44851},
            "policy": {"sha256": "ed4fa355e04fc7c3d4ed5d843129418868f60e3ebd14ca71c787e22669bad128", "size_bytes": 36466},
        },
    },
    "receipts": {
        "start": {"sha256": "ce58b05d6d701e198535313399dbde34ec9b25eda9e7abb13bd3186a350e1a86", "size_bytes": 6848, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "ONE_SHOT_STARTED_CLEANUP_REQUIRED"},
        "execution": {"sha256": "46a354c267c8c19803cce115291e81fdec2307d8e246c29e2aec02ad1af230f4", "size_bytes": 34743, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"},
        "lifecycle": {"sha256": "987c3e21b5d426544d4b5fe97f7ca32290e39d6c9aeee19961db849ffb393395", "size_bytes": 5841, "owner": [1000, 1000], "mode": "0440", "nlink": 1, "status": "PASS_V10_PROFILE_PROCESS_SYSCTL_SUDO_LIFECYCLE_CLEANUP"},
    },
    "other_receipts_absent": ["preflight_incomplete", "lifecycle_incomplete", "recovery"],
    "discovery": {
        "capture_valid": True,
        "matching_total": 5,
        "stored_count": 5,
        "dropped_count": 0,
        "storage_overflow": False,
        "expected_mount_total": 0,
        "mkdir_total": 0,
        "unexpected_total": 5,
        "raw_stdout_sha256": "d1e4a1a19d2de5ad585995dc43525696fb7eff937ce417d7228e3a830fc90f60",
        "raw_stdout_size_bytes": 2412,
        "gate_returncode": 2,
        "gate_stdout_size_bytes": 0,
        "gate_stderr_utf8": "R8_STDIO_PROBE_V10_NO_GO\n",
        "gate_stderr_sha256": "0fdf8b96e164e369038c761c6f794b0265aa6afd426937fd052eb7e6525d0939",
        "denials": [
            {"line_sha256": "b38249522a99cce59e006c91a37cbcab1d2aa451f8f5adaa14d4d90f722511ad", "line_size_bytes": 269, "operation": "capable", "class": "cap", "comm": "bwrap", "capname": "dac_override"},
            {"line_sha256": "28e547a65c4974114d82688dbbb130efb22578b3650f5685625afdc0e8c0e6fb", "line_size_bytes": 323, "operation": "open", "class": "file", "comm": "python3.12", "name": "/usr/share/zoneinfo/Etc/UTC"},
            {"line_sha256": "bf65ee62a249c5016135dfee8b1c238a3cfb296a1d4178d8f1b004074e337dc1", "line_size_bytes": 336, "operation": "open", "class": "file", "comm": "python3.12", "name": "/usr/local/lib/python3.12/dist-packages/"},
            {"line_sha256": "f0687844071237c28f274e4fcf17fb6e8aba36062d7a0426483d81891df5266f", "line_size_bytes": 336, "operation": "open", "class": "file", "comm": "python3.12", "name": "/usr/local/lib/python3.12/dist-packages/"},
            {"line_sha256": "d13104f000a18994b3da8b9c2dcacb7e45cf1b0ce3c35bc7944d815d8126dd8b", "line_size_bytes": 406, "operation": "exec", "class": "file", "comm": "python3.12", "name": "/usr/bin/sleep", "info": "no new privs", "error": "-1", "target": "r8-liquid-u3-stdio-transport-runtime-v10-20260808t045130z"},
        ],
        "classification_blocker": "five_fresh_denials_including_fatal_sleep_rpx_transition_rejected_by_no_new_privs",
    },
    "cleanup": {
        "pre_unload_zero_scans": 3,
        "post_unload_zero_scans": 3,
        "profiles_absent": True,
        "sysctls_unchanged": True,
        "sudo_timestamp_cleanup_proven": True,
    },
}

SUDO_GROUP_MEMBERSHIP_CONTRACT = {
    "path": str(SUDO_GROUP_PATH),
    "owner": [0, 0],
    "mode": "0644",
    "record": SUDO_GROUP_RECORD,
    "group_name": "sudo",
    "gid": SUDO_GROUP_GID,
    "members": ["zrj"],
}
SUDO_CLEANUP_BOUNDING_MODE = "preserve_host_only"
SUDO_CLEANUP_IDENTITY_CONTRACT = {
    "reuid": HOST_UID,
    "regid": HOST_GID,
    "supplementary_groups": [SUDO_GROUP_GID],
    "groups_mode": "EXACT_NUMERIC_GROUP_LIST_ONLY",
    "bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
    "bounding_argv_tokens": [],
    "explicit_bounding_change_forbidden": True,
    "inheritable_capabilities": "-all",
    "ambient_capabilities": "-all",
    "no_new_privs_option_present": False,
    "shell": False,
    "start_new_session": False,
    "forbidden_group_options": ["--clear-groups", "--init-groups", "--keep-groups"],
}
SUCCESS_MAGIC = b"R8LQSTDIOPASSV11\x00"
MAX_SUCCESS_FRAME_BYTES = 16_384
TMPFS_BYTES = 67_108_864
PROC_MOUNT_RULE = 'mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,'
EVIDENCE_CLASSIFICATION_CONTRACT = {
    "success_requires_closed_frame": True,
    "successful_gate_exit_is_pass": False,
    "success_pass_requires": "returncode_zero_empty_stderr_byte_exact_success_frame_and_zero_logged_denials_in_closed_boot_cursor_window_with_exact_silent_dac_override_deny",
    "audit_storage_limit": 16,
    "audit_line_limit_bytes": 4096,
    "expected_counts": {
        "matching_total": 0, "stored_count": 0, "dropped_count": 0,
        "expected_mount_total": 0, "mkdir_total": 0, "unexpected_total": 0,
        "storage_overflow": False,
    },
    "journal_boundary_required": True,
    "strict_utf8_required": True,
    "duplicate_or_escaped_critical_field_is_pass": False,
    "stderr_contract": {
        "returncode": 0,
        "stdout": "byte_exact_canonical_success_frame",
        "stderr_utf8": "",
    },
    "audit_overflow_is_pass": False,
    "gate_no_go_is_pass": False,
    "any_other_failure": "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING",
    "success_authorizes": "only_harmless_stdio_transport_confinement_evidence_and_independent_review_of_a_fresh_production_revision_no_gencase_solver_or_production",
}
AUDIT_STORAGE_CEILING = 16
AUDIT_LINE_MAX_BYTES = 4_096
AUDIT_QUERY_STDOUT_LIMIT = 131_072
AUDIT_QUERY_STDERR_LIMIT = 16_384
BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
BOOT_ID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
JOURNAL_CURSOR_RE = re.compile(r"^[A-Za-z0-9_.:;=+/-]{1,2048}$")
AUDIT_TOKEN_RE = re.compile(
    r'([A-Za-z_][A-Za-z0-9_]*)=("(?:[^"\\]|\\.)*"|[^\s"]+)(?:\s+|$)'
)
AUDIT_CRITICAL_FIELDS = frozenset(
    {
        "apparmor", "operation", "class", "info", "error", "profile", "name",
        "target", "comm", "fstype", "srcname", "source", "flags",
    }
)
AUDIT_MOUNT_ALLOWED_FIELDS = frozenset(
    {
        "apparmor", "operation", "class", "info", "error", "profile", "name",
        "pid", "comm", "fstype", "srcname", "flags",
    }
)
AUDIT_FORBIDDEN_TOKENS = (
    "/home/zrj/scout_ws", "GenCase", "DualSPHysics", "/dev/nvidia", "/opt/ros",
)
EXPECTED_INPUTS = {
    "probe_alpha.bin": {"size_bytes": 275, "sha256": "23f5a93dd2e8f837d38b55a657781d0cf61ce27bba03f45b02de2ec8103e04d7"},
    "probe_beta.bin": {"size_bytes": 520, "sha256": "740b7ba62c092ee12b02ceaba860f530207c6c17aeb58d9ec25637c3f0d91055"},
    "probe_gamma.bin": {"size_bytes": 993, "sha256": "3cf78c849414a2de89b003b64a690ee01af9c1355c384bb444fa47932df50701"},
}

LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
AUDIT_ROOT = LIQUID_ROOT / "audits"
START_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.start.json"
PREFLIGHT_FAILURE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.preflight_incomplete.json"
EXECUTION_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.execution.json"
LIFECYCLE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.lifecycle.json"
LIFECYCLE_FAILURE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.lifecycle_incomplete.json"
RECOVERY_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.recovery.json"
FROZEN_RECEIPT_PATHS = {
    "start_receipt": str(START_RECEIPT),
    "preflight_failure_receipt": str(PREFLIGHT_FAILURE_RECEIPT),
    "execution_receipt": str(EXECUTION_RECEIPT),
    "lifecycle_receipt": str(LIFECYCLE_RECEIPT),
    "lifecycle_failure_receipt": str(LIFECYCLE_FAILURE_RECEIPT),
    "recovery_receipt": str(RECOVERY_RECEIPT),
}

AUDIT_DIRECTORY_POLICY = {
    Path("/"): (0, 0, 0o755),
    Path("/home"): (0, 0, 0o755),
    Path("/home/zrj"): (HOST_UID, HOST_GID, 0o750),
    LIQUID_ROOT: (HOST_UID, HOST_GID, 0o750),
    AUDIT_ROOT: (HOST_UID, HOST_GID, 0o750),
}

REPOSITORY_PATHS = {
    "gate": PACKAGE_DIR / "scripts" / GATE_NAME,
    "supervisor": SCRIPT_PATH,
    "profile": PACKAGE_DIR / "config/apparmor_drafts" / PROFILE_NAME,
    "schema": PACKAGE_DIR / "schema" / SCHEMA_NAME,
    "policy": PACKAGE_DIR / "config/target_hosts" / POLICY_NAME,
}
SNAPSHOT_PATHS = {name: SNAPSHOT_ROOT / path.name for name, path in REPOSITORY_PATHS.items()}

SYSCTL_PATHS = (
    Path("/proc/sys/user/max_user_namespaces"),
    Path("/proc/sys/kernel/unprivileged_userns_clone"),
    Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"),
)
KERNEL_PROFILE_PATH = Path("/sys/kernel/security/apparmor/profiles")
AA_STATUS_JSON_ARGV = ("/usr/sbin/aa-status", "--json")

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "journalctl": (Path("/usr/bin/journalctl"), "c49bd25d7e7655b9a44ff867923952ed5a5e0a65e9df7a0510e239bf0558e3fa"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sleep": (Path("/usr/bin/sleep"), "a65efec857cfac0d4f43fc53affa73794ff1d1fdcb547931d4b92a98bda8e646"),
    "sudo": (Path("/usr/bin/sudo"), "136f2e48b0295b9fc595b8259cf2411ac43f27ddbfe02b956649ddaa2e92b9fa"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
    "true": (Path("/usr/bin/true"), "4b5a5694e3c0e8b1d58fc52ac6ef076e55e72c2f53195243ac86d5ff517cc2f6"),
}

SNAPSHOT_BOOTSTRAP_SOURCE = r'''import hashlib,json,os,stat,sys
ROOT="/run"
NAME="r8-liquid-u3_stdio_apparmor_transport_probe_v11_20260808T052940Z.snapshot"
SOURCES=(
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v11.py","r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v11.py"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py","r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v11.profile","r8-liquid-u3-stdio-transport-probe-v11.profile"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_stdio_apparmor_transport_probe_policy_v11.json","target_host_u3_stdio_apparmor_transport_probe_policy_v11.json"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v11.json","liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v11.json"),
)
def open_abs(path):
 parts=[p for p in path.split("/") if p]
 d=os.open("/",os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:
  for part in parts[:-1]:
   n=os.open(part,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
   os.close(d); d=n
  return os.open(parts[-1],os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
 finally:
  os.close(d)
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
manifest=[]
payloads=[]
for i,(source,name) in enumerate(SOURCES):
 digest=sys.argv[1+2*i]; size=int(sys.argv[2+2*i])
 if len(digest)!=64 or size<1 or size>2097152: raise SystemExit(74)
 raw=read_source(source,size,digest); payloads.append((name,raw,digest,size))
runfd=os.open(ROOT,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
try:
 os.mkdir(NAME,0o700,dir_fd=runfd)
 snapfd=os.open(NAME,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=runfd)
 try:
  for name,raw,digest,size in payloads:
   f=os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o400,dir_fd=snapfd)
   try:
    view=memoryview(raw)
    while view:
     n=os.write(f,view)
     if n<=0: raise SystemExit(75)
     view=view[n:]
    os.fsync(f); os.fchmod(f,0o444); os.fsync(f); m=os.fstat(f)
    if m.st_uid!=0 or m.st_gid!=0 or stat.S_IMODE(m.st_mode)!=0o444 or m.st_nlink!=1: raise SystemExit(76)
   finally: os.close(f)
   manifest.append({"name":name,"sha256":digest,"size_bytes":size})
  os.fchmod(snapfd,0o555); os.fsync(snapfd)
 finally: os.close(snapfd)
 os.fsync(runfd)
finally: os.close(runfd)
os.write(1,(json.dumps({"status":"ROOT_SNAPSHOT_V11_CREATED_NOT_EXECUTED","files":manifest},sort_keys=True,separators=(",",":"))+"\n").encode("ascii"))
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
if B.read(1)!=b"" or hashlib.sha256(p).hexdigest()!="{SNAPSHOT_BOOTSTRAP_SHA256}": raise SystemExit(81)
g={{"__name__":"__main__"}}
exec(compile(p,"<r8-liquid-root-snapshot-bootstrap-v11>","exec",dont_inherit=True,optimize=2),g,g)
'''
SNAPSHOT_BOOTSTRAP_LOADER_BYTES = SNAPSHOT_BOOTSTRAP_LOADER_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_LOADER_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_LOADER_BYTES).hexdigest()

sys.dont_write_bytecode = True


class SupervisorError(RuntimeError):
    """A fail-closed snapshot, execution, evidence, or lifecycle error."""


class SupervisorTermination(SupervisorError):
    """A catchable external termination request that must enter cleanup."""


class TerminationGuard:
    def __init__(self) -> None:
        self.received: list[int] = []
        self.cleanup_started = False
        self.preflight_sudo_cleanup_attempted = False
        self.preflight_sudo_cleanup_evidence: dict[str, Any] = {}
        self.preflight_sudo_cleanup_error: str | None = None
        self.previous: dict[signal.Signals, Any] = {}

    def _handle(self, signum: int, _frame: Any) -> None:
        self.received.append(signum)

    def checkpoint(self) -> None:
        if self.received and not self.cleanup_started:
            raise SupervisorTermination(f"external termination signal received: {self.received[-1]}")

    def __enter__(self) -> "TerminationGuard":
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM, signal.SIGQUIT, signal.SIGTSTP):
            self.previous[sig] = signal.getsignal(sig)
            signal.signal(sig, self._handle)
        return self

    def begin_cleanup(self) -> None:
        self.cleanup_started = True

    def __exit__(self, _exc_type: Any, _exc: Any, _traceback: Any) -> None:
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_regular_bytes(path: Path, *, limit: int = 4 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise SupervisorError(f"unsafe regular file: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise SupervisorError(f"regular file exceeds ceiling: {path}")
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON root is not object: {path}")
    return value


def _assert_schema_objects_closed(node: Any, path: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise SupervisorError(f"schema object is not closed: {path}")
        for key, value in node.items():
            _assert_schema_objects_closed(value, f"{path}.{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            _assert_schema_objects_closed(value, f"{path}[{index}]")


def _json_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(_json_equal(left[key], right[key]) for key in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(_json_equal(a, b) for a, b in zip(left, right))
    return left == right


def _validate_schema_instance(instance: Any, node: Mapping[str, Any], root: Mapping[str, Any], path: str = "$") -> None:
    reference = node.get("$ref")
    if reference is not None:
        if not isinstance(reference, str) or not reference.startswith("#/"):
            raise SupervisorError(f"unsupported schema reference: {path}")
        target: Any = root
        for part in reference[2:].split("/"):
            target = target[part.replace("~1", "/").replace("~0", "~")]
        if not isinstance(target, dict):
            raise SupervisorError(f"schema reference target differs: {path}")
        _validate_schema_instance(instance, target, root, path)
        return
    if "const" in node and not _json_equal(instance, node["const"]):
        raise SupervisorError(f"policy const differs: {path}")
    if "enum" in node and not any(_json_equal(instance, option) for option in node["enum"]):
        raise SupervisorError(f"policy enum differs: {path}")
    expected_type = node.get("type")
    type_ok = {
        "object": isinstance(instance, dict),
        "array": isinstance(instance, list),
        "string": isinstance(instance, str),
        "integer": isinstance(instance, int) and not isinstance(instance, bool),
        "boolean": isinstance(instance, bool),
    }.get(expected_type, True)
    if not type_ok:
        raise SupervisorError(f"policy type differs: {path}")
    if expected_type == "object":
        properties = node.get("properties", {})
        required = node.get("required", [])
        if not isinstance(properties, dict) or not isinstance(required, list) or any(key not in instance for key in required):
            raise SupervisorError(f"policy required object key differs: {path}")
        if node.get("additionalProperties") is False and any(key not in properties for key in instance):
            raise SupervisorError(f"policy object has an additional key: {path}")
        for key, value in instance.items():
            child = properties.get(key)
            if isinstance(child, dict):
                _validate_schema_instance(value, child, root, f"{path}.{key}")
    elif expected_type == "array":
        if len(instance) < node.get("minItems", 0) or len(instance) > node.get("maxItems", 1 << 60):
            raise SupervisorError(f"policy array length differs: {path}")
        child = node.get("items")
        if isinstance(child, dict):
            for index, value in enumerate(instance):
                _validate_schema_instance(value, child, root, f"{path}[{index}]")
    elif expected_type == "string":
        if len(instance) < node.get("minLength", 0) or len(instance) > node.get("maxLength", 1 << 60):
            raise SupervisorError(f"policy string length differs: {path}")
        pattern = node.get("pattern")
        if pattern is not None and re.search(pattern, instance) is None:
            raise SupervisorError(f"policy string pattern differs: {path}")
    elif expected_type == "integer":
        if instance < node.get("minimum", -(1 << 63)) or instance > node.get("maximum", 1 << 63):
            raise SupervisorError(f"policy integer range differs: {path}")


def verify_predecessor_v8_provenance(*, runtime_root: bool = False) -> dict[str, Any]:
    if runtime_root and os.geteuid() != 0:
        raise SupervisorError("runtime predecessor verification requires euid 0")
    package = WORKSPACE_PACKAGE_ROOT
    workspace_paths = {
        "gate": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v8.py",
        "supervisor": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py",
        "profile": package / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v8.profile",
        "schema": package / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v8.json",
        "policy": package / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v8.json",
        "tests": package / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v8.py",
    }
    for name, path in workspace_paths.items():
        raw = read_regular_bytes(path)
        expected = PREDECESSOR_V8["workspace_artifacts"][name]
        if len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]:
            raise SupervisorError(f"frozen v8 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V8["snapshot_root"])
    root_metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (root_metadata.st_uid, root_metadata.st_gid) == (0, 0)
    translated_root_owner = not runtime_root and os.geteuid() != 0 and (root_metadata.st_uid, root_metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(root_metadata.st_mode) or stat.S_IMODE(root_metadata.st_mode) != 0o555
            or root_metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise SupervisorError("frozen v8 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v8.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v8.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v8.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v8.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise SupervisorError("frozen v8 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V8["snapshot"]["artifacts"][name]
        root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
        translated_root_owner = not runtime_root and os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o444
                or metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise SupervisorError(f"frozen v8 snapshot artifact differs: {name}")
    documents: dict[str, dict[str, Any]] = {}
    for name in ("start", "execution", "lifecycle"):
        path = AUDIT_ROOT / f"{PREDECESSOR_V8['probe_id']}.{name}.json"
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V8["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (metadata.st_uid, metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(metadata.st_mode) != 0o440 or metadata.st_nlink != 1):
            raise SupervisorError(f"frozen v8 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise SupervisorError(f"frozen v8 receipt status differs: {name}")
        documents[name] = document
    audit = documents["execution"].get("apparmor_audit", {})
    denial = audit.get("expected_proc_mkdir_denials", [])
    if (audit.get("matching_total") != 1 or audit.get("expected_proc_mkdir_total") != 1
            or audit.get("unexpected_total") != 0 or audit.get("storage_overflow") is not False
            or len(denial) != 1 or f'profile="{PREDECESSOR_V8["discovery"]["profile"]}"' not in denial[0]
            or 'operation="mkdir"' not in denial[0] or 'name="/newroot/proc/"' not in denial[0]):
        raise SupervisorError("frozen v8 unique mkdir-denial evidence differs")
    lifecycle = documents["lifecycle"]
    if (lifecycle.get("cleanup", {}).get("stable_zero_scans") != [[], [], []]
            or lifecycle.get("cleanup", {}).get("post_unload_stable_zero_scans") != [[], [], []]
            or lifecycle.get("sysctls", {}).get("unchanged") is not True
            or lifecycle.get("sysctls", {}).get("before") != lifecycle.get("sysctls", {}).get("after")
            or any(lifecycle.get("profiles_after", {}).get("kernel_exact_counts", {}).values())
            or lifecycle.get("sudo_timestamp", {}).get("clear", {}).get("returncode") != 0
            or lifecycle.get("sudo_timestamp", {}).get("noninteractive_true_must_fail", {}).get("returncode") != 1):
        raise SupervisorError("frozen v8 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "verified": True}


def verify_predecessor_v9_provenance(*, runtime_root: bool = False) -> dict[str, Any]:
    if runtime_root and os.geteuid() != 0:
        raise SupervisorError("runtime predecessor verification requires euid 0")
    package = WORKSPACE_PACKAGE_ROOT
    workspace_paths = {
        "gate": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v9.py",
        "supervisor": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v9.py",
        "profile": package / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v9.profile",
        "schema": package / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v9.json",
        "policy": package / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v9.json",
        "tests": package / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v9.py",
    }
    for name, path in workspace_paths.items():
        raw = read_regular_bytes(path)
        expected = PREDECESSOR_V9["workspace_artifacts"][name]
        if len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]:
            raise SupervisorError(f"frozen v9 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V9["snapshot_root"])
    root_metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (root_metadata.st_uid, root_metadata.st_gid) == (0, 0)
    translated_root_owner = not runtime_root and os.geteuid() != 0 and (root_metadata.st_uid, root_metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(root_metadata.st_mode) or stat.S_IMODE(root_metadata.st_mode) != 0o555
            or root_metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise SupervisorError("frozen v9 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v9.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v9.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v9.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v9.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v9.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise SupervisorError("frozen v9 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V9["snapshot"]["artifacts"][name]
        root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
        translated_root_owner = not runtime_root and os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o444
                or metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise SupervisorError(f"frozen v9 snapshot artifact differs: {name}")
    documents: dict[str, dict[str, Any]] = {}
    for name in ("start", "execution", "lifecycle"):
        path = AUDIT_ROOT / f"{PREDECESSOR_V9['probe_id']}.{name}.json"
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V9["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (metadata.st_uid, metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(metadata.st_mode) != 0o440 or metadata.st_nlink != 1):
            raise SupervisorError(f"frozen v9 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise SupervisorError(f"frozen v9 receipt status differs: {name}")
        documents[name] = document
    for suffix in PREDECESSOR_V9["other_receipts_absent"]:
        if (AUDIT_ROOT / f"{PREDECESSOR_V9['probe_id']}.{suffix}.json").exists():
            raise SupervisorError(f"unexpected frozen v9 receipt exists: {suffix}")
    execution = documents["execution"]
    audit = execution.get("apparmor_audit", {})
    discovery = PREDECESSOR_V9["discovery"]
    denial = audit.get("expected_mount_denials", [])
    fields = {
        item.get("key"): item.get("value")
        for item in denial[0].get("fields", [])
    } if len(denial) == 1 and type(denial[0]) is dict else {}
    expected_fields = {
        "apparmor": "DENIED", "operation": "mount", "class": "mount",
        "info": "failed mntpnt match", "error": "-13", "profile": discovery["profile"],
        "name": "/newroot/proc/", "comm": "bwrap", "fstype": "proc",
        "srcname": "proc", "flags": "rw, nosuid, nodev, noexec",
    }
    gate = execution.get("gate", {})
    if (execution.get("production_authorized") is not False
            or execution.get("gencase_or_solver_executed") is not False
            or execution.get("host_writable_mount_used") is not False
            or execution.get("network_or_device_exposed") is not False
            or audit.get("capture_valid") is not True or audit.get("capture_errors") != []
            or audit.get("matching_total") != 1 or audit.get("expected_mount_total") != 1
            or audit.get("mkdir_total") != 0 or audit.get("unexpected_total") != 0
            or audit.get("storage_overflow") is not False or len(denial) != 1
            or denial[0].get("line_sha256") != discovery["audit_line_sha256"]
            or any(fields.get(key) != value for key, value in expected_fields.items())
            or type(fields.get("pid")) is not str or not fields["pid"].isdigit() or int(fields["pid"]) < 1
            or gate.get("returncode") != 1 or gate.get("stdout_size_bytes") != 0
            or gate.get("stderr_utf8_prefix") != discovery["gate_stderr_utf8"]
            or gate.get("stderr_sha256") != discovery["gate_stderr_sha256"]):
        raise SupervisorError("frozen v9 exact proc mount-denial evidence differs")
    lifecycle = documents["lifecycle"]
    cleanup = lifecycle.get("cleanup", {})
    if (cleanup.get("initial") != [] or cleanup.get("term_sent") != []
            or cleanup.get("after_term") != [] or cleanup.get("kill_sent") != []
            or cleanup.get("stable_zero_scans") != [[], [], []]
            or cleanup.get("post_unload_stable_zero_scans") != [[], [], []]
            or lifecycle.get("sysctls", {}).get("unchanged") is not True
            or lifecycle.get("sysctls", {}).get("before") != lifecycle.get("sysctls", {}).get("after")
            or any(lifecycle.get("profiles_after", {}).get("kernel_exact_counts", {}).values())
            or lifecycle.get("sudo_timestamp", {}).get("clear", {}).get("returncode") != 0
            or lifecycle.get("sudo_timestamp", {}).get("noninteractive_true_must_fail", {}).get("returncode") != 1):
        raise SupervisorError("frozen v9 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "verified": True}


def verify_predecessor_v10_provenance(*, runtime_root: bool = False) -> dict[str, Any]:
    if runtime_root and os.geteuid() != 0:
        raise SupervisorError("runtime predecessor verification requires euid 0")
    package = WORKSPACE_PACKAGE_ROOT
    workspace_paths = {
        "gate": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v10.py",
        "supervisor": package / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v10.py",
        "profile": package / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v10.profile",
        "schema": package / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v10.json",
        "policy": package / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v10.json",
        "tests": package / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v10.py",
    }
    for name, path in workspace_paths.items():
        raw = read_regular_bytes(path)
        expected = PREDECESSOR_V10["workspace_artifacts"][name]
        if len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]:
            raise SupervisorError(f"frozen v10 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V10["snapshot_root"])
    root_metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (root_metadata.st_uid, root_metadata.st_gid) == (0, 0)
    translated_root_owner = not runtime_root and os.geteuid() != 0 and (root_metadata.st_uid, root_metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(root_metadata.st_mode) or stat.S_IMODE(root_metadata.st_mode) != 0o555
            or root_metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise SupervisorError("frozen v10 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v10.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v10.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v10.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v10.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v10.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise SupervisorError("frozen v10 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V10["snapshot"]["artifacts"][name]
        root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
        translated_root_owner = not runtime_root and os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o444
                or metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise SupervisorError(f"frozen v10 snapshot artifact differs: {name}")
    documents: dict[str, dict[str, Any]] = {}
    for name in ("start", "execution", "lifecycle"):
        path = AUDIT_ROOT / f"{PREDECESSOR_V10['probe_id']}.{name}.json"
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V10["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (metadata.st_uid, metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(metadata.st_mode) != 0o440 or metadata.st_nlink != 1):
            raise SupervisorError(f"frozen v10 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise SupervisorError(f"frozen v10 receipt status differs: {name}")
        documents[name] = document
    for suffix in PREDECESSOR_V10["other_receipts_absent"]:
        if (AUDIT_ROOT / f"{PREDECESSOR_V10['probe_id']}.{suffix}.json").exists():
            raise SupervisorError(f"unexpected frozen v10 receipt exists: {suffix}")
    execution = documents["execution"]
    audit = execution.get("apparmor_audit", {})
    discovery = PREDECESSOR_V10["discovery"]
    denials = audit.get("unexpected_denials", [])
    if type(denials) is not list or len(denials) != len(discovery["denials"]):
        raise SupervisorError("frozen v10 denial count differs")
    for entry, expected in zip(denials, discovery["denials"], strict=True):
        fields = {
            item.get("key"): item.get("value")
            for item in entry.get("fields", [])
            if type(item) is dict
        } if type(entry) is dict else {}
        for key in ("operation", "class", "comm", "capname", "name", "info", "error", "target"):
            if key in expected and fields.get(key) != expected[key]:
                raise SupervisorError(f"frozen v10 denial field differs: {key}")
        if (fields.get("apparmor") != "DENIED"
                or fields.get("profile") != "r8-liquid-u3-stdio-transport-bootstrap-v10-20260808t045130z"
                or type(fields.get("pid")) is not str or not fields["pid"].isdigit() or int(fields["pid"]) < 1
                or entry.get("line_sha256") != expected["line_sha256"]
                or entry.get("line_size_bytes") != expected["line_size_bytes"]):
            raise SupervisorError("frozen v10 denial identity differs")
    gate = execution.get("gate", {})
    if (execution.get("production_authorized") is not False
            or execution.get("gencase_or_solver_executed") is not False
            or execution.get("host_writable_mount_used") is not False
            or execution.get("network_or_device_exposed") is not False
            or audit.get("capture_valid") is not discovery["capture_valid"]
            or audit.get("capture_errors") != []
            or audit.get("matching_total") != discovery["matching_total"]
            or audit.get("stored_count") != discovery["stored_count"]
            or audit.get("dropped_count") != discovery["dropped_count"]
            or audit.get("storage_overflow") is not discovery["storage_overflow"]
            or audit.get("expected_mount_total") != discovery["expected_mount_total"]
            or audit.get("mkdir_total") != discovery["mkdir_total"]
            or audit.get("unexpected_total") != discovery["unexpected_total"]
            or audit.get("raw_stdout_sha256") != discovery["raw_stdout_sha256"]
            or audit.get("raw_stdout_size_bytes") != discovery["raw_stdout_size_bytes"]
            or audit.get("sanitized_denials") != denials
            or gate.get("returncode") != discovery["gate_returncode"]
            or gate.get("stdout_size_bytes") != discovery["gate_stdout_size_bytes"]
            or gate.get("stderr_utf8_prefix") != discovery["gate_stderr_utf8"]
            or gate.get("stderr_sha256") != discovery["gate_stderr_sha256"]):
        raise SupervisorError("frozen v10 execution evidence differs")
    lifecycle = documents["lifecycle"]
    cleanup = lifecycle.get("cleanup", {})
    if (cleanup.get("initial") != [] or cleanup.get("term_sent") != []
            or cleanup.get("after_term") != [] or cleanup.get("kill_sent") != []
            or cleanup.get("stable_zero_scans") != [[], [], []]
            or cleanup.get("post_unload_stable_zero_scans") != [[], [], []]
            or lifecycle.get("sysctls", {}).get("unchanged") is not True
            or lifecycle.get("sysctls", {}).get("before") != lifecycle.get("sysctls", {}).get("after")
            or any(lifecycle.get("profiles_after", {}).get("kernel_exact_counts", {}).values())
            or lifecycle.get("sudo_timestamp", {}).get("clear", {}).get("returncode") != 0
            or lifecycle.get("sudo_timestamp", {}).get("noninteractive_true_must_fail", {}).get("returncode") != 1):
        raise SupervisorError("frozen v10 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "denials": 5, "verified": True}

def repository_static_review() -> dict[str, Any]:
    policy = read_json(REPOSITORY_PATHS["policy"])
    schema = read_json(REPOSITORY_PATHS["schema"])
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("closed policy/schema top-level contract differs")
    if policy.get("status") != "STATIC_READY_V11_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT":
        raise SupervisorError("v11 static status differs")
    if policy.get("authorization", {}).get("static_task_execution_performed") is not False:
        raise SupervisorError("static draft incorrectly claims execution")
    transport = policy.get("stdio_transport", {})
    if transport.get("success_magic_hex") != SUCCESS_MAGIC.hex():
        raise SupervisorError("fresh v11 success protocol magic differs")
    expected_frozen = {
        "probe_id": PROBE_ID,
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        **FROZEN_RECEIPT_PATHS,
    }
    if policy.get("frozen_identity") != expected_frozen:
        raise SupervisorError("frozen probe identity, labels, or receipt paths differ")
    commands = policy.get("fixed_commands", {})
    profile_path = str(SNAPSHOT_PATHS["profile"])
    expected_commands = {
        "aa_status_json_argv": list(AA_STATUS_JSON_ARGV),
        "gate_handoff_argv": gate_handoff_argv(),
        "parser_parse_argv": ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path],
        "parser_load_argv": ["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path],
        "parser_unload_argv": ["/usr/sbin/apparmor_parser", "-R", "-K", profile_path],
        "snapshot_producer_argv": ["/usr/bin/python3.12", "-I", "-B", str(SCRIPT_PATH), "emit-bootstrap"],
        "snapshot_root_loader_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", "-c",
            "<PINNED_SNAPSHOT_BOOTSTRAP_LOADER_SOURCE>", "<TEN_SHA256_SIZE_MANIFEST_ARGUMENTS>",
        ],
        "run_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["supervisor"]),
            "run", "--policy-sha256",
            "<EXTERNALLY_FROZEN_POLICY_SHA256>", "--admission-token", ADMISSION_TOKEN,
        ],
        "recover_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["supervisor"]),
            "recover-only", "--policy-sha256",
            "<EXTERNALLY_FROZEN_POLICY_SHA256>", "--recovery-token", RECOVERY_TOKEN,
        ],
        "sudo_timestamp_clear_argv": sudo_timestamp_argvs()[0],
        "sudo_timestamp_verify_argv": sudo_timestamp_argvs()[1],
    }
    for name, expected in expected_commands.items():
        if commands.get(name) != expected:
            raise SupervisorError(f"fixed command policy differs: {name}")
    if commands.get("operator_transport") != "snapshot producer stdout is root loader stdin; run and recover use pinned Python to read the root-owned 0444 snapshot supervisor; sudo password is read only from the inherited /dev/tty":
        raise SupervisorError("fixed operator transport policy differs")
    if commands.get("shell") is not False or commands.get("workspace_run_forbidden") is not True:
        raise SupervisorError("fixed command shell/workspace boundary differs")
    bootstrap = policy.get("snapshot_bootstrap", {})
    if (bootstrap.get("file_mode") != "0444"
            or bootstrap.get("runtime_supervisor_entrypoint") != "/usr/bin/python3.12 -I -B reads root-owned 0444 snapshot supervisor"
            or bootstrap.get("direct_snapshot_supervisor_exec_forbidden") is not True):
        raise SupervisorError("snapshot file mode and pinned Python entrypoint contract differ")
    expected_producer = {
        "workspace_snapshot_producer_root_forbidden": True,
        "workspace_snapshot_producer_exact_path": str(WORKSPACE_SUPERVISOR_PATH),
        "workspace_snapshot_producer_uid": [HOST_UID] * 4,
        "workspace_snapshot_producer_gid": [HOST_GID] * 4,
        "workspace_snapshot_producer_active_capability_sets": {"CapInh": 0, "CapPrm": 0, "CapEff": 0, "CapAmb": 0},
        "workspace_snapshot_producer_stdout_on_identity_failure": "empty",
    }
    if any(bootstrap.get(key) != value for key, value in expected_producer.items()):
        raise SupervisorError("unprivileged workspace snapshot producer identity contract differs")
    if policy.get("provenance", {}).get("predecessor_no_go") != PREDECESSOR_NO_GO:
        raise SupervisorError("predecessor v7 pre-start cleanup NO_GO record differs")
    if policy.get("provenance", {}).get("predecessor_v8") != PREDECESSOR_V8:
        raise SupervisorError("predecessor v8 frozen provenance record differs")
    predecessor_v8 = verify_predecessor_v8_provenance()
    if policy.get("provenance", {}).get("predecessor_v9") != PREDECESSOR_V9:
        raise SupervisorError("predecessor v9 frozen provenance record differs")
    predecessor_v9 = verify_predecessor_v9_provenance()
    if policy.get("provenance", {}).get("predecessor_v10") != PREDECESSOR_V10:
        raise SupervisorError("predecessor v10 frozen provenance record differs")
    predecessor_v10 = verify_predecessor_v10_provenance()
    receipt_contract = policy.get("receipt_contract", {})
    if (receipt_contract.get("sudo_group_membership") != SUDO_GROUP_MEMBERSHIP_CONTRACT
            or receipt_contract.get("sudo_cleanup_identity") != SUDO_CLEANUP_IDENTITY_CONTRACT
            or receipt_contract.get("sudo_cleanup_bounding_mode") != SUDO_CLEANUP_BOUNDING_MODE):
        raise SupervisorError("sudo cleanup membership or identity contract differs")
    mount_policy = policy.get("mount_discovery", {})
    mount_digest = mount_policy.get("effective_profile_lines_sha256")
    semantics_digest = policy.get("profile_semantics", {}).get("effective_lines_sha256")
    if semantics_digest != mount_digest:
        raise SupervisorError("duplicate effective profile semantics digests disagree")
    semantics = policy.get("profile_semantics", {})
    expected_runtime_semantics = {
        "runtime_execution": "/usr/bin/sleep rix,",
        "runtime_profile_loaded_but_unreachable": True,
        "runtime_authority": "inherits_bootstrap_label_with_empty_groups_zero_caps_nnp_nested_userns_disabled_and_empty_network_namespace",
        "explicit_silent_denials": ["deny capability dac_override,"],
        "avoidable_python_reads_prevented_by": ["python_-S", "TZ=UTC0"],
    }
    if any(semantics.get(key) != value for key, value in expected_runtime_semantics.items()):
        raise SupervisorError("v11 inherited-runtime and explicit-deny semantics differ")
    expected_v9_denial = {
        "profile": "bootstrap", "apparmor": "DENIED", "operation": "mount", "class": "mount",
        "info": "failed mntpnt match", "error": "-13", "name": "/newroot/proc/",
        "comm": "bwrap", "fstype": "proc", "srcname": "proc",
        "flags_required": ["rw", "nosuid", "nodev", "noexec"], "flags_optional": [],
    }
    if (mount_policy.get("proc_rules") != [PROC_MOUNT_RULE]
            or mount_policy.get("predecessor_v9_denial_evidence") != expected_v9_denial
            or mount_policy.get("zero_logged_denials_required") is not True
            or mount_policy.get("rules_may_be_guessed") is not False
            or mount_policy.get("success_frame_required") is not True):
        raise SupervisorError("v11 evidence-derived proc success admission contract differs")
    if policy.get("evidence_classification") != EVIDENCE_CLASSIFICATION_CONTRACT:
        raise SupervisorError("v11 zero-denial success evidence classification contract differs")
    if policy.get("trusted_system_tools", {}).get("root_supervisor") != trusted_tool_policy():
        raise SupervisorError("root supervisor trusted-tool policy differs")
    if policy.get("recovery_contract", {}).get("token") != RECOVERY_TOKEN:
        raise SupervisorError("cleanup-only recovery token policy differs")
    if policy.get("admission_capability", {}).get("public_environment_token_is_authority") is not False:
        raise SupervisorError("public admission token is incorrectly treated as authority")
    artifacts = policy.get("trusted_artifacts", {})
    if set(artifacts) != {"gate", "supervisor", "profile", "schema"}:
        raise SupervisorError("trusted runtime artifact set differs")
    observed: dict[str, dict[str, Any]] = {}
    for name in ("gate", "supervisor", "profile", "schema"):
        raw = read_regular_bytes(REPOSITORY_PATHS[name])
        entry = artifacts[name]
        if entry.get("sha256") != sha256_bytes(raw) or entry.get("size_bytes") != len(raw):
            raise SupervisorError(f"trusted artifact bytes differ: {name}")
        observed[name] = {"path": str(REPOSITORY_PATHS[name]), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}
    if bootstrap.get("source_sha256") != SNAPSHOT_BOOTSTRAP_SHA256 or bootstrap.get("source_size_bytes") != len(SNAPSHOT_BOOTSTRAP_BYTES):
        raise SupervisorError("minimal root bootstrap identity differs")
    if bootstrap.get("loader_sha256") != SNAPSHOT_BOOTSTRAP_LOADER_SHA256 or bootstrap.get("loader_size_bytes") != len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES):
        raise SupervisorError("minimal root bootstrap loader identity differs")
    policy_raw = read_regular_bytes(REPOSITORY_PATHS["policy"])
    return {"policy_sha256": sha256_bytes(policy_raw), "policy_size_bytes": len(policy_raw), "artifacts": observed, "bootstrap_source_sha256": SNAPSHOT_BOOTSTRAP_SHA256, "predecessor_v8": predecessor_v8, "predecessor_v9": predecessor_v9, "predecessor_v10": predecessor_v10, "execution_performed": False}


def snapshot_manifest_arguments(review: Mapping[str, Any]) -> list[str]:
    arguments: list[str] = []
    for name in ("gate", "supervisor", "profile", "schema"):
        entry = review["artifacts"][name]
        arguments.extend((entry["sha256"], str(entry["size_bytes"])))
    arguments.extend((review["policy_sha256"], str(review["policy_size_bytes"])))
    return arguments


def verify_snapshot_producer_identity() -> dict[str, Any]:
    """Forbid root or non-canonical workspace execution of the byte producer."""

    if SCRIPT_PATH != WORKSPACE_SUPERVISOR_PATH:
        raise SupervisorError("snapshot producer must be the exact workspace v11 supervisor")
    if (os.getuid(), os.geteuid(), os.getgid(), os.getegid()) != (HOST_UID, HOST_UID, HOST_GID, HOST_GID):
        raise SupervisorError("snapshot producer must never execute as root or another identity")
    fields: dict[str, str] = {}
    for line in Path("/proc/self/status").read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    try:
        uid = [int(value) for value in fields["Uid"].split()]
        gid = [int(value) for value in fields["Gid"].split()]
        capabilities = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapAmb")}
    except (KeyError, ValueError) as exc:
        raise SupervisorError("cannot parse snapshot producer identity") from exc
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or any(capabilities.values()):
        raise SupervisorError("snapshot producer UID/GID or active capability sets differ")
    return {"uid": uid, "gid": gid, "active_capabilities": capabilities, "root_forbidden": True, "script_path": str(SCRIPT_PATH)}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def require_tool(name: str) -> dict[str, Any]:
    path, expected = TRUSTED_TOOLS[name]
    raw = read_regular_bytes(path, limit=512 * 1024 * 1024)
    metadata = os.stat(path, follow_symlinks=False)
    if metadata.st_uid != 0 or metadata.st_gid != 0 or not metadata.st_mode & stat.S_IXUSR or sha256_bytes(raw) != expected:
        raise SupervisorError(f"trusted tool identity differs: {path}")
    return {"path": str(path), "sha256": expected, "size_bytes": len(raw)}


def verify_snapshot(policy_sha256: str) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    if os.geteuid() != 0 or SCRIPT_PATH != SNAPSHOT_PATHS["supervisor"]:
        raise SupervisorError("run requires the snapshot supervisor with euid 0")
    root = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0 or stat.S_IMODE(root.st_mode) != 0o555:
        raise SupervisorError("snapshot root ownership or mode differs")
    policy_raw = read_regular_bytes(SNAPSHOT_PATHS["policy"])
    if len(policy_sha256) != 64 or sha256_bytes(policy_raw) != policy_sha256:
        raise SupervisorError("externally frozen snapshot policy digest differs")
    policy = json.loads(policy_raw.decode("utf-8"))
    if not isinstance(policy, dict):
        raise SupervisorError("snapshot policy root is not an object")
    artifacts = policy.get("trusted_artifacts", {})
    observed: dict[str, dict[str, Any]] = {}
    for name, path in SNAPSHOT_PATHS.items():
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444:
            raise SupervisorError(f"snapshot artifact metadata differs: {name}")
        raw = read_regular_bytes(path)
        if name == "policy":
            expected_digest, expected_size = policy_sha256, len(policy_raw)
        else:
            entry = artifacts.get(name)
            if not isinstance(entry, dict):
                raise SupervisorError(f"snapshot artifact policy missing: {name}")
            expected_digest, expected_size = entry.get("sha256"), entry.get("size_bytes")
        if sha256_bytes(raw) != expected_digest or len(raw) != expected_size:
            raise SupervisorError(f"snapshot artifact bytes differ: {name}")
        observed[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "uid": metadata.st_uid, "gid": metadata.st_gid, "mode": "0444"}
    schema = json.loads(read_regular_bytes(SNAPSHOT_PATHS["schema"]).decode("utf-8"))
    if not isinstance(schema, dict):
        raise SupervisorError("snapshot schema root is not an object")
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if policy.get("authorization", {}).get("admission_token") != ADMISSION_TOKEN:
        raise SupervisorError("snapshot policy admission identity differs")
    if policy.get("provenance", {}).get("predecessor_v8") != PREDECESSOR_V8:
        raise SupervisorError("snapshot policy predecessor v8 provenance differs")
    verify_predecessor_v8_provenance(runtime_root=True)
    if policy.get("provenance", {}).get("predecessor_v9") != PREDECESSOR_V9:
        raise SupervisorError("snapshot policy predecessor v9 provenance differs")
    verify_predecessor_v9_provenance(runtime_root=True)
    if policy.get("provenance", {}).get("predecessor_v10") != PREDECESSOR_V10:
        raise SupervisorError("snapshot policy predecessor v10 provenance differs")
    verify_predecessor_v10_provenance(runtime_root=True)
    return policy, observed


def _proc_starttime(pid: int) -> int | None:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except FileNotFoundError:
        return None
    end = raw.rfind(")")
    fields = raw[end + 2:].split() if end >= 0 else []
    if len(fields) < 20:
        return None
    try:
        return int(fields[19])
    except ValueError:
        return None


def _signal_open_pidfd(descriptor: int, sig: signal.Signals) -> bool:
    try:
        signal.pidfd_send_signal(descriptor, sig)
        return True
    except ProcessLookupError:
        return False


def _signal_pidfd(pid: int, sig: signal.Signals, *, expected_starttime: int) -> bool:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise SupervisorError("pidfd signaling is unavailable")
    if not isinstance(expected_starttime, int) or isinstance(expected_starttime, bool) or expected_starttime < 1:
        raise SupervisorError("refusing to signal a task without a frozen positive starttime")
    try:
        descriptor = os.pidfd_open(pid, 0)
    except ProcessLookupError:
        return False
    try:
        if _proc_starttime(pid) != expected_starttime:
            return False
        return _signal_open_pidfd(descriptor, sig)
    finally:
        os.close(descriptor)


def _bounded_extend(target: bytearray, block: bytes, limit: int) -> bool:
    remaining = max(0, limit - len(target))
    target.extend(block[:remaining])
    return len(block) > remaining


def run_bounded_command(
    argv: list[str],
    *,
    stdin_bytes: bytes | None = b"",
    stdout_limit: int = 65_536,
    stderr_limit: int = 65_536,
    timeout_seconds: float = 30,
    env: Mapping[str, str] | None = None,
    pass_fds: tuple[int, ...] = (),
    start_new_session: bool = True,
) -> dict[str, Any]:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise SupervisorError("pidfd signaling is unavailable")
    process = subprocess.Popen(
        argv,
        stdin=None if stdin_bytes is None else subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env=dict(env or {"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"}),
        close_fds=True,
        pass_fds=pass_fds,
        start_new_session=start_new_session,
    )
    try:
        leader_pidfd = os.pidfd_open(process.pid, 0)
    except BaseException:
        # The direct child is still unreaped, so its numeric PID cannot be
        # reused while this birth-time pidfd fallback kills and reaps it.
        process.kill()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        raise
    selector: selectors.BaseSelector | None = None
    stdout = bytearray()
    stderr = bytearray()
    position = 0
    streams: dict[str, Any] = {}
    failure: str | None = None
    cleanup_failure: str | None = None
    deadline = time.monotonic() + timeout_seconds
    try:
        if process.stdout is None or process.stderr is None or (stdin_bytes is not None and process.stdin is None):
            raise SupervisorError("bounded command pipes are unavailable")
        selector = selectors.DefaultSelector()
        streams = {"stdout": process.stdout, "stderr": process.stderr}
        for name, stream in streams.items():
            os.set_blocking(stream.fileno(), False)
            selector.register(stream.fileno(), selectors.EVENT_READ, name)
        if stdin_bytes is not None:
            os.set_blocking(process.stdin.fileno(), False)
            selector.register(process.stdin.fileno(), selectors.EVENT_WRITE, "stdin")
        while selector.get_map():
            if time.monotonic() >= deadline:
                failure = failure or "deadline"
                break
            events = selector.select(timeout=0.1)
            for key, _mask in events:
                channel = key.data
                descriptor = key.fd
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, stdin_bytes[position:position + 4096])
                    except (BrokenPipeError, ConnectionResetError):
                        count = 0
                    except BlockingIOError:
                        continue
                    if count:
                        position += count
                    if not count or position == len(stdin_bytes):
                        selector.unregister(descriptor)
                        process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 4096)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    streams[channel].close()
                    continue
                target = stdout if channel == "stdout" else stderr
                limit = stdout_limit if channel == "stdout" else stderr_limit
                if _bounded_extend(target, block, limit):
                    failure = f"{channel}_overflow"
                    break
            if failure is not None:
                break
        if failure is not None:
            _signal_open_pidfd(leader_pidfd, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
    finally:
        if selector is not None:
            selector.close()
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
        if process.poll() is None:
            try:
                process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                failure = failure or "leader_alive_after_pipe_close"
                try:
                    _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                except OSError as exc:
                    cleanup_failure = f"final leader pidfd kill failed: {exc}"
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            cleanup_failure = cleanup_failure or "bounded command leader remained unreaped"
        os.close(leader_pidfd)
        if cleanup_failure is not None:
            raise SupervisorError(cleanup_failure)
    returncode = process.returncode
    if returncode is None:
        raise SupervisorError("bounded command return code is unavailable after reap")
    return {
        "argv": argv,
        "returncode": returncode,
        "stdout": bytes(stdout),
        "stderr": bytes(stderr),
        "stdin_size_bytes": None if stdin_bytes is None else len(stdin_bytes),
        "stdin_fully_written": stdin_bytes is None or position == len(stdin_bytes),
        "failure": failure,
        "start_new_session": start_new_session,
    }


def _command_evidence(result: Mapping[str, Any], *, include_prefix: bool = False) -> dict[str, Any]:
    stdout = result["stdout"]
    stderr = result["stderr"]
    evidence = {
        "argv": result["argv"],
        "returncode": result["returncode"],
        "stdout_size_bytes": len(stdout),
        "stdout_sha256": sha256_bytes(stdout),
        "stderr_size_bytes": len(stderr),
        "stderr_sha256": sha256_bytes(stderr),
        "failure": result["failure"],
        "start_new_session": result["start_new_session"],
    }
    if include_prefix:
        evidence["stderr_utf8_prefix"] = stderr[:4096].decode("utf-8", "replace")
    return evidence


def run_checked(
    argv: list[str],
    *,
    timeout_seconds: float = 30,
    stdout_limit: int = 65_536,
    stderr_limit: int = 65_536,
    require_silent: bool = False,
) -> dict[str, Any]:
    result = run_bounded_command(argv, timeout_seconds=timeout_seconds, stdout_limit=stdout_limit, stderr_limit=stderr_limit)
    if (result["returncode"] != 0 or result["failure"] is not None
            or (require_silent and (result["stdout"] or result["stderr"]))):
        raise SupervisorError(f"trusted command failed: {argv[0]}")
    return result


def _read_proc_file(proc_root: Path, pid: int, relative: str, *, limit: int = 4096) -> bytes:
    directory = os.open(proc_root / str(pid), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        descriptor = os.open(relative, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory)
        try:
            result = bytearray()
            while True:
                block = os.read(descriptor, min(4096, limit + 1 - len(result)))
                if not block:
                    return bytes(result)
                result.extend(block)
                if len(result) > limit:
                    raise SupervisorError(f"proc file exceeds ceiling: {pid}/{relative}")
        finally:
            os.close(descriptor)
    finally:
        os.close(directory)


def _stat_starttime(stat_raw: str, *, identity: str) -> int:
    end = stat_raw.rfind(")")
    fields = stat_raw[end + 2:].split() if end >= 0 else []
    if len(fields) < 20:
        raise SupervisorError(f"cannot parse proc starttime for {identity}")
    try:
        starttime = int(fields[19])
    except ValueError as exc:
        raise SupervisorError(f"cannot parse proc starttime for {identity}") from exc
    if starttime < 1:
        raise SupervisorError(f"non-positive proc starttime for {identity}")
    return starttime


def _task_record(proc_root: Path, tgid: int, tid: int) -> dict[str, Any]:
    attr = _read_proc_file(proc_root, tgid, f"task/{tid}/attr/current").decode("utf-8")
    task_stat = _read_proc_file(proc_root, tgid, f"task/{tid}/stat").decode("ascii")
    leader_stat = _read_proc_file(proc_root, tgid, "stat").decode("ascii")
    return {
        "tgid": tgid,
        "tid": tid,
        "label": attr.strip().split(" (", 1)[0],
        "attr_current": attr.strip(),
        "tgid_starttime": _stat_starttime(leader_stat, identity=f"tgid {tgid}"),
        "tid_starttime": _stat_starttime(task_stat, identity=f"tgid {tgid} tid {tid}"),
    }


def labeled_processes(proc_root: Path = Path("/proc")) -> list[dict[str, Any]]:
    """Authoritatively enumerate every task/thread carrying a fresh v11 label."""

    observed: list[dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            task_entries = tuple((entry / "task").iterdir())
        except (FileNotFoundError, ProcessLookupError) as exc:
            if isinstance(exc, OSError) and exc.errno not in (None, errno.ENOENT, errno.ESRCH):
                raise
            continue
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ESRCH):
                continue
            raise SupervisorError(f"cannot enumerate authoritative tasks for tgid {entry.name}: {exc}") from exc
        tgid = int(entry.name)
        for task_entry in task_entries:
            if not task_entry.name.isdecimal():
                continue
            tid = int(task_entry.name)
            try:
                record = _task_record(proc_root, tgid, tid)
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                if exc.errno in (errno.ENOENT, errno.ESRCH):
                    continue
                raise SupervisorError(f"cannot read authoritative attr/current for tgid {tgid} tid {tid}: {exc}") from exc
            if record["label"] in LABELS:
                observed.append(record)
    return sorted(observed, key=lambda item: (item["tgid"], item["tid"]))


def require_stable_zero_labels(*, scans: int = 3, interval: float = 0.1) -> list[list[dict[str, Any]]]:
    if scans != 3:
        raise SupervisorError("exactly three stable-zero scans are required")
    history: list[list[dict[str, Any]]] = []
    for index in range(scans):
        current = labeled_processes()
        history.append(current)
        if current:
            raise SupervisorError(f"labeled process residue remains: {current}")
        if index + 1 < scans:
            time.sleep(interval)
    return history


def _signal_labeled(records: Iterable[Mapping[str, Any]], sig: signal.Signals) -> list[dict[str, Any]]:
    sent: list[dict[str, Any]] = []
    signaled_tgids: set[int] = set()
    for item in records:
        tgid = int(item["tgid"])
        tid = int(item["tid"])
        if tgid in signaled_tgids:
            continue
        tgid_starttime = int(item["tgid_starttime"])
        tid_starttime = int(item["tid_starttime"])
        try:
            current = _task_record(Path("/proc"), tgid, tid)
        except (FileNotFoundError, ProcessLookupError):
            continue
        if (current["tgid_starttime"] != tgid_starttime or current["tid_starttime"] != tid_starttime
                or current["label"] not in LABELS):
            continue
        if _signal_pidfd(tgid, sig, expected_starttime=tgid_starttime):
            signaled_tgids.add(tgid)
            sent.append({
                "tgid": tgid,
                "trigger_tid": tid,
                "tgid_starttime": tgid_starttime,
                "trigger_tid_starttime": tid_starttime,
                "label": current["label"],
                "signal": int(sig),
            })
    return sent


def terminate_labeled_processes() -> dict[str, Any]:
    initial = labeled_processes()
    term_sent = _signal_labeled(initial, signal.SIGTERM)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if not labeled_processes():
            break
        time.sleep(0.1)
    after_term = labeled_processes()
    kill_sent = _signal_labeled(after_term, signal.SIGKILL)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if not labeled_processes():
            break
        time.sleep(0.1)
    stable = require_stable_zero_labels()
    return {"initial": initial, "term_sent": term_sent, "after_term": after_term, "kill_sent": kill_sent, "stable_zero_scans": stable}


def read_sysctls() -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}
    for path in SYSCTL_PATHS:
        raw = read_regular_bytes(path, limit=128)
        try:
            value = raw.decode("ascii")
        except UnicodeError as exc:
            raise SupervisorError(f"sysctl is not ASCII: {path}") from exc
        result[str(path)] = {"value": value, "sha256": sha256_bytes(raw)}
    if set(result) != {str(path) for path in SYSCTL_PATHS}:
        raise SupervisorError("exact three-sysctl snapshot differs")
    return result


def read_kernel_profile_entries() -> dict[str, list[str]]:
    raw = read_regular_bytes(KERNEL_PROFILE_PATH, limit=8 * 1024 * 1024)
    entries: dict[str, list[str]] = {label: [] for label in LABELS}
    for line in raw.decode("utf-8", "strict").splitlines():
        name = line.split(" (", 1)[0]
        if name in entries:
            entries[name].append(line)
    return entries


def read_kernel_profile_counts() -> dict[str, int]:
    return {label: len(lines) for label, lines in read_kernel_profile_entries().items()}


def profile_state() -> dict[str, Any]:
    entries = read_kernel_profile_entries()
    counts = {label: len(lines) for label, lines in entries.items()}
    enforce = {label: lines == [f"{label} (enforce)"] for label, lines in entries.items()}
    result = run_bounded_command(list(AA_STATUS_JSON_ARGV), stdout_limit=2 * 1024 * 1024, stderr_limit=16_384, timeout_seconds=15)
    if result["returncode"] != 0 or result["failure"] is not None or result["stderr"]:
        raise SupervisorError("aa-status profile query failed")
    try:
        parsed = json.loads(result["stdout"].decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("aa-status JSON differs") from exc
    if type(parsed) is not dict or type(parsed.get("profiles")) is not dict:
        raise SupervisorError("aa-status JSON root/profiles object differs")
    profiles = parsed["profiles"]
    if any(type(name) is not str or type(mode) is not str for name, mode in profiles.items()):
        raise SupervisorError("aa-status JSON profile mapping differs")
    presence = {label: label in profiles for label in LABELS}
    modes = {label: profiles.get(label) for label in LABELS}
    for label in LABELS:
        if presence[label] != (counts[label] > 0):
            raise SupervisorError(f"aa-status/kernel profile state disagrees: {label}")
        if presence[label] and modes[label] != "enforce":
            raise SupervisorError(f"aa-status profile mode is not exact enforce: {label}")
    return {
        "kernel_exact_counts": counts,
        "kernel_exact_lines": entries,
        "kernel_exact_enforce": enforce,
        "aa_status_exact_presence": presence,
        "aa_status_exact_modes": modes,
        "aa_status_stdout_sha256": sha256_bytes(result["stdout"]),
        "aa_status_stdout_size_bytes": len(result["stdout"]),
    }


def require_profile_counts(state: Mapping[str, Any], expected: int) -> None:
    counts = state.get("kernel_exact_counts", {})
    lines = state.get("kernel_exact_lines", {})
    enforce = state.get("kernel_exact_enforce", {})
    presence = state.get("aa_status_exact_presence", {})
    modes = state.get("aa_status_exact_modes", {})
    if any(counts.get(label) != expected for label in LABELS):
        raise SupervisorError(f"exact profile count differs from {expected}")
    if expected == 1 and any(lines.get(label) != [f"{label} (enforce)"] or enforce.get(label) is not True for label in LABELS):
        raise SupervisorError("loaded profile mode is not exact enforce")
    if expected == 0 and any(lines.get(label) != [] for label in LABELS):
        raise SupervisorError("unloaded profile lines are not empty")
    if any(presence.get(label) is not (expected > 0) for label in LABELS):
        raise SupervisorError(f"aa-status profile presence differs from {expected > 0}")
    expected_mode = "enforce" if expected == 1 else None
    if any(modes.get(label) != expected_mode for label in LABELS):
        raise SupervisorError(f"aa-status profile mode differs from {expected_mode!r}")


def _open_absolute_directory(path: Path) -> int:
    if not path.is_absolute() or path != AUDIT_ROOT:
        raise SupervisorError("receipt directory must be the exact frozen audit root")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        current = Path("/")
        metadata = os.fstat(descriptor)
        expected = AUDIT_DIRECTORY_POLICY[current]
        if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != expected[0]
                or metadata.st_gid != expected[1] or stat.S_IMODE(metadata.st_mode) != expected[2]):
            raise SupervisorError(f"receipt directory component metadata differs: {current}")
        for part in path.parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
            current /= part
            expected = AUDIT_DIRECTORY_POLICY.get(current)
            if expected is None:
                raise SupervisorError(f"receipt directory component is outside frozen policy: {current}")
            metadata = os.fstat(descriptor)
            if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != expected[0]
                    or metadata.st_gid != expected[1] or stat.S_IMODE(metadata.st_mode) != expected[2]):
                raise SupervisorError(f"receipt directory component metadata differs: {current}")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> dict[str, Any]:
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    directory = _open_absolute_directory(path.parent)
    try:
        descriptor = os.open(path.name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o400, dir_fd=directory)
        try:
            view = memoryview(raw)
            while view:
                count = os.write(descriptor, view)
                if count <= 0:
                    raise SupervisorError(f"short one-shot O_EXCL write: {path.name}")
                view = view[count:]
            os.fsync(descriptor)
            os.fchown(descriptor, HOST_UID, HOST_GID)
            os.fchmod(descriptor, mode)
            os.fsync(descriptor)
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != mode:
                raise SupervisorError(f"append-only receipt inode differs: {path.name}")
        finally:
            os.close(descriptor)
        os.fsync(directory)
    finally:
        os.close(directory)
    return {
        "path": str(path),
        "sha256": sha256_bytes(raw),
        "size_bytes": len(raw),
        "uid": HOST_UID,
        "gid": HOST_GID,
        "mode": f"{mode:04o}",
        "creation": "O_EXCL_NOFOLLOW_ONE_SHOT_NOT_IMMUTABLE_PARENT_OWNER_CAN_REMOVE",
        "file_and_parent_fsynced": True,
    }


def _assert_one_shot_paths_absent() -> None:
    for path in (
        START_RECEIPT,
        PREFLIGHT_FAILURE_RECEIPT,
        EXECUTION_RECEIPT,
        LIFECYCLE_RECEIPT,
        LIFECYCLE_FAILURE_RECEIPT,
        RECOVERY_RECEIPT,
    ):
        try:
            os.lstat(path)
        except FileNotFoundError:
            continue
        raise SupervisorError(f"one-shot evidence path already exists: {path}")


def parse_success_frame(frame: bytes) -> dict[str, Any]:
    prefix_size = len(SUCCESS_MAGIC)
    payload_offset = prefix_size + 4
    if not payload_offset + 2 + 32 <= len(frame) <= MAX_SUCCESS_FRAME_BYTES or not frame.startswith(SUCCESS_MAGIC):
        raise SupervisorError("success frame size or magic differs")
    size = struct.unpack(">I", frame[prefix_size:payload_offset])[0]
    if size < 2 or payload_offset + size + 32 != len(frame):
        raise SupervisorError("success frame declared size differs")
    payload = frame[payload_offset:payload_offset + size]
    if hashlib.sha256(payload).digest() != frame[-32:]:
        raise SupervisorError("success frame payload digest differs")
    try:
        value = json.loads(payload.decode("ascii"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("success frame JSON differs") from exc
    if not isinstance(value, dict) or json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii") != payload:
        raise SupervisorError("success frame JSON is not canonical")
    expected_keys = {
        "status", "bootstrap_profile", "bootstrap_label", "bootstrap_identity",
        "work_tmpfs", "inputs", "host_stdin_consumed_and_fd0_replaced_with_eof_pipe",
        "fds_after_stdin", "runtime", "fds_before_success", "host_writable_mounts",
    }
    if set(value) != expected_keys:
        raise SupervisorError("success frame key set differs")
    if (value["status"] != "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V11"
            or value["bootstrap_profile"] != BOOTSTRAP_PROFILE
            or value["bootstrap_label"] != BOOTSTRAP_PROFILE + " (enforce)"):
        raise SupervisorError("success bootstrap contract differs")
    if value["host_writable_mounts"] != [] or value["host_stdin_consumed_and_fd0_replaced_with_eof_pipe"] is not True:
        raise SupervisorError("success transport isolation contract differs")
    if (type(value["fds_after_stdin"]) is not list or type(value["fds_before_success"]) is not list
            or any(type(descriptor) is not int for descriptor in value["fds_after_stdin"] + value["fds_before_success"])
            or value["fds_after_stdin"] != [0, 1, 2] or value["fds_before_success"] != [0, 1, 2]):
        raise SupervisorError("success descriptor contract differs")

    def require_guest_identity(observed: Any, context: str) -> None:
        if not isinstance(observed, dict) or set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
            raise SupervisorError(f"{context} identity key set differs")
        uid, gid, groups = observed["uid"], observed["gid"], observed["groups"]
        if (type(uid) is not list or type(gid) is not list or type(groups) is not list
                or any(type(item) is not int for item in uid + gid + groups)
                or uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups != []):
            raise SupervisorError(f"{context} identity differs")
        capabilities = observed["capabilities"]
        cap_keys = {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        if (type(capabilities) is not dict or set(capabilities) != cap_keys
                or any(type(capabilities[key]) is not int or capabilities[key] != 0 for key in cap_keys)):
            raise SupervisorError(f"{context} capability evidence differs")
        if type(observed["no_new_privs"]) is not int or observed["no_new_privs"] != 1:
            raise SupervisorError(f"{context} NNP evidence differs")

    require_guest_identity(value["bootstrap_identity"], "bootstrap")
    if value["inputs"] != EXPECTED_INPUTS:
        raise SupervisorError("success synthetic input contract differs")
    work_tmpfs = value["work_tmpfs"]
    if not isinstance(work_tmpfs, dict) or set(work_tmpfs) != {"filesystem_type", "total_bytes", "mount_options"}:
        raise SupervisorError("success tmpfs key set differs")
    options = work_tmpfs["mount_options"]
    if (work_tmpfs["filesystem_type"] != "tmpfs" or work_tmpfs["total_bytes"] != TMPFS_BYTES
            or isinstance(work_tmpfs["total_bytes"], bool) or not isinstance(options, list)
            or any(not isinstance(option, str) for option in options) or len(options) != len(set(options))
            or not {"rw", "nosuid", "nodev"}.issubset(set(options))):
        raise SupervisorError("success tmpfs contract differs")
    runtime = value["runtime"]
    if not isinstance(runtime, dict) or set(runtime) != {"label", "identity", "returncode", "stdin", "stdout_stderr"}:
        raise SupervisorError("success runtime key set differs")
    require_guest_identity(runtime["identity"], "runtime")
    if (runtime["label"] != BOOTSTRAP_PROFILE + " (enforce)" or runtime["returncode"] != -int(signal.SIGTERM)
            or isinstance(runtime["returncode"], bool) or runtime["stdin"] != "guest_internal_eof_pipe_fd0"
            or runtime["stdout_stderr"] != "internal_empty_pipes"):
        raise SupervisorError("success runtime contract differs")
    return {"payload": value, "frame_sha256": sha256_bytes(frame), "frame_size_bytes": len(frame)}


def read_boot_id() -> str:
    raw = BOOT_ID_PATH.read_bytes()
    if len(raw) != 37 or not raw.endswith(b"\n"):
        raise SupervisorError("kernel boot-id framing differs")
    try:
        value = raw[:-1].decode("ascii", "strict")
    except UnicodeError as exc:
        raise SupervisorError("kernel boot-id is not strict ASCII") from exc
    if BOOT_ID_RE.fullmatch(value) is None:
        raise SupervisorError("kernel boot-id syntax differs")
    return value


def _parse_cursor_output(raw: bytes) -> str:
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError as exc:
        raise SupervisorError("journal cursor output is not strict UTF-8") from exc
    lines = text.splitlines()
    if len(lines) != 1 or not lines[0].startswith("-- cursor: "):
        raise SupervisorError("journal cursor framing differs")
    cursor = lines[0][11:]
    if JOURNAL_CURSOR_RE.fullmatch(cursor) is None:
        raise SupervisorError("journal cursor syntax differs")
    return cursor


def _cursor_matches_boot(cursor: str, boot_id: str) -> bool:
    parts = cursor.split(";")
    fields: dict[str, str] = {}
    for part in parts:
        if part.count("=") != 1:
            return False
        key, value = part.split("=", 1)
        if not key or not value or key in fields:
            return False
        fields[key] = value
    return fields.get("b") == boot_id.replace("-", "")


def capture_journal_anchor() -> dict[str, Any]:
    boot_id = read_boot_id()
    compact_boot_id = boot_id.replace("-", "")
    argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet", f"--boot={compact_boot_id}",
        "--lines=0", "--show-cursor",
    ]
    result = run_bounded_command(
        argv, stdout_limit=4_096, stderr_limit=4_096, timeout_seconds=15,
    )
    if (type(result.get("returncode")) is not int or result["returncode"] != 0
            or result.get("failure") is not None or result.get("stderr") != b""):
        raise SupervisorError("bounded pre-run journal cursor query failed")
    cursor = _parse_cursor_output(result["stdout"])
    if not _cursor_matches_boot(cursor, boot_id):
        raise SupervisorError("journal cursor boot identity differs")
    return {
        "boot_id": boot_id,
        "cursor": cursor,
        "journal_anchor_argv": argv,
        "raw_stdout_sha256": sha256_bytes(result["stdout"]),
        "raw_stdout_size_bytes": len(result["stdout"]),
    }


def _decode_quoted_audit_value(raw: str) -> str:
    if not (raw.startswith('"') and raw.endswith('"')):
        return raw
    body = raw[1:-1]
    output: list[str] = []
    index = 0
    while index < len(body):
        character = body[index]
        if character != "\\":
            output.append(character)
            index += 1
            continue
        index += 1
        if index >= len(body):
            raise ValueError("trailing_escape")
        escaped = body[index]
        if escaped in {'"', "\\"}:
            output.append(escaped)
            index += 1
        elif escaped in {"n", "r", "t"}:
            output.append({"n": "\n", "r": "\r", "t": "\t"}[escaped])
            index += 1
        elif escaped == "x" and index + 2 < len(body) and re.fullmatch(r"[0-9A-Fa-f]{2}", body[index + 1:index + 3]):
            output.append(chr(int(body[index + 1:index + 3], 16)))
            index += 3
        else:
            raise ValueError("illegal_escape")
    value = "".join(output)
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in value):
        raise ValueError("decoded_control_character")
    return value


def _parse_audit_line(line: str) -> dict[str, Any]:
    raw = line.encode("utf-8")
    entry: dict[str, Any] = {
        "line_sha256": sha256_bytes(raw),
        "line_size_bytes": len(raw),
        "line_truncated": False,
        "sanitized_line": None,
        "parse_status": "UNPARSEABLE",
        "parse_error": None,
        "fields": [],
    }
    if len(raw) > AUDIT_LINE_MAX_BYTES:
        entry["line_truncated"] = True
        entry["parse_error"] = "LINE_TOO_LONG"
        return entry
    sanitized = re.sub(r"\b(pid|ppid|task|peer)=([0-9]+)", r"\1=<redacted>", line)
    if len(sanitized.encode("utf-8")) > AUDIT_LINE_MAX_BYTES:
        entry["line_truncated"] = True
        entry["parse_error"] = "SANITIZED_LINE_TOO_LONG"
        return entry
    entry["sanitized_line"] = sanitized
    start_match = re.search(r"(?<![A-Za-z0-9_])apparmor=", line)
    if start_match is None:
        entry["parse_error"] = "APPARMOR_FIELD_MISSING"
        return entry
    payload = line[start_match.start():]
    position = 0
    seen: set[str] = set()
    fields: list[dict[str, str]] = []
    try:
        while position < len(payload):
            match = AUDIT_TOKEN_RE.match(payload, position)
            if match is None:
                raise ValueError("TOKEN_SYNTAX")
            key, raw_value = match.group(1), match.group(2)
            if key in seen:
                raise ValueError("DUPLICATE_FIELD")
            if len(key) > 64 or len(raw_value.encode("utf-8")) > 1_024 or len(fields) >= 64:
                raise ValueError("FIELD_BOUND_EXCEEDED")
            if key in AUDIT_CRITICAL_FIELDS and "\\" in raw_value:
                raise ValueError("CRITICAL_FIELD_ESCAPE")
            value = _decode_quoted_audit_value(raw_value)
            seen.add(key)
            fields.append({"key": key, "raw_value": raw_value, "value": value})
            position = match.end()
    except ValueError as exc:
        entry["parse_error"] = str(exc)
        entry["fields"] = fields
        return entry
    entry["parse_status"] = "PARSED_UNIQUE_KV"
    entry["fields"] = fields
    return entry


def _entry_fields(entry: Mapping[str, Any]) -> dict[str, str]:
    fields = entry.get("fields")
    if type(fields) is not list:
        return {}
    observed: dict[str, str] = {}
    for field in fields:
        if (type(field) is not dict or set(field) != {"key", "raw_value", "value"}
                or any(type(field.get(key)) is not str for key in ("key", "raw_value", "value"))
                or field["key"] in observed):
            return {}
        observed[field["key"]] = field["value"]
    return observed


def _mount_flags_are_discovery_safe(raw: Any) -> bool:
    if type(raw) is not str or not raw:
        return False
    flags = [item.strip() for item in raw.split(",")]
    if any(not item for item in flags) or len(flags) != len(set(flags)):
        return False
    observed = set(flags)
    return (
        {"rw", "nosuid", "nodev", "noexec"}.issubset(observed)
        and observed.issubset({"rw", "nosuid", "nodev", "noexec", "relatime", "silent"})
    )


def _is_exact_mount_discovery(entry: Mapping[str, Any]) -> bool:
    if (entry.get("parse_status") != "PARSED_UNIQUE_KV"
            or entry.get("line_truncated") is not False
            or type(entry.get("sanitized_line")) is not str):
        return False
    fields = _entry_fields(entry)
    return (
        set(fields) == AUDIT_MOUNT_ALLOWED_FIELDS
        and fields.get("apparmor") == "DENIED"
        and fields.get("profile") == BOOTSTRAP_PROFILE
        and fields.get("operation") == "mount"
        and fields.get("class") == "mount"
        and fields.get("info") == "failed mntpnt match"
        and fields.get("error") == "-13"
        and fields.get("comm") == "bwrap"
        and fields.get("fstype") == "proc"
        and fields.get("name") == "/newroot/proc/"
        and fields.get("srcname") == "proc"
        and type(fields.get("pid")) is str and fields["pid"].isdigit()
        and int(fields["pid"]) > 0
        and _mount_flags_are_discovery_safe(fields.get("flags"))
        and RUNTIME_PROFILE not in entry["sanitized_line"]
        and not any(token in entry["sanitized_line"] for token in AUDIT_FORBIDDEN_TOKENS)
    )


def _empty_audit(anchor: Mapping[str, Any], boot_after: str, query_argv: list[str] | None) -> dict[str, Any]:
    return {
        "capture_valid": False,
        "capture_errors": [],
        "boot_id_before": anchor.get("boot_id"),
        "boot_id_after": boot_after,
        "start_cursor": anchor.get("cursor"),
        "end_cursor": None,
        "journal_anchor_argv": anchor.get("journal_anchor_argv"),
        "journal_query_argv": query_argv,
        "raw_stdout_sha256": sha256_bytes(b""),
        "raw_stdout_size_bytes": 0,
        "storage_ceiling": AUDIT_STORAGE_CEILING,
        "matching_total": 0,
        "stored_count": 0,
        "dropped_count": 0,
        "storage_overflow": False,
        "expected_mount_total": 0,
        "mkdir_total": 0,
        "unexpected_total": 0,
        "sanitized_denials": [],
        "expected_mount_denials": [],
        "unexpected_denials": [],
    }


def capture_apparmor_denials(anchor: Mapping[str, Any]) -> dict[str, Any]:
    boot_after = read_boot_id()
    boot_before = anchor.get("boot_id")
    cursor = anchor.get("cursor")
    if (type(boot_before) is not str or BOOT_ID_RE.fullmatch(boot_before) is None
            or type(cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(cursor) is None):
        observed = _empty_audit(anchor, boot_after, None)
        observed["capture_errors"] = ["INVALID_PRE_RUN_ANCHOR"]
        return observed
    if boot_after != boot_before:
        observed = _empty_audit(anchor, boot_after, None)
        observed["capture_errors"] = ["BOOT_ID_CHANGED"]
        return observed
    compact_boot_id = boot_before.replace("-", "")
    argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
        "--output=short-iso-precise", f"--boot={compact_boot_id}",
        f"--after-cursor={cursor}", "--show-cursor",
    ]
    observed = _empty_audit(anchor, boot_after, argv)
    result = run_bounded_command(
        argv,
        stdout_limit=AUDIT_QUERY_STDOUT_LIMIT,
        stderr_limit=AUDIT_QUERY_STDERR_LIMIT,
        timeout_seconds=15,
    )
    raw = result.get("stdout") if type(result.get("stdout")) is bytes else b""
    observed["raw_stdout_sha256"] = sha256_bytes(raw)
    observed["raw_stdout_size_bytes"] = len(raw)
    errors: list[str] = []
    if type(result.get("returncode")) is not int or result.get("returncode") != 0:
        errors.append("JOURNAL_QUERY_RETURN_CODE")
    if result.get("failure") is not None:
        errors.append("JOURNAL_QUERY_BOUNDED_FAILURE")
    if result.get("stderr") != b"":
        errors.append("JOURNAL_QUERY_STDERR")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError:
        observed["capture_errors"] = errors + ["JOURNAL_QUERY_INVALID_UTF8"]
        return observed
    lines = text.splitlines()
    cursor_indices = [index for index, line in enumerate(lines) if line.startswith("-- cursor: ")]
    if len(cursor_indices) != 1 or cursor_indices[0] != len(lines) - 1:
        observed["capture_errors"] = errors + ["END_CURSOR_FRAMING"]
        return observed
    end_cursor = lines[-1][11:]
    if (JOURNAL_CURSOR_RE.fullmatch(end_cursor) is None
            or not _cursor_matches_boot(end_cursor, boot_before)):
        observed["capture_errors"] = errors + ["END_CURSOR_INVALID_OR_STALE"]
        return observed
    observed["end_cursor"] = end_cursor
    sanitized: list[dict[str, Any]] = []
    expected: list[dict[str, Any]] = []
    unexpected: list[dict[str, Any]] = []
    matching_total = expected_total = mkdir_total = unexpected_total = 0
    for line in lines[:-1]:
        if not any(label in line for label in LABELS) or "apparmor" not in line or "DENIED" not in line:
            continue
        matching_total += 1
        entry = _parse_audit_line(line)
        if len(sanitized) < AUDIT_STORAGE_CEILING:
            sanitized.append(entry)
        fields = _entry_fields(entry)
        if fields.get("operation") == "mkdir":
            mkdir_total += 1
        if _is_exact_mount_discovery(entry):
            expected_total += 1
            if len(expected) < AUDIT_STORAGE_CEILING:
                expected.append(entry)
        else:
            unexpected_total += 1
            if len(unexpected) < AUDIT_STORAGE_CEILING:
                unexpected.append(entry)
    stored_count = len(sanitized)
    observed.update({
        "capture_valid": not errors,
        "capture_errors": errors,
        "matching_total": matching_total,
        "stored_count": stored_count,
        "dropped_count": matching_total - stored_count,
        "storage_overflow": matching_total > stored_count,
        "expected_mount_total": expected_total,
        "mkdir_total": mkdir_total,
        "unexpected_total": unexpected_total,
        "sanitized_denials": sanitized,
        "expected_mount_denials": expected,
        "unexpected_denials": unexpected,
    })
    return observed


def gate_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups",
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--no-new-privs", "--",
        "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["gate"]), "internal-run",
    ]


def create_root_admission_pipe() -> tuple[int, dict[str, Any]]:
    """Create a root-owned, single-consumer anonymous-FD capability."""

    if os.geteuid() != 0:
        raise SupervisorError("admission capability creation requires euid 0")
    read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
    try:
        for descriptor in (read_fd, write_fd):
            metadata = os.fstat(descriptor)
            if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                    or stat.S_IMODE(metadata.st_mode) != 0o600):
                raise SupervisorError("root admission pipe metadata differs")
        nonce = os.getrandom(32)
        view = memoryview(nonce)
        while view:
            count = os.write(write_fd, view)
            if count <= 0:
                raise SupervisorError("short root admission capability write")
            view = view[count:]
        digest = sha256_bytes(nonce)
    except BaseException:
        os.close(read_fd)
        raise
    finally:
        os.close(write_fd)
    return read_fd, {
        "transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ",
        "size_bytes": 32,
        "sha256": digest,
        "pipe_uid": 0,
        "pipe_gid": 0,
        "pipe_mode": "0600",
        "public_environment_token_is_not_authority": True,
    }


def _denial_stderr_envelope(stderr: Any) -> dict[str, Any]:
    raw = stderr if type(stderr) is bytes else b""
    evidence = {
        "valid": False,
        "sha256": sha256_bytes(raw),
        "size_bytes": len(raw),
        "utf8": None,
        "contract": "EXACT_BWRAP_PROC_MOUNT_PERMISSION_DENIED_LINE",
    }
    if type(stderr) is not bytes or not (1 <= len(raw) <= 512):
        return evidence
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError:
        return evidence
    evidence["utf8"] = text
    if text != "bwrap: Can't mount proc on /proc: Permission denied\n":
        return evidence
    evidence["valid"] = True
    return evidence


def _audit_is_closed_success_window(audit: Mapping[str, Any]) -> bool:
    count_keys = (
        "storage_ceiling", "matching_total", "stored_count", "dropped_count",
        "expected_mount_total", "mkdir_total", "unexpected_total", "raw_stdout_size_bytes",
    )
    if not all(type(audit.get(key)) is int for key in count_keys):
        return False
    if (audit.get("capture_valid") is not True or audit.get("capture_errors") != []
            or audit.get("storage_ceiling") != AUDIT_STORAGE_CEILING
            or audit.get("matching_total") != 0
            or audit.get("stored_count") != 0
            or audit.get("dropped_count") != 0
            or audit.get("storage_overflow") is not False
            or audit.get("expected_mount_total") != 0
            or audit.get("mkdir_total") != 0
            or audit.get("unexpected_total") != 0
            or audit.get("raw_stdout_size_bytes") < 1
            or type(audit.get("raw_stdout_sha256")) is not str
            or re.fullmatch(r"[0-9a-f]{64}", audit["raw_stdout_sha256"]) is None):
        return False
    boot_before, boot_after = audit.get("boot_id_before"), audit.get("boot_id_after")
    start_cursor, end_cursor = audit.get("start_cursor"), audit.get("end_cursor")
    if (type(boot_before) is not str or BOOT_ID_RE.fullmatch(boot_before) is None
            or boot_after != boot_before
            or type(start_cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(start_cursor) is None
            or type(end_cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(end_cursor) is None
            or not _cursor_matches_boot(start_cursor, boot_before)
            or not _cursor_matches_boot(end_cursor, boot_before)):
        return False
    compact_boot_id = boot_before.replace("-", "")
    expected_anchor_argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet", f"--boot={compact_boot_id}",
        "--lines=0", "--show-cursor",
    ]
    expected_query_argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
        "--output=short-iso-precise", f"--boot={compact_boot_id}",
        f"--after-cursor={start_cursor}", "--show-cursor",
    ]
    if (audit.get("journal_anchor_argv") != expected_anchor_argv
            or audit.get("journal_query_argv") != expected_query_argv):
        return False
    started, ended = audit.get("run_started_epoch"), audit.get("run_ended_epoch")
    if (type(started) is not float or type(ended) is not float
            or started <= 0.0 or ended < started):
        return False
    cleanup = audit.get("prequery_label_cleanup")
    if (type(cleanup) is not dict or cleanup.get("initial") != []
            or cleanup.get("term_sent") != [] or cleanup.get("after_term") != []
            or cleanup.get("kill_sent") != []
            or cleanup.get("stable_zero_scans") != [[], [], []]):
        return False
    profile_observation = audit.get("profiles_before_audit_query")
    if (type(profile_observation) is not dict
            or profile_observation.get("kernel_exact_counts") != {label: 1 for label in LABELS}
            or profile_observation.get("kernel_exact_enforce") != {label: True for label in LABELS}
            or profile_observation.get("aa_status_exact_presence") != {label: True for label in LABELS}
            or profile_observation.get("aa_status_exact_modes") != {label: "enforce" for label in LABELS}):
        return False
    empty_sha256 = sha256_bytes(b"")
    expected_sync = {
        "argv": ["/usr/bin/journalctl", "--sync"],
        "returncode": 0,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": 0,
        "stderr_sha256": empty_sha256,
        "failure": None,
        "start_new_session": True,
    }
    if audit.get("journal_sync") != expected_sync:
        return False
    postquery_profile = audit.get("profiles_after_audit_query")
    if (audit.get("postquery_stable_zero_labels") != [[], [], []]
            or type(postquery_profile) is not dict
            or postquery_profile.get("kernel_exact_counts") != {label: 1 for label in LABELS}
            or postquery_profile.get("kernel_exact_enforce") != {label: True for label in LABELS}
            or postquery_profile.get("aa_status_exact_presence") != {label: True for label in LABELS}
            or postquery_profile.get("aa_status_exact_modes") != {label: "enforce" for label in LABELS}):
        return False
    sanitized = audit.get("sanitized_denials")
    expected = audit.get("expected_mount_denials")
    unexpected = audit.get("unexpected_denials")
    if sanitized != [] or expected != [] or unexpected != []:
        return False
    return True


def classify_execution(result: Mapping[str, Any], audit: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    stdout, stderr = result.get("stdout"), result.get("stderr")
    evidence = _command_evidence(result, include_prefix=True)
    success_frame: dict[str, Any] | None = None
    if type(stdout) is bytes:
        try:
            success_frame = parse_success_frame(stdout)
        except SupervisorError:
            pass
    evidence["success_frame"] = success_frame
    returncode = result.get("returncode")
    exact_success_window = _audit_is_closed_success_window(audit)
    if (result.get("failure") is None and type(returncode) is int and returncode == 0
            and stderr == b"" and success_frame is not None and exact_success_window):
        return "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V11_CLEANUP_PENDING", evidence
    # Revision v11 promotes only a byte-exact success frame inside a boot/cursor
    # bound window with zero fresh denials for either unique v11 label.
    return "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING", evidence


def sudo_timestamp_argvs() -> tuple[list[str], list[str]]:
    prefix = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27",
        "--inh-caps=-all", "--ambient-caps=-all", "--",
    ]
    argvs = (
        prefix + ["/usr/bin/sudo", "-K"],
        prefix + ["/usr/bin/sudo", "-n", "/usr/bin/true"],
    )
    validate_sudo_cleanup_argv_contract(argvs[0], ["/usr/bin/sudo", "-K"])
    validate_sudo_cleanup_argv_contract(argvs[1], ["/usr/bin/sudo", "-n", "/usr/bin/true"])
    return argvs


def validate_sudo_cleanup_argv_contract(argv: list[str], command_tail: list[str]) -> None:
    expected_prefix = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27",
        "--inh-caps=-all", "--ambient-caps=-all", "--",
    ]
    if type(argv) is not list or any(type(token) is not str for token in argv):
        raise SupervisorError("sudo cleanup argv type differs")
    if command_tail not in (["/usr/bin/sudo", "-K"], ["/usr/bin/sudo", "-n", "/usr/bin/true"]):
        raise SupervisorError("sudo cleanup inner command differs")
    if argv != expected_prefix + command_tail:
        raise SupervisorError("sudo cleanup argv differs or contains injected arguments")
    if any(token == "--bounding-set" or token.startswith("--bounding-set=") for token in argv):
        raise SupervisorError("sudo cleanup must preserve the host bounding set without a bounding argv token")
    if argv.count("--") != 1 or any(token in argv for token in ("--clear-groups", "--init-groups", "--keep-groups", "--no-new-privs")):
        raise SupervisorError("sudo cleanup group, delimiter, or NNP argv contract differs")


def verify_sudo_group_membership() -> dict[str, Any]:
    raw = read_regular_bytes(SUDO_GROUP_PATH, limit=1 << 20)
    metadata = os.stat(SUDO_GROUP_PATH, follow_symlinks=False)
    if (metadata.st_uid != 0 or metadata.st_gid != 0
            or stat.S_IMODE(metadata.st_mode) != 0o644):
        raise SupervisorError("/etc/group owner or mode differs from sudo cleanup contract")
    try:
        lines = raw.decode("utf-8", "strict").splitlines()
    except UnicodeError as exc:
        raise SupervisorError("/etc/group is not strict UTF-8") from exc
    sudo_or_gid_records: list[str] = []
    for line in lines:
        fields = line.split(":")
        if len(fields) != 4:
            raise SupervisorError("/etc/group contains a malformed record")
        if fields[0] == "sudo" or fields[2] == str(SUDO_GROUP_GID):
            sudo_or_gid_records.append(line)
    if sudo_or_gid_records != [SUDO_GROUP_RECORD]:
        raise SupervisorError("exact sudo:x:27:zrj membership record differs")
    return dict(SUDO_GROUP_MEMBERSHIP_CONTRACT)


def clear_invoking_user_sudo_timestamp() -> dict[str, Any]:
    # This helper is also the fail-safe for errors that happen before the
    # ordinary trusted-tool preflight completes.  Verify every executable it
    # may launch before inspecting the PTY or spawning a cleanup command.
    for name in ("setpriv", "sudo", "true"):
        require_tool(name)
    membership = verify_sudo_group_membership()
    if not os.isatty(0):
        raise SupervisorError("sudo timestamp cleanup requires the original invoking PTY on fd0")
    tty = {"stdin_isatty": True}
    try:
        metadata = os.fstat(0)
        foreground_pgrp = os.tcgetpgrp(0)
        current_pgrp = os.getpgrp()
        if foreground_pgrp != current_pgrp:
            raise SupervisorError("sudo timestamp cleanup requires the supervisor foreground process group")
        tty.update({
            "stdin_device": metadata.st_dev,
            "stdin_inode": metadata.st_ino,
            "stdin_rdev": metadata.st_rdev,
            "session_id": os.getsid(0),
            "process_group": current_pgrp,
            "foreground_process_group": foreground_pgrp,
        })
    except OSError as exc:
        raise SupervisorError("cannot capture inherited PTY identity") from exc
    clear_argv, verify_argv = sudo_timestamp_argvs()
    validate_sudo_cleanup_argv_contract(clear_argv, ["/usr/bin/sudo", "-K"])
    validate_sudo_cleanup_argv_contract(verify_argv, ["/usr/bin/sudo", "-n", "/usr/bin/true"])
    clear = run_bounded_command(clear_argv, stdin_bytes=None, stdout_limit=4096, stderr_limit=4096, timeout_seconds=15, start_new_session=False)
    verify = run_bounded_command(verify_argv, stdin_bytes=None, stdout_limit=4096, stderr_limit=4096, timeout_seconds=15, start_new_session=False)
    if (clear["argv"] != clear_argv or clear["returncode"] != 0 or isinstance(clear["returncode"], bool)
            or clear["failure"] is not None or clear["start_new_session"] is not False
            or clear["stdout"] or clear["stderr"]):
        raise SupervisorError("UID1000 sudo -K all-timestamp invalidation failed")
    if (verify["argv"] != verify_argv or verify["returncode"] != 1 or isinstance(verify["returncode"], bool)
            or verify["failure"] is not None or verify["start_new_session"] is not False
            or verify["stdout"] or verify["stderr"] != b"sudo: a password is required\n"):
        raise SupervisorError("sudo all-timestamp invalidation was not proven by -n true")
    return {
        "scope": "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE",
        "bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "pty": tty,
        "membership": membership,
        "identity_contract": dict(SUDO_CLEANUP_IDENTITY_CONTRACT),
        "clear": _command_evidence(clear),
        "noninteractive_true_must_fail": _command_evidence(verify, include_prefix=True),
    }


def validate_sudo_cleanup_evidence(sudo_clear: Mapping[str, Any]) -> None:
    expected_keys = {
        "scope", "bounding_mode", "pty", "membership", "identity_contract", "clear",
        "noninteractive_true_must_fail",
    }
    if type(sudo_clear) is not dict or set(sudo_clear) != expected_keys:
        raise SupervisorError("sudo timestamp cleanup evidence key set differs")
    clear_argv, verify_argv = sudo_timestamp_argvs()
    empty_sha256 = sha256_bytes(b"")
    expected_clear = {
        "argv": clear_argv,
        "returncode": 0,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": 0,
        "stderr_sha256": empty_sha256,
        "failure": None,
        "start_new_session": False,
    }
    password_stderr = b"sudo: a password is required\n"
    expected_verify = {
        "argv": verify_argv,
        "returncode": 1,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": len(password_stderr),
        "stderr_sha256": sha256_bytes(password_stderr),
        "failure": None,
        "start_new_session": False,
        "stderr_utf8_prefix": password_stderr.decode("ascii"),
    }
    pty = sudo_clear["pty"]
    pty_keys = {
        "stdin_isatty", "stdin_device", "stdin_inode", "stdin_rdev", "session_id",
        "process_group", "foreground_process_group",
    }
    if (sudo_clear["scope"] != "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE"
            or sudo_clear["bounding_mode"] != SUDO_CLEANUP_BOUNDING_MODE
            or not _json_equal(sudo_clear["membership"], SUDO_GROUP_MEMBERSHIP_CONTRACT)
            or not _json_equal(sudo_clear["identity_contract"], SUDO_CLEANUP_IDENTITY_CONTRACT)
            or not _json_equal(sudo_clear["clear"], expected_clear)
            or not _json_equal(sudo_clear["noninteractive_true_must_fail"], expected_verify)
            or type(pty) is not dict or set(pty) != pty_keys
            or pty["stdin_isatty"] is not True
            or any(type(pty[key]) is not int for key in pty_keys - {"stdin_isatty"})
            or any(pty[key] < 0 for key in {"stdin_device", "stdin_inode", "stdin_rdev"})
            or pty["session_id"] < 1 or pty["process_group"] < 1
            or pty["foreground_process_group"] != pty["process_group"]):
        raise SupervisorError("sudo timestamp cleanup evidence differs")


def execution_receipt_document(
    *,
    status: str,
    policy_sha256: str,
    snapshot: Mapping[str, Any],
    tools: Mapping[str, Any],
    sysctls_before: Mapping[str, Any],
    profiles_before: Mapping[str, Any],
    profiles_loaded: Mapping[str, Any] | None,
    gate_evidence: Mapping[str, Any] | None,
    admission_capability: Mapping[str, Any] | None,
    audit: Mapping[str, Any] | None,
    error: str | None,
) -> dict[str, Any]:
    if not status.endswith("CLEANUP_PENDING"):
        raise SupervisorError("execution receipt must remain cleanup-pending")
    return {
        "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_EXECUTION_RECEIPT",
        "probe_id": PROBE_ID,
        "status": status,
        "cleanup_complete": False,
        "production_authorized": False,
        "gencase_or_solver_executed": False,
        "host_writable_mount_used": False,
        "network_or_device_exposed": False,
        "policy_sha256": policy_sha256,
        "snapshot": dict(snapshot),
        "trusted_tools": dict(tools),
        "sysctls_before": dict(sysctls_before),
        "profiles_before": dict(profiles_before),
        "profiles_loaded": None if profiles_loaded is None else dict(profiles_loaded),
        "gate": None if gate_evidence is None else dict(gate_evidence),
        "root_admission_capability": None if admission_capability is None else dict(admission_capability),
        "apparmor_audit": None if audit is None else dict(audit),
        "error": error,
        "next_required_evidence": str(LIFECYCLE_RECEIPT),
    }


def lifecycle_document(
    *,
    execution_receipt: Mapping[str, Any],
    cleanup: Mapping[str, Any],
    profiles_after: Mapping[str, Any],
    sysctls_before: Mapping[str, Any],
    sysctls_after: Mapping[str, Any],
    sudo_clear: Mapping[str, Any],
) -> dict[str, Any]:
    stable = cleanup.get("stable_zero_scans")
    post_unload_stable = cleanup.get("post_unload_stable_zero_scans")
    if (cleanup.get("initial") != [] or cleanup.get("term_sent") != []
            or cleanup.get("after_term") != [] or cleanup.get("kill_sent") != []
            or stable != [[], [], []] or post_unload_stable != [[], [], []]):
        raise SupervisorError("lifecycle requires exact pre- and post-unload all-task empty label scans")
    require_profile_counts(profiles_after, 0)
    if sysctls_before != sysctls_after:
        raise SupervisorError("host sysctls changed across probe lifecycle")
    validate_sudo_cleanup_evidence(sudo_clear)
    return {
        "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_LIFECYCLE_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "PASS_V11_PROFILE_PROCESS_SYSCTL_SUDO_LIFECYCLE_CLEANUP",
        "execution_receipt": dict(execution_receipt),
        "cleanup": dict(cleanup),
        "profiles_after": dict(profiles_after),
        "sysctls": {"before": dict(sysctls_before), "after": dict(sysctls_after), "unchanged": True},
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp": dict(sudo_clear),
        "production_authorized": False,
        "next_allowed_stage": "REVIEW_THIS_SINGLE_ATTEMPT_RECEIPT_AND_CREATE_FRESH_SUCCESSOR_ONLY",
    }


def run_once(*, policy_sha256: str, admission_token: str) -> dict[str, Any]:
    with TerminationGuard() as termination_guard:
        try:
            return _run_once_guarded(
                policy_sha256=policy_sha256,
                admission_token=admission_token,
                termination_guard=termination_guard,
            )
        except BaseException as exc:
            if termination_guard.cleanup_started:
                raise
            primary_error = f"{type(exc).__name__}: {exc}"[:4096]
            if not termination_guard.preflight_sudo_cleanup_attempted:
                termination_guard.preflight_sudo_cleanup_attempted = True
                try:
                    sudo_evidence = clear_invoking_user_sudo_timestamp()
                    validate_sudo_cleanup_evidence(sudo_evidence)
                    termination_guard.preflight_sudo_cleanup_evidence = dict(sudo_evidence)
                except BaseException as cleanup_exc:
                    termination_guard.preflight_sudo_cleanup_error = (
                        f"{type(cleanup_exc).__name__}: {cleanup_exc}"[:4096]
                    )
            termination_guard.begin_cleanup()
            sudo_clear = dict(termination_guard.preflight_sudo_cleanup_evidence)
            cleanup_error = termination_guard.preflight_sudo_cleanup_error
            cleanup_proven = cleanup_error is None and bool(sudo_clear)
            document = {
                "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_PREFLIGHT_FAILURE_RECEIPT",
                "probe_id": PROBE_ID,
                "status": (
                    "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANED_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED"
                    if cleanup_proven else
                    "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANUP_INCOMPLETE_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED"
                ),
                "requested_policy_sha256": policy_sha256,
                "primary_error": primary_error,
                "start_receipt_created_by_this_run": False,
                "parser_invoked_by_this_run": False,
                "profile_load_attempted_by_this_run": False,
                "probe_executed_by_this_run": False,
                "sudo_timestamp_cleanup_attempted": termination_guard.preflight_sudo_cleanup_attempted,
                "sudo_timestamp_cleanup_proven": cleanup_proven,
                "sudo_timestamp_cleanup_error": cleanup_error,
                "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
                "sudo_timestamp_observation": sudo_clear,
                "identity_consumed": True,
                "production_authorized": False,
            }
            try:
                record = write_json_new(PREFLIGHT_FAILURE_RECEIPT, document)
            except BaseException as receipt_exc:
                receipt_error = f"{type(receipt_exc).__name__}: {receipt_exc}"[:4096]
                raise SupervisorError(
                    "pre-start failure and preflight receipt incomplete; "
                    f"primary={primary_error}; sudo_cleanup={cleanup_error}; receipt={receipt_error}"
                ) from exc
            cleanup_status = "proven" if cleanup_proven else "incomplete"
            raise SupervisorError(f"pre-start failure; sudo timestamp cleanup {cleanup_status}; preserved receipt {record['path']}") from exc


def _run_once_guarded(
    *,
    policy_sha256: str,
    admission_token: str,
    termination_guard: TerminationGuard,
) -> dict[str, Any]:
    termination_guard.checkpoint()
    if admission_token != ADMISSION_TOKEN:
        raise SupervisorError("explicit one-shot admission token differs")
    policy, snapshot = verify_snapshot(policy_sha256)
    _assert_one_shot_paths_absent()
    if policy.get("authorization", {}).get("attempts_per_identity") != 1:
        raise SupervisorError("one-shot policy differs")
    tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    termination_guard.preflight_sudo_cleanup_attempted = True
    try:
        sudo_admission = clear_invoking_user_sudo_timestamp()
        validate_sudo_cleanup_evidence(sudo_admission)
        termination_guard.preflight_sudo_cleanup_evidence = dict(sudo_admission)
    except BaseException as exc:
        termination_guard.preflight_sudo_cleanup_error = f"{type(exc).__name__}: {exc}"[:4096]
        raise
    termination_guard.checkpoint()
    sysctls_before = read_sysctls()
    if policy.get("process_lifecycle", {}).get("expected_sysctls") != sysctls_before:
        raise SupervisorError("pre-attempt sysctls differ from the frozen policy baseline")
    profiles_before = profile_state()
    require_profile_counts(profiles_before, 0)
    initial_zero = require_stable_zero_labels()
    audit_anchor = capture_journal_anchor()
    start_document = {
        "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_START_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "ONE_SHOT_STARTED_CLEANUP_REQUIRED",
        "policy_sha256": policy_sha256,
        "snapshot": snapshot,
        "profiles_before": profiles_before,
        "initial_stable_zero_labels": initial_zero,
        "journal_anchor": audit_anchor,
        "sysctls_before": sysctls_before,
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp_admission": sudo_admission,
    }
    start_record = write_json_new(START_RECEIPT, start_document)
    load_attempted = False
    profiles_loaded: dict[str, Any] | None = None
    admission_capability: dict[str, Any] | None = None
    execution_record: dict[str, Any] | None = None
    execution_status = "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    primary_error: str | None = None
    cleanup_errors: list[str] = []
    cleanup: dict[str, Any] = {"stable_zero_scans": None}
    profiles_after: dict[str, Any] = {
        "kernel_exact_counts": {label: -1 for label in LABELS},
        "aa_status_exact_presence": {label: True for label in LABELS},
        "aa_status_exact_modes": {label: "unknown" for label in LABELS},
    }
    sudo_clear: dict[str, Any] = {}
    sysctls_after: dict[str, Any] = {}
    try:
        termination_guard.checkpoint()
        profile_path = str(SNAPSHOT_PATHS["profile"])
        run_checked(["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
        termination_guard.checkpoint()
        load_attempted = True
        run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
        termination_guard.checkpoint()
        profiles_loaded = profile_state()
        require_profile_counts(profiles_loaded, 1)
        termination_guard.checkpoint()
        started = time.time()
        gate_env = {
            "HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC",
            SNAPSHOT_ENV: str(SNAPSHOT_ROOT), ADMISSION_ENV: ADMISSION_TOKEN,
        }
        termination_guard.checkpoint()
        admission_fd, admission_capability = create_root_admission_pipe()
        gate_env[ADMISSION_FD_ENV] = str(admission_fd)
        gate_env[ADMISSION_SHA256_ENV] = admission_capability["sha256"]
        try:
            gate_result = run_bounded_command(
                gate_handoff_argv(),
                stdout_limit=MAX_SUCCESS_FRAME_BYTES,
                stderr_limit=8192,
                timeout_seconds=25,
                env=gate_env,
                pass_fds=(admission_fd,),
            )
        finally:
            os.close(admission_fd)
        termination_guard.checkpoint()
        ended = time.time()
        prequery_label_cleanup = terminate_labeled_processes()
        if (prequery_label_cleanup.get("initial") != []
                or prequery_label_cleanup.get("term_sent") != []
                or prequery_label_cleanup.get("after_term") != []
                or prequery_label_cleanup.get("kill_sent") != []
                or prequery_label_cleanup.get("stable_zero_scans") != [[], [], []]):
            raise SupervisorError("pre-query label cleanup found residue or required a signal")
        profiles_before_audit_query = profile_state()
        require_profile_counts(profiles_before_audit_query, 1)
        journal_sync_result = run_checked(
            ["/usr/bin/journalctl", "--sync"], timeout_seconds=15,
            stdout_limit=4_096, stderr_limit=4_096, require_silent=True,
        )
        audit = capture_apparmor_denials(audit_anchor)
        audit["run_started_epoch"] = started
        audit["run_ended_epoch"] = ended
        audit["prequery_label_cleanup"] = prequery_label_cleanup
        audit["profiles_before_audit_query"] = profiles_before_audit_query
        audit["journal_sync"] = _command_evidence(journal_sync_result)
        audit["postquery_stable_zero_labels"] = require_stable_zero_labels()
        audit["profiles_after_audit_query"] = profile_state()
        require_profile_counts(audit["profiles_after_audit_query"], 1)
        termination_guard.checkpoint()
        execution_status, gate_evidence = classify_execution(gate_result, audit)
        document = execution_receipt_document(
            status=execution_status,
            policy_sha256=policy_sha256,
            snapshot=snapshot,
            tools=tools,
            sysctls_before=sysctls_before,
            profiles_before=profiles_before,
            profiles_loaded=profiles_loaded,
            gate_evidence=gate_evidence,
            admission_capability=admission_capability,
            audit=audit,
            error=None,
        )
        execution_record = write_json_new(EXECUTION_RECEIPT, document)
    except BaseException as exc:
        primary_error = f"{type(exc).__name__}: {exc}"[:4096]
        try:
            document = execution_receipt_document(
                status="ATTEMPT_ABORTED_CLEANUP_PENDING",
                policy_sha256=policy_sha256,
                snapshot=snapshot,
                tools=tools,
                sysctls_before=sysctls_before,
                profiles_before=profiles_before,
                profiles_loaded=profiles_loaded,
                gate_evidence=None,
                admission_capability=admission_capability,
                audit=None,
                error=primary_error,
            )
            execution_record = write_json_new(EXECUTION_RECEIPT, document)
            execution_status = document["status"]
        except BaseException as receipt_exc:
            cleanup_errors.append(f"execution_receipt: {type(receipt_exc).__name__}: {receipt_exc}"[:4096])
    finally:
        termination_guard.begin_cleanup()
        try:
            cleanup = terminate_labeled_processes()
        except BaseException as exc:
            cleanup_errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
        pre_unload_zero_proven = cleanup.get("stable_zero_scans") == [[], [], []]
        if not pre_unload_zero_proven:
            try:
                fallback_zero = require_stable_zero_labels()
                cleanup = {**cleanup, "stable_zero_scans": fallback_zero}
                pre_unload_zero_proven = True
            except BaseException as exc:
                cleanup_errors.append(f"pre_unload_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            if not pre_unload_zero_proven:
                raise SupervisorError("profile unload forbidden until all-task label zero is proven")
            current_counts = read_kernel_profile_counts()
            if load_attempted or any(current_counts.get(label, 0) for label in LABELS):
                unload = run_bounded_command(["/usr/sbin/apparmor_parser", "-R", "-K", str(SNAPSHOT_PATHS["profile"])], stdout_limit=65_536, stderr_limit=65_536, timeout_seconds=15)
                if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                    raise SupervisorError("profile unload failed")
            profiles_after = profile_state()
            require_profile_counts(profiles_after, 0)
        except BaseException as exc:
            cleanup_errors.append(f"profile_cleanup: {type(exc).__name__}: {exc}"[:4096])
        try:
            final_zero = require_stable_zero_labels()
            if cleanup.get("stable_zero_scans") != [[], [], []]:
                raise SupervisorError("cleanup label scans differ")
            cleanup = {**cleanup, "post_unload_stable_zero_scans": final_zero}
        except BaseException as exc:
            cleanup_errors.append(f"final_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            sudo_clear = clear_invoking_user_sudo_timestamp()
        except BaseException as exc:
            cleanup_errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])
        try:
            sysctls_after = read_sysctls()
            if sysctls_after != sysctls_before:
                raise SupervisorError("host sysctls changed")
        except BaseException as exc:
            cleanup_errors.append(f"sysctl_postcondition: {type(exc).__name__}: {exc}"[:4096])
        cleanup = {**cleanup, "termination_signals": list(termination_guard.received)}

    if execution_record is not None and not cleanup_errors:
        try:
            lifecycle = lifecycle_document(
                execution_receipt=execution_record,
                cleanup=cleanup,
                profiles_after=profiles_after,
                sysctls_before=sysctls_before,
                sysctls_after=sysctls_after,
                sudo_clear=sudo_clear,
            )
            lifecycle_record = write_json_new(LIFECYCLE_RECEIPT, lifecycle)
            return {"execution_status": execution_status, "execution_receipt": execution_record, "lifecycle_receipt": lifecycle_record, "primary_error": primary_error, "cleanup_errors": []}
        except BaseException as exc:
            cleanup_errors.append(f"lifecycle_receipt: {type(exc).__name__}: {exc}"[:4096])

    failure = {
        "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_LIFECYCLE_INCOMPLETE_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "LIFECYCLE_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        "start_receipt": start_record,
        "execution_receipt": execution_record,
        "execution_status": execution_status,
        "primary_error": primary_error,
        "cleanup_errors": cleanup_errors,
        "cleanup_observation": cleanup,
        "profiles_after_observation": profiles_after,
        "sysctls_after_observation": sysctls_after,
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp_observation": sudo_clear,
        "production_authorized": False,
    }
    failure_record = write_json_new(LIFECYCLE_FAILURE_RECEIPT, failure)
    raise SupervisorError(f"v11 lifecycle incomplete; preserved receipt {failure_record['path']}")


def recover_only(*, policy_sha256: str, recovery_token: str) -> dict[str, Any]:
    """Idempotent cleanup path that can never create an admission FD or run gate."""

    if recovery_token != RECOVERY_TOKEN:
        raise SupervisorError("explicit cleanup-only recovery token differs")
    policy, snapshot = verify_snapshot(policy_sha256)
    try:
        recovery_receipt_preexisting = stat.S_ISREG(os.lstat(RECOVERY_RECEIPT).st_mode)
    except FileNotFoundError:
        recovery_receipt_preexisting = False
    errors: list[str] = []
    recovery_tools: dict[str, Any] = {}
    try:
        sysctls_before = read_sysctls()
        if policy.get("process_lifecycle", {}).get("expected_sysctls") != sysctls_before:
            errors.append("preexisting_sysctls_differ_from_frozen_baseline")
    except BaseException as exc:
        sysctls_before = {}
        errors.append(f"sysctls_before: {type(exc).__name__}: {exc}"[:4096])
    try:
        before_entries = read_kernel_profile_entries()
    except BaseException as exc:
        before_entries = {label: [] for label in LABELS}
        errors.append(f"kernel_profiles_before: {type(exc).__name__}: {exc}"[:4096])
    profiles_before = {"kernel_exact_lines": before_entries, "kernel_exact_counts": {label: len(lines) for label, lines in before_entries.items()}}
    if any(count not in (0, 1) for count in profiles_before["kernel_exact_counts"].values()):
        errors.append("impossible_duplicate_exact_profile_labels")
    with TerminationGuard() as termination_guard:
        termination_guard.begin_cleanup()
        try:
            cleanup = terminate_labeled_processes()
        except BaseException as exc:
            cleanup = {"stable_zero_scans": None}
            errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
        label_zero_proven = cleanup.get("stable_zero_scans") == [[], [], []]
        if not label_zero_proven:
            errors.append("pre_unload_all_task_label_zero_not_proven")
        try:
            current_entries = read_kernel_profile_entries()
            current_counts = {label: len(lines) for label, lines in current_entries.items()}
            profile_presence_unknown = False
        except BaseException as exc:
            current_entries = {label: [] for label in LABELS}
            current_counts = {label: -1 for label in LABELS}
            profile_presence_unknown = True
            errors.append(f"kernel_profiles_pre_unload: {type(exc).__name__}: {exc}"[:4096])
        unload_evidence: dict[str, Any] | None = None
        if profile_presence_unknown or any(current_counts.get(label, 0) for label in LABELS):
            if not label_zero_proven:
                errors.append("profile_unload_skipped_until_label_zero_is_proven")
            else:
                try:
                    recovery_tools["apparmor_parser"] = require_tool("apparmor_parser")
                    unload = run_bounded_command(
                        ["/usr/sbin/apparmor_parser", "-R", "-K", str(SNAPSHOT_PATHS["profile"])],
                        stdout_limit=65_536,
                        stderr_limit=65_536,
                        timeout_seconds=15,
                    )
                    if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                        raise SupervisorError("cleanup-only exact profile unload failed")
                    unload_evidence = _command_evidence(unload, include_prefix=True)
                except BaseException as exc:
                    errors.append(f"profile_unload: {type(exc).__name__}: {exc}"[:4096])
        try:
            after_entries = read_kernel_profile_entries()
        except BaseException as exc:
            after_entries = {label: ["UNKNOWN"] for label in LABELS}
            errors.append(f"kernel_profiles_after: {type(exc).__name__}: {exc}"[:4096])
        profiles_after_kernel = {"kernel_exact_lines": after_entries, "kernel_exact_counts": {label: len(lines) for label, lines in after_entries.items()}}
        if any(profiles_after_kernel["kernel_exact_counts"].values()):
            errors.append("exact_profiles_remain_after_recovery")
        try:
            post_unload_zero = require_stable_zero_labels()
        except BaseException as exc:
            post_unload_zero = []
            errors.append(f"post_unload_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            recovery_tools["aa_status"] = require_tool("aa_status")
            profiles_after = profile_state()
            require_profile_counts(profiles_after, 0)
        except BaseException as exc:
            profiles_after = profiles_after_kernel
            errors.append(f"profile_corroboration: {type(exc).__name__}: {exc}"[:4096])
        try:
            for name in ("setpriv", "sudo", "true"):
                recovery_tools[name] = require_tool(name)
            sudo_clear = clear_invoking_user_sudo_timestamp()
        except BaseException as exc:
            sudo_clear = {}
            errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])
        try:
            sysctls_after = read_sysctls()
            expected_sysctls = policy.get("process_lifecycle", {}).get("expected_sysctls")
            if sysctls_after != expected_sysctls or (sysctls_before and sysctls_after != sysctls_before):
                errors.append("sysctls_not_equal_to_frozen_baseline_after_recovery")
        except BaseException as exc:
            sysctls_after = {}
            errors.append(f"sysctls_after: {type(exc).__name__}: {exc}"[:4096])
        if errors:
            raise SupervisorError("cleanup-only recovery incomplete: " + " | ".join(errors))
        document = {
            "document_type": "SMPCC_R8_LIQUID_U3_STDIO_APPARMOR_PROBE_V11_RECOVERY_RECEIPT",
            "probe_id": PROBE_ID,
            "status": "PASS_V11_CLEANUP_ONLY_NO_PROBE_EXECUTED",
            "policy_sha256": policy_sha256,
            "snapshot": snapshot,
            "trusted_tools": recovery_tools,
            "cleanup": {**cleanup, "post_unload_stable_zero_scans": post_unload_zero},
            "profiles_before": profiles_before,
            "profile_unload": unload_evidence,
            "profiles_after": profiles_after,
            "sysctls": {"before": sysctls_before, "after": sysctls_after, "unchanged": True},
            "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
            "sudo_timestamp": sudo_clear,
            "termination_signals_ignored_during_cleanup": list(termination_guard.received),
            "probe_executed": False,
            "admission_fd_created": False,
            "production_authorized": False,
        }
        if recovery_receipt_preexisting:
            record = {
                "path": str(RECOVERY_RECEIPT),
                "preexisting_not_rewritten": True,
                "same_uid_parent_can_remove_or_replace_so_existing_bytes_are_not_retrusted": True,
            }
        else:
            record = write_json_new(RECOVERY_RECEIPT, document)
    return {"status": document["status"], "recovery_receipt": record}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("emit-bootstrap")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--policy-sha256", required=True)
    run_parser.add_argument("--admission-token", required=True)
    recover_parser = subparsers.add_parser("recover-only")
    recover_parser.add_argument("--policy-sha256", required=True)
    recover_parser.add_argument("--recovery-token", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = repository_static_review()
            print(json.dumps({"status": "PASS_V11_SUPERVISOR_STATIC_ONLY_EXECUTION_NOT_PERFORMED", "review": review, "snapshot_bootstrap": {"source_sha256": SNAPSHOT_BOOTSTRAP_SHA256, "source_size_bytes": len(SNAPSHOT_BOOTSTRAP_BYTES), "loader_sha256": SNAPSHOT_BOOTSTRAP_LOADER_SHA256, "loader_size_bytes": len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES), "manifest_arguments": snapshot_manifest_arguments(review)}}, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "emit-bootstrap":
            # Unprivileged delivery only. The root-side fixed -c loader verifies
            # exact length/hash and strict EOF before compiling these bytes.
            verify_snapshot_producer_identity()
            sys.stdout.buffer.write(SNAPSHOT_BOOTSTRAP_BYTES)
            sys.stdout.buffer.flush()
            return 0
        if arguments.command == "recover-only":
            result = recover_only(policy_sha256=arguments.policy_sha256, recovery_token=arguments.recovery_token)
            print(json.dumps({"status": "V11_CLEANUP_ONLY_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
            return 0
        result = run_once(policy_sha256=arguments.policy_sha256, admission_token=arguments.admission_token)
        print(json.dumps({"status": "V11_ONE_SHOT_LIFECYCLE_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
        return 0 if result["execution_status"] == "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V11_CLEANUP_PENDING" else 2
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        target = sys.stderr if arguments.command == "emit-bootstrap" else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=target)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
