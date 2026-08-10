#!/usr/bin/env python3
"""Static gate and unprivileged runner for the fresh harmless v11 exact proc-mount successor.

``self-check`` is read-only and never invokes a subprocess. ``internal-run``
is a supervisor-only entry point: it requires the exact root-owned snapshot,
UID/GID 1000, empty groups, zero capabilities, NNP=1, and an explicit
one-shot admission token.  It never writes a host file.
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
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
WORKSPACE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
SNAPSHOT_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_SNAPSHOT"
ADMISSION_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION"
ADMISSION_FD_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_STDIO_PROBE_V11_ADMISSION_SHA256"
PROBE_ID = "u3_stdio_apparmor_transport_probe_v11_20260808T052940Z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")
ADMISSION_TOKEN = f"{PROBE_ID}:single-harmless-attempt"
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
BOOTSTRAP_PROFILE = "r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z"
RUNTIME_PROFILE = "r8-liquid-u3-stdio-transport-runtime-v11-20260808t052940z"
HOST_UID = 1000
HOST_GID = 1000
SUDO_GROUP_GID = 27
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
    "path": "/etc/group",
    "owner": [0, 0],
    "mode": "0644",
    "record": "sudo:x:27:zrj",
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
TMPFS_BYTES = 67_108_864

POLICY_NAME = "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v11.json"
SCHEMA_NAME = "target_host_u3_stdio_apparmor_transport_probe_policy_v11.json"
PROFILE_NAME = "r8-liquid-u3-stdio-transport-probe-v11.profile"
SUPERVISOR_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_ROOT / "scripts" / SUPERVISOR_NAME
AA_STATUS_JSON_ARGV = ("/usr/sbin/aa-status", "--json")
FROZEN_RECEIPT_PATHS = {
    "start_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.start.json",
    "preflight_failure_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.preflight_incomplete.json",
    "execution_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.execution.json",
    "lifecycle_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.lifecycle.json",
    "lifecycle_failure_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.lifecycle_incomplete.json",
    "recovery_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.recovery.json",
}

if SNAPSHOT_ENV in os.environ:
    BASE = Path(os.environ[SNAPSHOT_ENV])
    POLICY_PATH = BASE / POLICY_NAME
    SCHEMA_PATH = BASE / SCHEMA_NAME
    PROFILE_PATH = BASE / PROFILE_NAME
    SUPERVISOR_PATH = BASE / SUPERVISOR_NAME
else:
    BASE = PACKAGE_DIR
    POLICY_PATH = BASE / "config/target_hosts" / POLICY_NAME
    SCHEMA_PATH = BASE / "schema" / SCHEMA_NAME
    PROFILE_PATH = BASE / "config/apparmor_drafts" / PROFILE_NAME
    SUPERVISOR_PATH = SCRIPT_PATH.with_name(SUPERVISOR_NAME)

STDIN_MAGIC = b"R8LQSTDIOPROBE11\x00"
SUCCESS_MAGIC = b"R8LQSTDIOPASSV11\x00"
MAX_STDIN_FRAME_BYTES = 65_536
MAX_SUCCESS_FRAME_BYTES = 16_384
MAX_CHILD_STDERR_BYTES = 4_096
OUTER_DEADLINE_SECONDS = 20

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "sleep": (Path("/usr/bin/sleep"), "a65efec857cfac0d4f43fc53affa73794ff1d1fdcb547931d4b92a98bda8e646"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

INPUT_PAYLOADS: dict[str, bytes] = {
    "probe_alpha.bin": b"R8_STDIO_PROBE_ALPHA_V11\n" * 11,
    "probe_beta.bin": bytes(range(256)) * 2 + b"BETA_V11",
    "probe_gamma.bin": hashlib.sha256(b"r8-stdio-probe-v11-gamma-seed").digest() * 31 + b"G",
}
INPUT_CONTRACT: dict[str, tuple[int, str]] = {
    name: (len(raw), hashlib.sha256(raw).hexdigest()) for name, raw in INPUT_PAYLOADS.items()
}

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

BASELINE_MOUNT_RULES = (
    "mount options=(rw, silent, rslave) -> /,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,',
    'mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,',
    'pivot_root oldroot=/tmp/oldroot/ /tmp/,',
    "mount options=(rw, silent, rprivate) -> /oldroot/,",
    'umount /oldroot/,',
    'pivot_root oldroot=/newroot/ /newroot/,',
    "umount /,",
    "mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,",
    "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,',
    'mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,',
)

EXPECTED_PROFILE_LINES = (
    f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "userns create,",
    "/usr/bin/bwrap rix,",
    "/usr/bin/python3.12 rix,",
    "/usr/bin/sleep rix,",
    "/ r,",
    "/usr/ r,",
    "/usr/lib/** mr,",
    "/usr/lib64/** mr,",
    "/lib/** mr,",
    "/lib64/** mr,",
    "/etc/ld.so.cache r,",
    "/etc/ld.so.conf r,",
    "/etc/ld.so.conf.d/** r,",
    "/work/ rw,",
    "/work/input/ rw,",
    "/work/input/probe_alpha.bin rw,",
    "/work/input/probe_beta.bin rw,",
    "/work/input/probe_gamma.bin rw,",
    "owner /proc/** r,",
    "owner /proc/*/uid_map w,",
    "owner /proc/*/gid_map w,",
    "owner /proc/*/setgroups w,",
    "/proc/filesystems r,",
    "/proc/sys/kernel/overflowuid r,",
    "/proc/sys/kernel/overflowgid r,",
    "/proc/sys/user/max_user_namespaces w,",
    *BASELINE_MOUNT_RULES,
    "/tmp/newroot/ rw,",
    "/tmp/newroot/** rw,",
    "/tmp/oldroot/ rw,",
    "/tmp/oldroot/** rw,",
    "/newroot/usr/ rw,",
    "/newroot/lib wl,",
    "/newroot/lib64 wl,",
    "/newroot/work/ rw,",
    "/newroot/proc/ rw,",
    "deny capability dac_override,",
    "capability sys_admin,",
    "capability sys_ptrace,",
    "capability sys_resource,",
    "capability setpcap,",
    "capability net_admin,",
    f"ptrace (read, readby) peer={BOOTSTRAP_PROFILE},",
    f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
    "signal (receive) set=(term,kill,exists) peer=unconfined,",
    "network unix dgram,",
    "network inet dgram,",
    "network inet6 dgram,",
    "network netlink raw,",
    "}",
    f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "/usr/bin/sleep rm,",
    "/usr/lib/** mr,",
    "/usr/lib64/** mr,",
    "/lib/** mr,",
    "/lib64/** mr,",
    "/etc/ld.so.cache r,",
    f"signal (receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
    "signal (receive) set=(term,kill,exists) peer=unconfined,",
    "}",
)
EXPECTED_PROFILE_LINES_SHA256 = hashlib.sha256(
    ("\n".join(EXPECTED_PROFILE_LINES) + "\n").encode("utf-8")
).hexdigest()


HELPER_SOURCE = rf'''import errno
import hashlib
import json
import os
import signal
import stat
import struct
import subprocess
import sys
import time

READ_EXACT = FRAME_READ_EXACT
STREAM = FRAME_STREAM
BOOTSTRAP_PROFILE = {BOOTSTRAP_PROFILE!r}
RUNTIME_PROFILE = {RUNTIME_PROFILE!r}
SUCCESS_MAGIC = {SUCCESS_MAGIC!r}
INPUTS = {tuple((name, size, digest) for name, (size, digest) in INPUT_CONTRACT.items())!r}
ENV = {{"HOME": "/nonexistent", "PATH": "/usr/bin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC0"}}

class ProbeError(RuntimeError):
    pass

def read_status(path):
    fields = {{}}
    with open(path, "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    try:
        result = {{
            "uid": [int(v) for v in fields["Uid"].split()],
            "gid": [int(v) for v in fields["Gid"].split()],
            "groups": [int(v) for v in fields["Groups"].split()],
            "capabilities": {{k: int(fields[k], 16) for k in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}},
            "no_new_privs": int(fields["NoNewPrivs"]),
        }}
    except (KeyError, ValueError) as exc:
        raise ProbeError("status_parse") from exc
    if result["uid"] != [0, 0, 0, 0] or result["gid"] != [0, 0, 0, 0]:
        raise ProbeError("identity")
    if result["groups"] or any(result["capabilities"].values()) or result["no_new_privs"] != 1:
        raise ProbeError("privilege_boundary")
    return result

def read_label(path, expected):
    with open(path, "r", encoding="ascii") as source:
        observed = source.read().strip()
    if observed != expected + " (enforce)":
        raise ProbeError("label")
    return observed

def live_fds(expected):
    live = set()
    for name in os.listdir("/proc/self/fd"):
        try:
            descriptor = int(name)
        except ValueError:
            continue
        try:
            os.fstat(descriptor)
        except OSError as exc:
            if exc.errno == errno.EBADF:
                continue
            raise
        live.add(descriptor)
    if live != set(expected):
        raise ProbeError("fd_leak")
    return sorted(live)

def verify_work_tmpfs():
    filesystem = os.statvfs("/work")
    total_bytes = filesystem.f_blocks * filesystem.f_frsize
    if total_bytes != {TMPFS_BYTES}:
        raise ProbeError("tmpfs_size")
    matches = []
    with open("/proc/self/mountinfo", "r", encoding="ascii") as source:
        for line in source:
            left, separator, right = line.rstrip("\n").partition(" - ")
            if not separator:
                raise ProbeError("mountinfo_parse")
            fields = left.split()
            filesystem_fields = right.split()
            if len(fields) < 6 or len(filesystem_fields) < 3:
                raise ProbeError("mountinfo_fields")
            if fields[4] == "/work":
                matches.append({{"filesystem_type": filesystem_fields[0], "mount_options": fields[5].split(",")}})
    if len(matches) != 1 or matches[0]["filesystem_type"] != "tmpfs":
        raise ProbeError("tmpfs_identity")
    if not {{"rw", "nosuid", "nodev"}}.issubset(set(matches[0]["mount_options"])):
        raise ProbeError("tmpfs_mount_options")
    return {{"filesystem_type": "tmpfs", "total_bytes": total_bytes, "mount_options": matches[0]["mount_options"]}}

def write_inputs():
    os.mkdir("/work/input", 0o700)
    observed = {{}}
    for name, expected_size, expected_hash in INPUTS:
        header = READ_EXACT(40)
        announced_size = struct.unpack(">Q", header[:8])[0]
        announced_hash = header[8:].hex()
        if announced_size != expected_size or announced_hash != expected_hash:
            raise ProbeError("input_header")
        descriptor = os.open("/work/input/" + name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o400)
        digest = hashlib.sha256()
        remaining = expected_size
        try:
            while remaining:
                block = READ_EXACT(min(4096, remaining))
                digest.update(block)
                remaining -= len(block)
                view = memoryview(block)
                while view:
                    count = os.write(descriptor, view)
                    if count <= 0:
                        raise ProbeError("short_write")
                    view = view[count:]
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o400)
            metadata = os.fstat(descriptor)
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
                    or metadata.st_size != expected_size or metadata.st_uid != 0
                    or metadata.st_gid != 0 or stat.S_IMODE(metadata.st_mode) != 0o400):
                raise ProbeError("input_inode")
        finally:
            os.close(descriptor)
        if digest.hexdigest() != expected_hash:
            raise ProbeError("input_digest")
        observed[name] = {{"size_bytes": expected_size, "sha256": expected_hash}}
    if STREAM.read(1) != b"":
        raise ProbeError("trailing_input")
    STREAM.close()
    try:
        os.close(0)
    except OSError as exc:
        if exc.errno != errno.EBADF:
            raise
    read_end, write_end = os.pipe2(os.O_CLOEXEC)
    os.close(write_end)
    if read_end != 0:
        os.dup2(read_end, 0, inheritable=True)
        os.close(read_end)
    else:
        os.set_inheritable(0, True)
    if os.read(0, 1) != b"" or not os.get_inheritable(0):
        raise ProbeError("stdin_eof_pipe")
    return observed

def terminate_runtime(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        stdout, stderr = process.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        stdout, stderr = process.communicate(timeout=2)
        if stdout or stderr:
            raise ProbeError("runtime_output")
        raise ProbeError("runtime_term_timeout")
    return stdout, stderr

def run_runtime():
    process = subprocess.Popen(["/usr/bin/sleep", "30"], stdin=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/work", env=ENV, close_fds=True, start_new_session=True)
    if process.stdin is not None or process.stdout is None or process.stderr is None:
        raise ProbeError("runtime_pipe")
    label_path = f"/proc/{{process.pid}}/attr/current"
    status_path = f"/proc/{{process.pid}}/status"
    expected_label = BOOTSTRAP_PROFILE + " (enforce)"
    deadline = time.monotonic() + 1
    label = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        try:
            with open(label_path, "r", encoding="ascii") as source:
                label = source.read().strip()
        except FileNotFoundError:
            time.sleep(0.01)
            continue
        if label == expected_label:
            break
        time.sleep(0.01)
    if label != expected_label:
        terminate_runtime(process)
        raise ProbeError("runtime_label")
    runtime_identity = read_status(status_path)
    stdout, stderr = terminate_runtime(process)
    if process.returncode != -signal.SIGTERM or stdout or stderr:
        raise ProbeError("runtime_exit")
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.01)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        raise ProbeError("runtime_residue")
    return {{"label": label, "identity": runtime_identity, "returncode": process.returncode, "stdin": "guest_internal_eof_pipe_fd0", "stdout_stderr": "internal_empty_pipes"}}

def emit_success(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if len(payload) > 8192:
        raise ProbeError("success_payload_size")
    frame = SUCCESS_MAGIC + struct.pack(">I", len(payload)) + payload + hashlib.sha256(payload).digest()
    view = memoryview(frame)
    while view:
        count = os.write(1, view)
        if count <= 0:
            raise ProbeError("success_write")
        view = view[count:]

def main():
    bootstrap_label = read_label("/proc/self/attr/current", BOOTSTRAP_PROFILE)
    bootstrap_identity = read_status("/proc/self/status")
    work_tmpfs = verify_work_tmpfs()
    observed = write_inputs()
    fds_after_stdin = live_fds({{0, 1, 2}})
    runtime = run_runtime()
    fds_before_success = live_fds({{0, 1, 2}})
    emit_success({{"status": "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V11", "bootstrap_profile": BOOTSTRAP_PROFILE, "bootstrap_label": bootstrap_label, "bootstrap_identity": bootstrap_identity, "work_tmpfs": work_tmpfs, "inputs": observed, "host_stdin_consumed_and_fd0_replaced_with_eof_pipe": True, "fds_after_stdin": fds_after_stdin, "runtime": runtime, "fds_before_success": fds_before_success, "host_writable_mounts": []}})
    return 0

try:
    raise SystemExit(main())
except (ProbeError, OSError, ValueError, UnicodeError, subprocess.SubprocessError):
    try:
        os.write(2, b"R8_STDIO_PROBE_V11_NO_GO\n")
    except OSError:
        pass
    raise SystemExit(2)
'''

HELPER_BYTES = HELPER_SOURCE.encode("utf-8")
HELPER_SHA256 = hashlib.sha256(HELPER_BYTES).hexdigest()
HELPER_SIZE_BYTES = len(HELPER_BYTES)

LOADER_SOURCE = f'''import hashlib,struct,sys
B=sys.stdin.buffer
def r(n):
 d=bytearray()
 while len(d)<n:
  x=B.read(n-len(d))
  if not x: raise SystemExit(91)
  d.extend(x)
 return bytes(d)
if r({len(STDIN_MAGIC)})!={STDIN_MAGIC!r}: raise SystemExit(92)
n=struct.unpack(">I",r(4))[0]
h=r(32)
if n!={HELPER_SIZE_BYTES} or h.hex()!="{HELPER_SHA256}": raise SystemExit(93)
p=r(n)
if hashlib.sha256(p).digest()!=h: raise SystemExit(94)
g={{"__name__":"__main__","FRAME_READ_EXACT":r,"FRAME_STREAM":B}}
exec(compile(p,"<r8-stdio-probe-helper-v11>","exec",dont_inherit=True,optimize=2),g,g)
'''
LOADER_BYTES = LOADER_SOURCE.encode("ascii")
LOADER_SHA256 = hashlib.sha256(LOADER_BYTES).hexdigest()
LOADER_SIZE_BYTES = len(LOADER_BYTES)

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed static, identity, framing, or conduit error."""


def read_regular_bytes(path: Path, *, limit: int = 2 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise GateError(f"unsafe regular file: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise GateError(f"regular file exceeds ceiling: {path}")
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def require_tool(name: str) -> dict[str, Any]:
    path, expected = TRUSTED_TOOLS[name]
    raw = read_regular_bytes(path, limit=512 * 1024 * 1024)
    metadata = os.stat(path, follow_symlinks=False)
    if metadata.st_uid != 0 or metadata.st_gid != 0 or not metadata.st_mode & stat.S_IXUSR:
        raise GateError(f"trusted tool metadata differs: {path}")
    if sha256_bytes(raw) != expected:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": expected, "size_bytes": len(raw)}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def build_stdin_frame() -> bytes:
    parts = [STDIN_MAGIC, struct.pack(">I", HELPER_SIZE_BYTES), bytes.fromhex(HELPER_SHA256), HELPER_BYTES]
    for name, raw in INPUT_PAYLOADS.items():
        size, digest = INPUT_CONTRACT[name]
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise GateError(f"synthetic payload identity differs: {name}")
        parts.extend((struct.pack(">Q", size), bytes.fromhex(digest), raw))
    frame = b"".join(parts)
    if len(frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("stdin frame exceeds its fixed ceiling")
    return frame


def parse_stdin_frame(frame: bytes) -> dict[str, Any]:
    if len(frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("stdin frame exceeds ceiling")
    view = memoryview(frame)
    position = 0

    def take(size: int) -> bytes:
        nonlocal position
        if size < 0 or position + size > len(view):
            raise GateError("stdin frame truncation")
        result = bytes(view[position:position + size])
        position += size
        return result

    if take(len(STDIN_MAGIC)) != STDIN_MAGIC:
        raise GateError("stdin frame magic differs")
    helper_size = struct.unpack(">I", take(4))[0]
    helper_hash = take(32).hex()
    helper = take(helper_size)
    if helper_size != HELPER_SIZE_BYTES or helper_hash != HELPER_SHA256 or sha256_bytes(helper) != helper_hash:
        raise GateError("stdin helper identity differs")
    inputs: dict[str, dict[str, Any]] = {}
    for name, (expected_size, expected_hash) in INPUT_CONTRACT.items():
        size = struct.unpack(">Q", take(8))[0]
        digest = take(32).hex()
        raw = take(size)
        if size != expected_size or digest != expected_hash or sha256_bytes(raw) != digest:
            raise GateError(f"stdin input identity differs: {name}")
        inputs[name] = {"size_bytes": size, "sha256": digest}
    if position != len(view):
        raise GateError("stdin frame has trailing bytes")
    return {"helper": {"size_bytes": helper_size, "sha256": helper_hash}, "inputs": inputs, "size_bytes": len(frame), "sha256": sha256_bytes(frame)}


def parse_success_frame(frame: bytes) -> dict[str, Any]:
    prefix_size = len(SUCCESS_MAGIC)
    payload_offset = prefix_size + 4
    if not payload_offset + 2 + 32 <= len(frame) <= MAX_SUCCESS_FRAME_BYTES or not frame.startswith(SUCCESS_MAGIC):
        raise GateError("success frame size or magic differs")
    size = struct.unpack(">I", frame[prefix_size:payload_offset])[0]
    if size < 2 or payload_offset + size + 32 != len(frame):
        raise GateError("success frame declared length differs")
    payload = frame[payload_offset:payload_offset + size]
    if hashlib.sha256(payload).digest() != frame[-32:]:
        raise GateError("success frame digest differs")
    try:
        value = json.loads(payload.decode("ascii"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError("success frame JSON differs") from exc
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if canonical != payload or not isinstance(value, dict):
        raise GateError("success frame is not canonical")
    expected_keys = {
        "status", "bootstrap_profile", "bootstrap_label", "bootstrap_identity",
        "work_tmpfs", "inputs", "host_stdin_consumed_and_fd0_replaced_with_eof_pipe",
        "fds_after_stdin", "runtime", "fds_before_success", "host_writable_mounts",
    }
    if set(value) != expected_keys:
        raise GateError("success frame key set differs")
    if value["status"] != "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V11":
        raise GateError("success status differs")
    if value["bootstrap_profile"] != BOOTSTRAP_PROFILE or value["bootstrap_label"] != BOOTSTRAP_PROFILE + " (enforce)":
        raise GateError("bootstrap label evidence differs")
    if value["host_writable_mounts"] != [] or value["host_stdin_consumed_and_fd0_replaced_with_eof_pipe"] is not True:
        raise GateError("success isolation evidence differs")
    if (type(value["fds_after_stdin"]) is not list or type(value["fds_before_success"]) is not list
            or any(type(descriptor) is not int for descriptor in value["fds_after_stdin"] + value["fds_before_success"])
            or value["fds_after_stdin"] != [0, 1, 2] or value["fds_before_success"] != [0, 1, 2]):
        raise GateError("success descriptor evidence differs")

    def require_guest_identity(observed: Any, context: str) -> None:
        if not isinstance(observed, dict) or set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
            raise GateError(f"{context} identity key set differs")
        uid, gid, groups = observed["uid"], observed["gid"], observed["groups"]
        if (type(uid) is not list or type(gid) is not list or type(groups) is not list
                or any(type(item) is not int for item in uid + gid + groups)
                or uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups != []):
            raise GateError(f"{context} identity differs")
        capabilities = observed["capabilities"]
        cap_keys = {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        if (type(capabilities) is not dict or set(capabilities) != cap_keys
                or any(type(capabilities[key]) is not int or capabilities[key] != 0 for key in cap_keys)):
            raise GateError(f"{context} capability evidence differs")
        if type(observed["no_new_privs"]) is not int or observed["no_new_privs"] != 1:
            raise GateError(f"{context} NNP evidence differs")

    require_guest_identity(value["bootstrap_identity"], "bootstrap")
    expected_inputs = {name: {"size_bytes": size, "sha256": digest} for name, (size, digest) in INPUT_CONTRACT.items()}
    if value["inputs"] != expected_inputs:
        raise GateError("success synthetic input evidence differs")
    work_tmpfs = value["work_tmpfs"]
    if not isinstance(work_tmpfs, dict) or set(work_tmpfs) != {"filesystem_type", "total_bytes", "mount_options"}:
        raise GateError("success tmpfs key set differs")
    options = work_tmpfs["mount_options"]
    if (work_tmpfs["filesystem_type"] != "tmpfs" or work_tmpfs["total_bytes"] != TMPFS_BYTES
            or isinstance(work_tmpfs["total_bytes"], bool) or not isinstance(options, list)
            or any(not isinstance(option, str) for option in options) or len(options) != len(set(options))
            or not {"rw", "nosuid", "nodev"}.issubset(set(options))):
        raise GateError("success tmpfs evidence differs")
    runtime = value["runtime"]
    if not isinstance(runtime, dict) or set(runtime) != {"label", "identity", "returncode", "stdin", "stdout_stderr"}:
        raise GateError("success runtime key set differs")
    require_guest_identity(runtime["identity"], "runtime")
    if (runtime["label"] != BOOTSTRAP_PROFILE + " (enforce)" or runtime["returncode"] != -int(signal.SIGTERM)
            or isinstance(runtime["returncode"], bool) or runtime["stdin"] != "guest_internal_eof_pipe_fd0"
            or runtime["stdout_stderr"] != "internal_empty_pipes"):
        raise GateError("success runtime evidence differs")
    return value


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
        raise GateError("sudo cleanup argv type differs")
    if command_tail not in (["/usr/bin/sudo", "-K"], ["/usr/bin/sudo", "-n", "/usr/bin/true"]):
        raise GateError("sudo cleanup inner command differs")
    if argv != expected_prefix + command_tail:
        raise GateError("sudo cleanup argv differs or contains injected arguments")
    if any(token == "--bounding-set" or token.startswith("--bounding-set=") for token in argv):
        raise GateError("sudo cleanup must preserve the host bounding set without a bounding argv token")
    if argv.count("--") != 1 or any(token in argv for token in ("--clear-groups", "--init-groups", "--keep-groups", "--no-new-privs")):
        raise GateError("sudo cleanup group, delimiter, or NNP argv contract differs")


def bwrap_argv() -> list[str]:
    return [
        "/usr/bin/timeout", "--foreground", "--signal=TERM", "--kill-after=2s", "15s",
        "/usr/bin/aa-exec", "-p", BOOTSTRAP_PROFILE, "--",
        "/usr/bin/bwrap", "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--uid", "0", "--gid", "0", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-stdio-probe-v11", "--clearenv",
        "--setenv", "HOME", "/nonexistent", "--setenv", "PATH", "/usr/bin",
        "--setenv", "LC_ALL", "C.UTF-8", "--setenv", "LANG", "C.UTF-8", "--setenv", "TZ", "UTC0",
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--size", str(TMPFS_BYTES), "--tmpfs", "/work", "--proc", "/proc", "--chdir", "/work", "--",
        "/usr/bin/python3.12", "-I", "-B", "-S", "-c", LOADER_SOURCE,
    ]


def verify_argv_contract(argv: list[str]) -> None:
    if argv != bwrap_argv():
        raise GateError("fixed argv differs")
    for forbidden in ("--bind", "--bind-fd", "--file", "--dev", "--dev-bind", "--share-net"):
        if forbidden in argv:
            raise GateError(f"forbidden bwrap option present: {forbidden}")
    if argv.count("--ro-bind") != 1 or argv[argv.index("--ro-bind") + 1:argv.index("--ro-bind") + 3] != ["/usr", "/usr"]:
        raise GateError("read-only host bind differs")
    index = argv.index("--size")
    if argv[index:index + 4] != ["--size", str(TMPFS_BYTES), "--tmpfs", "/work"]:
        raise GateError("bounded /work tmpfs differs")
    if argv[argv.index("--proc"):argv.index("--proc") + 2] != ["--proc", "/proc"]:
        raise GateError("single proc discovery target differs")


def _effective_lines(text: str) -> tuple[str, ...]:
    return tuple(line.strip() for line in (raw.split("#", 1)[0] for raw in text.splitlines()) if line.strip())


def verify_profile() -> dict[str, Any]:
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    text = raw.decode("utf-8")
    lines = _effective_lines(text)
    if lines != EXPECTED_PROFILE_LINES:
        raise GateError("effective AppArmor rule set differs from the exact reviewed tuple")
    lines_sha256 = hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()
    if lines_sha256 != EXPECTED_PROFILE_LINES_SHA256:
        raise GateError("effective AppArmor rule-set digest differs")
    if lines.count(f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted) {{") != 1:
        raise GateError("bootstrap profile identity differs")
    if lines.count(f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{") != 1:
        raise GateError("runtime profile identity differs")
    runtime_exec = "/usr/bin/sleep rix,"
    if (lines.count(runtime_exec) != 1
            or any("rpx" in line.lower() or " -> " in line for line in lines if "/usr/bin/sleep" in line)):
        raise GateError("NNP-compatible inherited sleep execution differs")
    if lines.count("deny capability dac_override,") != 1:
        raise GateError("exact evidence-derived silent dac_override deny differs")
    mount_lines = tuple(line for line in lines if line.startswith(("mount ", "remount ", "pivot_root ", "umount ")))
    if mount_lines != BASELINE_MOUNT_RULES:
        raise GateError("provenance-pinned mount baseline differs")
    if lines.count("/newroot/proc/ rw,") != 1:
        raise GateError("exact bootstrap /newroot/proc directory permission differs")
    if lines.count(PROC_MOUNT_RULE) != 1:
        raise GateError("exact evidence-derived proc mount rule differs")
    forbidden = (
        "/newroot/proc/**", "/newroot/proc/*", "/dev/", "/dev ",
        "GenCase", "DualSPHysics", "/home/zrj/scout_ws", "/opt/ros",
        "flags=(unconfined)", " mount,",
    )
    effective = "\n".join(lines)
    if any(token in effective for token in forbidden):
        raise GateError("profile contains forbidden or guessed authority")
    runtime = effective.split(f"profile {RUNTIME_PROFILE}", 1)[1]
    for token in ("userns ", "mount ", "capability ", "network ", "/proc", "/dev", "/work", "/usr/bin/python"):
        if token in runtime:
            raise GateError(f"runtime profile contains forbidden authority: {token}")
    return {
        "path": str(PROFILE_PATH),
        "sha256": sha256_bytes(raw),
        "effective_lines_sha256": lines_sha256,
        "effective_line_count": len(lines),
        "mount_rules": list(mount_lines),
        "production_authorized": False,
    }


def _assert_schema_objects_closed(node: Any, path: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise GateError(f"schema object is not closed: {path}")
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
            raise GateError(f"unsupported schema reference: {path}")
        target: Any = root
        for part in reference[2:].split("/"):
            target = target[part.replace("~1", "/").replace("~0", "~")]
        if not isinstance(target, dict):
            raise GateError(f"schema reference target differs: {path}")
        _validate_schema_instance(instance, target, root, path)
        return
    if "const" in node and not _json_equal(instance, node["const"]):
        raise GateError(f"policy const differs: {path}")
    if "enum" in node and not any(_json_equal(instance, option) for option in node["enum"]):
        raise GateError(f"policy enum differs: {path}")
    expected_type = node.get("type")
    type_ok = {
        "object": isinstance(instance, dict),
        "array": isinstance(instance, list),
        "string": isinstance(instance, str),
        "integer": isinstance(instance, int) and not isinstance(instance, bool),
        "boolean": isinstance(instance, bool),
    }.get(expected_type, True)
    if not type_ok:
        raise GateError(f"policy type differs: {path}")
    if expected_type == "object":
        properties = node.get("properties", {})
        required = node.get("required", [])
        if not isinstance(properties, dict) or not isinstance(required, list) or any(key not in instance for key in required):
            raise GateError(f"policy required object key differs: {path}")
        if node.get("additionalProperties") is False and any(key not in properties for key in instance):
            raise GateError(f"policy object has an additional key: {path}")
        for key, value in instance.items():
            child = properties.get(key)
            if isinstance(child, dict):
                _validate_schema_instance(value, child, root, f"{path}.{key}")
    elif expected_type == "array":
        if len(instance) < node.get("minItems", 0) or len(instance) > node.get("maxItems", 1 << 60):
            raise GateError(f"policy array length differs: {path}")
        child = node.get("items")
        if isinstance(child, dict):
            for index, value in enumerate(instance):
                _validate_schema_instance(value, child, root, f"{path}[{index}]")
    elif expected_type == "string":
        if len(instance) < node.get("minLength", 0) or len(instance) > node.get("maxLength", 1 << 60):
            raise GateError(f"policy string length differs: {path}")
        pattern = node.get("pattern")
        if pattern is not None and re.search(pattern, instance) is None:
            raise GateError(f"policy string pattern differs: {path}")
    elif expected_type == "integer":
        if instance < node.get("minimum", -(1 << 63)) or instance > node.get("maximum", 1 << 63):
            raise GateError(f"policy integer range differs: {path}")


def artifact_paths() -> dict[str, Path]:
    return {"gate": SCRIPT_PATH, "supervisor": SUPERVISOR_PATH, "profile": PROFILE_PATH, "schema": SCHEMA_PATH}


def verify_predecessor_v8_provenance() -> dict[str, Any]:
    package = WORKSPACE_ROOT
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
            raise GateError(f"frozen v8 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V8["snapshot_root"])
    metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
    translated_root_owner = os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o555
            or metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise GateError("frozen v8 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v8.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v8.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v8.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v8.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise GateError("frozen v8 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        file_metadata = os.lstat(path)
        expected = PREDECESSOR_V8["snapshot"]["artifacts"][name]
        root_owner = (file_metadata.st_uid, file_metadata.st_gid) == (0, 0)
        translated_root_owner = os.geteuid() != 0 and (file_metadata.st_uid, file_metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(file_metadata.st_mode) or stat.S_IMODE(file_metadata.st_mode) != 0o444
                or file_metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise GateError(f"frozen v8 snapshot artifact differs: {name}")
    receipt_root = Path("/home/zrj/scout_liquid_lab/audits")
    receipt_documents: dict[str, dict[str, Any]] = {}
    for name, suffix in (("start", "start"), ("execution", "execution"), ("lifecycle", "lifecycle")):
        path = receipt_root / f"{PREDECESSOR_V8['probe_id']}.{suffix}.json"
        raw = read_regular_bytes(path)
        file_metadata = os.lstat(path)
        expected = PREDECESSOR_V8["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (file_metadata.st_uid, file_metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(file_metadata.st_mode) != 0o440 or file_metadata.st_nlink != 1):
            raise GateError(f"frozen v8 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise GateError(f"frozen v8 receipt status differs: {name}")
        receipt_documents[name] = document
    audit = receipt_documents["execution"].get("apparmor_audit", {})
    if (audit.get("matching_total") != 1 or audit.get("expected_proc_mkdir_total") != 1
            or audit.get("unexpected_total") != 0 or audit.get("storage_overflow") is not False
            or len(audit.get("expected_proc_mkdir_denials", [])) != 1
            or f'profile="{PREDECESSOR_V8["discovery"]["profile"]}"' not in audit["expected_proc_mkdir_denials"][0]
            or 'operation="mkdir"' not in audit["expected_proc_mkdir_denials"][0]
            or 'name="/newroot/proc/"' not in audit["expected_proc_mkdir_denials"][0]):
        raise GateError("frozen v8 unique mkdir-denial evidence differs")
    lifecycle = receipt_documents["lifecycle"]
    if (lifecycle.get("cleanup", {}).get("stable_zero_scans") != [[], [], []]
            or lifecycle.get("cleanup", {}).get("post_unload_stable_zero_scans") != [[], [], []]
            or lifecycle.get("sysctls", {}).get("unchanged") is not True
            or lifecycle.get("sysctls", {}).get("before") != lifecycle.get("sysctls", {}).get("after")
            or any(lifecycle.get("profiles_after", {}).get("kernel_exact_counts", {}).values())
            or lifecycle.get("sudo_timestamp", {}).get("clear", {}).get("returncode") != 0
            or lifecycle.get("sudo_timestamp", {}).get("noninteractive_true_must_fail", {}).get("returncode") != 1):
        raise GateError("frozen v8 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "verified": True}


def verify_predecessor_v9_provenance() -> dict[str, Any]:
    package = WORKSPACE_ROOT
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
            raise GateError(f"frozen v9 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V9["snapshot_root"])
    metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
    translated_root_owner = os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o555
            or metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise GateError("frozen v9 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v9.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v9.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v9.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v9.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v9.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise GateError("frozen v9 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        file_metadata = os.lstat(path)
        expected = PREDECESSOR_V9["snapshot"]["artifacts"][name]
        root_owner = (file_metadata.st_uid, file_metadata.st_gid) == (0, 0)
        translated_root_owner = os.geteuid() != 0 and (file_metadata.st_uid, file_metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(file_metadata.st_mode) or stat.S_IMODE(file_metadata.st_mode) != 0o444
                or file_metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise GateError(f"frozen v9 snapshot artifact differs: {name}")
    receipt_root = Path("/home/zrj/scout_liquid_lab/audits")
    documents: dict[str, dict[str, Any]] = {}
    for name in ("start", "execution", "lifecycle"):
        path = receipt_root / f"{PREDECESSOR_V9['probe_id']}.{name}.json"
        raw = read_regular_bytes(path)
        file_metadata = os.lstat(path)
        expected = PREDECESSOR_V9["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (file_metadata.st_uid, file_metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(file_metadata.st_mode) != 0o440 or file_metadata.st_nlink != 1):
            raise GateError(f"frozen v9 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise GateError(f"frozen v9 receipt status differs: {name}")
        documents[name] = document
    for suffix in PREDECESSOR_V9["other_receipts_absent"]:
        if (receipt_root / f"{PREDECESSOR_V9['probe_id']}.{suffix}.json").exists():
            raise GateError(f"unexpected frozen v9 receipt exists: {suffix}")
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
        raise GateError("frozen v9 exact proc mount-denial evidence differs")
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
        raise GateError("frozen v9 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "verified": True}


def verify_predecessor_v10_provenance() -> dict[str, Any]:
    package = WORKSPACE_ROOT
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
            raise GateError(f"frozen v10 workspace artifact differs: {name}")
    snapshot_root = Path(PREDECESSOR_V10["snapshot_root"])
    root_metadata = os.lstat(snapshot_root)
    overflow_uid = int(Path("/proc/sys/kernel/overflowuid").read_text(encoding="ascii").strip())
    overflow_gid = int(Path("/proc/sys/kernel/overflowgid").read_text(encoding="ascii").strip())
    root_owner = (root_metadata.st_uid, root_metadata.st_gid) == (0, 0)
    translated_root_owner = os.geteuid() != 0 and (root_metadata.st_uid, root_metadata.st_gid) == (overflow_uid, overflow_gid)
    if (not stat.S_ISDIR(root_metadata.st_mode) or stat.S_IMODE(root_metadata.st_mode) != 0o555
            or root_metadata.st_nlink != 2 or not (root_owner or translated_root_owner)):
        raise GateError("frozen v10 snapshot directory metadata differs")
    snapshot_names = {
        "gate": "r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v10.py",
        "supervisor": "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v10.py",
        "profile": "r8-liquid-u3-stdio-transport-probe-v10.profile",
        "schema": "target_host_u3_stdio_apparmor_transport_probe_policy_v10.json",
        "policy": "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v10.json",
    }
    if {entry.name for entry in snapshot_root.iterdir()} != set(snapshot_names.values()):
        raise GateError("frozen v10 snapshot file set differs")
    for name, filename in snapshot_names.items():
        path = snapshot_root / filename
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V10["snapshot"]["artifacts"][name]
        root_owner = (metadata.st_uid, metadata.st_gid) == (0, 0)
        translated_root_owner = os.geteuid() != 0 and (metadata.st_uid, metadata.st_gid) == (overflow_uid, overflow_gid)
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o444
                or metadata.st_nlink != 1 or not (root_owner or translated_root_owner)):
            raise GateError(f"frozen v10 snapshot artifact differs: {name}")
    receipt_root = Path("/home/zrj/scout_liquid_lab/audits")
    documents: dict[str, dict[str, Any]] = {}
    for name in ("start", "execution", "lifecycle"):
        path = receipt_root / f"{PREDECESSOR_V10['probe_id']}.{name}.json"
        raw = read_regular_bytes(path)
        metadata = os.lstat(path)
        expected = PREDECESSOR_V10["receipts"][name]
        if (len(raw) != expected["size_bytes"] or sha256_bytes(raw) != expected["sha256"]
                or (metadata.st_uid, metadata.st_gid) != tuple(expected["owner"])
                or stat.S_IMODE(metadata.st_mode) != 0o440 or metadata.st_nlink != 1):
            raise GateError(f"frozen v10 receipt bytes or metadata differ: {name}")
        document = json.loads(raw.decode("utf-8"))
        if type(document) is not dict or document.get("status") != expected["status"]:
            raise GateError(f"frozen v10 receipt status differs: {name}")
        documents[name] = document
    for suffix in PREDECESSOR_V10["other_receipts_absent"]:
        if (receipt_root / f"{PREDECESSOR_V10['probe_id']}.{suffix}.json").exists():
            raise GateError(f"unexpected frozen v10 receipt exists: {suffix}")
    execution = documents["execution"]
    audit = execution.get("apparmor_audit", {})
    discovery = PREDECESSOR_V10["discovery"]
    denials = audit.get("unexpected_denials", [])
    if type(denials) is not list or len(denials) != len(discovery["denials"]):
        raise GateError("frozen v10 denial count differs")
    for entry, expected in zip(denials, discovery["denials"], strict=True):
        fields = {
            item.get("key"): item.get("value")
            for item in entry.get("fields", [])
            if type(item) is dict
        } if type(entry) is dict else {}
        for key in ("operation", "class", "comm", "capname", "name", "info", "error", "target"):
            if key in expected and fields.get(key) != expected[key]:
                raise GateError(f"frozen v10 denial field differs: {key}")
        if (fields.get("apparmor") != "DENIED"
                or fields.get("profile") != "r8-liquid-u3-stdio-transport-bootstrap-v10-20260808t045130z"
                or type(fields.get("pid")) is not str or not fields["pid"].isdigit() or int(fields["pid"]) < 1
                or entry.get("line_sha256") != expected["line_sha256"]
                or entry.get("line_size_bytes") != expected["line_size_bytes"]):
            raise GateError("frozen v10 denial identity differs")
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
        raise GateError("frozen v10 execution evidence differs")
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
        raise GateError("frozen v10 lifecycle closure differs")
    return {"workspace_artifacts": 6, "snapshot_artifacts": 5, "receipts": 3, "denials": 5, "verified": True}

def verify_static() -> dict[str, Any]:
    policy = read_json(POLICY_PATH)
    schema = read_json(SCHEMA_PATH)
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("closed top-level policy/schema contract differs")
    expected = {
        "schema_version": "smpcc-r8-liquid-target-u3-stdio-apparmor-transport-probe-policy-v11",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_STDIO_APPARMOR_TRANSPORT_PROBE_V11",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "status": "STATIC_READY_V11_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT",
        "next_allowed_stage": "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT",
    }
    for key, value in expected.items():
        if policy.get(key) != value:
            raise GateError(f"policy identity differs: {key}")
    authorization = policy.get("authorization", {})
    if authorization.get("static_task_execution_performed") is not False or authorization.get("attempts_per_identity") != 1:
        raise GateError("static-only one-shot authorization differs")
    if authorization.get("admission_token") != ADMISSION_TOKEN:
        raise GateError("explicit admission token differs")
    frozen = policy.get("frozen_identity", {})
    expected_frozen = {
        "probe_id": PROBE_ID,
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        **FROZEN_RECEIPT_PATHS,
    }
    if frozen != expected_frozen:
        raise GateError("frozen probe identity, labels, or receipt paths differ")
    if policy.get("provenance", {}).get("predecessor_no_go") != PREDECESSOR_NO_GO:
        raise GateError("predecessor v7 pre-start cleanup NO_GO record differs")
    if policy.get("provenance", {}).get("predecessor_v8") != PREDECESSOR_V8:
        raise GateError("predecessor v8 frozen provenance record differs")
    predecessor_v8 = verify_predecessor_v8_provenance()
    if policy.get("provenance", {}).get("predecessor_v9") != PREDECESSOR_V9:
        raise GateError("predecessor v9 frozen provenance record differs")
    predecessor_v9 = verify_predecessor_v9_provenance()
    if policy.get("provenance", {}).get("predecessor_v10") != PREDECESSOR_V10:
        raise GateError("predecessor v10 frozen provenance record differs")
    predecessor_v10 = verify_predecessor_v10_provenance()
    if policy.get("trusted_system_tools", {}).get("unprivileged_runner") != trusted_tool_policy():
        raise GateError("unprivileged trusted-tool contract differs")
    reviewed = policy.get("reviewed_bytes", {})
    expected_inputs = [{"name": name, "size_bytes": size, "sha256": digest} for name, (size, digest) in INPUT_CONTRACT.items()]
    if reviewed.get("helper") != {"size_bytes": HELPER_SIZE_BYTES, "sha256": HELPER_SHA256}:
        raise GateError("helper bytes policy differs")
    if reviewed.get("loader") != {"size_bytes": LOADER_SIZE_BYTES, "sha256": LOADER_SHA256}:
        raise GateError("loader bytes policy differs")
    if reviewed.get("inputs") != expected_inputs:
        raise GateError("synthetic input policy differs")
    frame = build_stdin_frame()
    parsed_frame = parse_stdin_frame(frame)
    if reviewed.get("stdin_frame") != {"size_bytes": len(frame), "sha256": sha256_bytes(frame), "maximum_size_bytes": MAX_STDIN_FRAME_BYTES}:
        raise GateError("stdin frame policy differs")
    transport = policy.get("stdio_transport", {})
    if (transport.get("stdin_magic_hex") != STDIN_MAGIC.hex()
            or transport.get("success_magic_hex") != SUCCESS_MAGIC.hex()):
        raise GateError("fresh v11 stdio protocol magic differs")
    verify_argv_contract(bwrap_argv())
    commands = policy.get("fixed_commands", {})
    if commands.get("aa_status_json_argv") != list(AA_STATUS_JSON_ARGV):
        raise GateError("fixed aa-status JSON argv differs")
    if commands.get("bwrap_argv_sha256") != sha256_bytes(json.dumps(bwrap_argv(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")):
        raise GateError("fixed bwrap argv hash differs")
    journal_contract = {
        "journal_boot_id_path": "/proc/sys/kernel/random/boot_id",
        "journal_anchor_argv_template": ["/usr/bin/journalctl", "-k", "--no-pager", "--quiet", "--boot=<BOOT_ID_32HEX>", "--lines=0", "--show-cursor"],
        "journal_sync_argv": ["/usr/bin/journalctl", "--sync"],
        "journal_query_argv_template": ["/usr/bin/journalctl", "-k", "--no-pager", "--quiet", "--output=short-iso-precise", "--boot=<BOOT_ID_32HEX>", "--after-cursor=<PRE_RUN_CURSOR>", "--show-cursor"],
        "journal_anchor_stdout_ceiling_bytes": 4096,
        "journal_query_stdout_ceiling_bytes": 131072,
        "journal_stderr_ceiling_bytes": 16384,
        "audit_line_ceiling_bytes": 4096,
        "audit_storage_ceiling": 16,
    }
    if any(commands.get(key) != value for key, value in journal_contract.items()):
        raise GateError("fixed cursor/boot journal evidence contract differs")
    clear_argv, verify_argv = sudo_timestamp_argvs()
    if (commands.get("sudo_timestamp_clear_argv") != clear_argv
            or commands.get("sudo_timestamp_verify_argv") != verify_argv):
        raise GateError("fixed sudo timestamp cleanup argv differs")
    snapshot_supervisor = str(SNAPSHOT_ROOT / SUPERVISOR_NAME)
    expected_run = [
        "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", snapshot_supervisor,
        "run", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
        "--admission-token", ADMISSION_TOKEN,
    ]
    expected_recover = [
        "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", snapshot_supervisor,
        "recover-only", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
        "--recovery-token", f"{PROBE_ID}:cleanup-only-no-probe",
    ]
    if commands.get("run_argv_template") != expected_run or commands.get("recover_argv_template") != expected_recover:
        raise GateError("root snapshot supervisor must be read by the pinned Python entrypoint")
    bootstrap = policy.get("snapshot_bootstrap", {})
    if (bootstrap.get("file_mode") != "0444"
            or bootstrap.get("runtime_supervisor_entrypoint") != "/usr/bin/python3.12 -I -B reads root-owned 0444 snapshot supervisor"
            or bootstrap.get("direct_snapshot_supervisor_exec_forbidden") is not True):
        raise GateError("snapshot file mode and Python entrypoint contract differ")
    expected_producer = {
        "workspace_snapshot_producer_root_forbidden": True,
        "workspace_snapshot_producer_exact_path": str(WORKSPACE_SUPERVISOR_PATH),
        "workspace_snapshot_producer_uid": [HOST_UID] * 4,
        "workspace_snapshot_producer_gid": [HOST_GID] * 4,
        "workspace_snapshot_producer_active_capability_sets": {"CapInh": 0, "CapPrm": 0, "CapEff": 0, "CapAmb": 0},
        "workspace_snapshot_producer_stdout_on_identity_failure": "empty",
    }
    if any(bootstrap.get(key) != value for key, value in expected_producer.items()):
        raise GateError("unprivileged workspace snapshot producer identity contract differs")
    receipt_contract = policy.get("receipt_contract", {})
    if (receipt_contract.get("sudo_group_membership") != SUDO_GROUP_MEMBERSHIP_CONTRACT
            or receipt_contract.get("sudo_cleanup_identity") != SUDO_CLEANUP_IDENTITY_CONTRACT
            or receipt_contract.get("sudo_cleanup_bounding_mode") != SUDO_CLEANUP_BOUNDING_MODE):
        raise GateError("sudo cleanup membership or identity contract differs")
    if policy.get("trusted_system_tools", {}).get("root_supervisor", {}).get("python") != trusted_tool_policy()["python"]:
        raise GateError("root Python entrypoint identity differs")
    profile = verify_profile()
    mount_policy = policy.get("mount_discovery", {})
    if mount_policy.get("effective_mount_rules") != list(BASELINE_MOUNT_RULES):
        raise GateError("policy mount baseline differs")
    if mount_policy.get("effective_profile_lines_sha256") != EXPECTED_PROFILE_LINES_SHA256:
        raise GateError("policy effective profile semantics digest differs")
    semantics_digest = policy.get("profile_semantics", {}).get("effective_lines_sha256")
    if semantics_digest != EXPECTED_PROFILE_LINES_SHA256 or semantics_digest != profile["effective_lines_sha256"]:
        raise GateError("policy profile_semantics effective-lines digest differs from actual profile")
    semantics = policy.get("profile_semantics", {})
    expected_semantics = {
        "effective_line_count": len(EXPECTED_PROFILE_LINES),
        "effective_lines_sha256": EXPECTED_PROFILE_LINES_SHA256,
        "external_includes": [],
        "named_only": True,
        "persistent_install": False,
        "runtime_execution": "/usr/bin/sleep rix,",
        "runtime_profile_loaded_but_unreachable": True,
        "runtime_authority": "inherits_bootstrap_label_with_empty_groups_zero_caps_nnp_nested_userns_disabled_and_empty_network_namespace",
        "explicit_silent_denials": ["deny capability dac_override,"],
        "avoidable_python_reads_prevented_by": ["python_-S", "TZ=UTC0"],
    }
    if semantics != expected_semantics:
        raise GateError("policy inherited-runtime and explicit-deny semantics differ")
    if mount_policy.get("proc_rules") != [PROC_MOUNT_RULE]:
        raise GateError("policy exact evidence-derived proc rule differs")
    expected_v9_denial = {
        "profile": "bootstrap", "apparmor": "DENIED", "operation": "mount", "class": "mount",
        "info": "failed mntpnt match", "error": "-13", "name": "/newroot/proc/",
        "comm": "bwrap", "fstype": "proc", "srcname": "proc",
        "flags_required": ["rw", "nosuid", "nodev", "noexec"], "flags_optional": [],
    }
    if (mount_policy.get("predecessor_v9_denial_evidence") != expected_v9_denial
            or mount_policy.get("zero_logged_denials_required") is not True
            or mount_policy.get("rules_may_be_guessed") is not False
            or mount_policy.get("success_frame_required") is not True):
        raise GateError("v11 evidence-derived proc success admission contract differs")
    if policy.get("evidence_classification") != EVIDENCE_CLASSIFICATION_CONTRACT:
        raise GateError("v11 zero-denial success evidence classification contract differs")
    artifacts = policy.get("trusted_artifacts", {})
    if set(artifacts) != set(artifact_paths()):
        raise GateError("trusted artifact set differs")
    observed_artifacts: dict[str, dict[str, Any]] = {}
    for name, path in artifact_paths().items():
        raw = read_regular_bytes(path)
        entry = artifacts[name]
        if entry.get("sha256") != sha256_bytes(raw) or entry.get("size_bytes") != len(raw):
            raise GateError(f"trusted artifact identity differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}
    return {"policy_sha256": sha256_bytes(read_regular_bytes(POLICY_PATH)), "artifacts": observed_artifacts, "profile": profile, "stdin_frame": parsed_frame, "helper_sha256": HELPER_SHA256, "loader_sha256": LOADER_SHA256, "predecessor_v8": predecessor_v8, "predecessor_v9": predecessor_v9, "predecessor_v10": predecessor_v10, "execution_performed": False}


def read_identity_status(path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    try:
        return {
            "uid": [int(value) for value in fields["Uid"].split()],
            "gid": [int(value) for value in fields["Gid"].split()],
            "groups": [int(value) for value in fields["Groups"].split()],
            "capabilities": {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
            "no_new_privs": int(fields["NoNewPrivs"]),
        }
    except (KeyError, ValueError) as exc:
        raise GateError("cannot parse host child identity") from exc


def verify_host_identity(status_value: Mapping[str, Any] | None = None) -> dict[str, Any]:
    observed = dict(read_identity_status() if status_value is None else status_value)
    if set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
        raise GateError("host gate identity key set differs")
    uid, gid, groups = observed.get("uid"), observed.get("gid"), observed.get("groups")
    if (type(uid) is not list or type(gid) is not list
            or any(type(item) is not int for item in uid + gid)
            or uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4):
        raise GateError("host gate UID/GID fields differ")
    if type(groups) is not list or any(type(item) is not int for item in groups) or groups != []:
        raise GateError("host gate supplementary groups differ")
    caps = observed.get("capabilities")
    if (type(caps) is not dict or set(caps) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
            or any(type(value) is not int or value != 0 for value in caps.values())):
        raise GateError("host gate capability sets differ")
    if type(observed.get("no_new_privs")) is not int or observed.get("no_new_privs") != 1:
        raise GateError("host gate NNP must be one before aa-exec")
    return observed


def verify_snapshot_runtime() -> dict[str, Any]:
    if SNAPSHOT_ENV not in os.environ or Path(os.environ[SNAPSHOT_ENV]) != SNAPSHOT_ROOT:
        raise GateError("internal-run requires the exact snapshot environment")
    if SCRIPT_PATH.parent != SNAPSHOT_ROOT:
        raise GateError("internal-run gate is not executing from the snapshot")
    root_metadata = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root_metadata.st_mode) or root_metadata.st_uid != 0 or root_metadata.st_gid != 0 or stat.S_IMODE(root_metadata.st_mode) != 0o555:
        raise GateError("snapshot directory ownership or mode differs")
    for path in (SCRIPT_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH, SUPERVISOR_PATH):
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444:
            raise GateError(f"snapshot artifact metadata differs: {path.name}")
    return {"root": str(SNAPSHOT_ROOT), "mode": "0555", "owner": [0, 0]}


def consume_root_admission_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[ADMISSION_FD_ENV])
        expected_sha256 = os.environ[ADMISSION_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise GateError("root admission FD capability metadata is absent") from exc
    if descriptor < 3 or len(expected_sha256) != 64 or any(character not in "0123456789abcdef" for character in expected_sha256):
        raise GateError("root admission FD capability metadata differs")
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or stat.S_IMODE(metadata.st_mode) != 0o600 or not os.get_inheritable(descriptor)):
            raise GateError("root admission FD capability inode differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise GateError("root admission FD capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError("root admission FD capability has trailing bytes")
    finally:
        os.close(descriptor)
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise GateError("root admission FD capability digest differs")
    os.environ.pop(ADMISSION_FD_ENV, None)
    os.environ.pop(ADMISSION_SHA256_ENV, None)
    return {
        "transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ",
        "size_bytes": len(payload),
        "sha256": expected_sha256,
        "pipe_uid": metadata.st_uid,
        "pipe_gid": metadata.st_gid,
        "pipe_mode": "0600",
    }


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


def _proc_labeled_members(proc_root: Path = Path("/proc")) -> list[dict[str, Any]]:
    """Enumerate all threads in the fixed UID1000 gate tree under fresh labels.

    This is deliberately local unprivileged cleanup evidence, not a claim of
    all-UID label absence.  The root supervisor performs the authoritative
    all-task scan before and after profile lifecycle operations.
    """

    members: list[dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal():
            continue
        pid = int(entry.name)
        try:
            status_raw = (entry / "status").read_text(encoding="ascii")
            uid_line = next((line for line in status_raw.splitlines() if line.startswith("Uid:")), "")
            uid_fields = [int(value) for value in uid_line.split()[1:]]
            if len(uid_fields) != 4:
                raise GateError(f"cannot parse task UID authority for pid {pid}")
            if uid_fields != [HOST_UID] * 4:
                continue
            leader_stat_raw = (entry / "stat").read_text(encoding="ascii")
            task_entries = tuple((entry / "task").iterdir())
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ESRCH):
                continue
            raise GateError(f"cannot read labeled task authority for pid {pid}: {exc}") from exc
        end = leader_stat_raw.rfind(")")
        leader_fields = leader_stat_raw[end + 2:].split() if end >= 0 else []
        if len(leader_fields) < 20:
            raise GateError(f"cannot parse labeled task stat for pid {pid}")
        tgid_starttime = int(leader_fields[19])
        if tgid_starttime < 1:
            raise GateError(f"non-positive tgid starttime for pid {pid}")
        for task_entry in task_entries:
            if not task_entry.name.isdecimal():
                continue
            tid = int(task_entry.name)
            try:
                attr_raw = (task_entry / "attr/current").read_text(encoding="utf-8")
                task_stat_raw = (task_entry / "stat").read_text(encoding="ascii")
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                if exc.errno in (errno.ENOENT, errno.ESRCH):
                    continue
                raise GateError(f"cannot read labeled thread authority for tgid {pid} tid {tid}: {exc}") from exc
            task_end = task_stat_raw.rfind(")")
            task_fields = task_stat_raw[task_end + 2:].split() if task_end >= 0 else []
            if len(task_fields) < 20:
                raise GateError(f"cannot parse labeled thread stat for tgid {pid} tid {tid}")
            tid_starttime = int(task_fields[19])
            if tid_starttime < 1:
                raise GateError(f"non-positive tid starttime for tgid {pid} tid {tid}")
            label = attr_raw.strip().split(" (", 1)[0]
            if label in (BOOTSTRAP_PROFILE, RUNTIME_PROFILE):
                members.append({
                    "tgid": pid,
                    "tid": tid,
                    "pgrp": int(task_fields[2]),
                    "tgid_starttime": tgid_starttime,
                    "tid_starttime": tid_starttime,
                    "label": label,
                })
    return sorted(members, key=lambda item: (item["tgid"], item["tid"]))


def _signal_open_pidfd(pidfd: int, sig: signal.Signals) -> bool:
    try:
        signal.pidfd_send_signal(pidfd, sig)
        return True
    except ProcessLookupError:
        return False


def _signal_pid(pid: int, sig: signal.Signals, starttime: int) -> bool:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise GateError("pidfd signaling is unavailable")
    if not isinstance(starttime, int) or isinstance(starttime, bool) or starttime < 1:
        raise GateError("refusing to signal a task without a frozen positive starttime")
    try:
        pidfd = os.pidfd_open(pid, 0)
    except ProcessLookupError:
        return False
    try:
        if _proc_starttime(pid) != starttime:
            return False
        return _signal_open_pidfd(pidfd, sig)
    finally:
        os.close(pidfd)


def _terminate_probe_tree(process: subprocess.Popen[bytes], leader_pidfd: int) -> dict[str, Any]:
    """Bounded pidfd cleanup independent of leader lifetime and process group."""

    evidence: dict[str, Any] = {"term": [], "kill": [], "stable_zero_scans": []}
    if process.poll() is None and _signal_open_pidfd(leader_pidfd, signal.SIGTERM):
        evidence["term"].append({"pid": process.pid, "role": "leader"})
    signaled_tgids: set[int] = set()
    for item in _proc_labeled_members():
        if item["tgid"] == process.pid or item["tgid"] in signaled_tgids:
            continue
        if _signal_pid(item["tgid"], signal.SIGTERM, item["tgid_starttime"]):
            signaled_tgids.add(item["tgid"])
            evidence["term"].append(item)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if process.poll() is not None and not _proc_labeled_members():
            break
        time.sleep(0.05)
    if process.poll() is None and _signal_open_pidfd(leader_pidfd, signal.SIGKILL):
        evidence["kill"].append({"pid": process.pid, "role": "leader"})
    signaled_tgids = set()
    for item in _proc_labeled_members():
        if item["tgid"] == process.pid or item["tgid"] in signaled_tgids:
            continue
        if _signal_pid(item["tgid"], signal.SIGKILL, item["tgid_starttime"]):
            signaled_tgids.add(item["tgid"])
            evidence["kill"].append(item)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and _proc_labeled_members():
        time.sleep(0.05)
    for index in range(3):
        current = _proc_labeled_members()
        evidence["stable_zero_scans"].append(current)
        if current:
            raise GateError(f"labeled task residue remains after bounded cleanup: {current}")
        if index < 2:
            time.sleep(0.05)
    return evidence


def _bounded_extend(target: bytearray, block: bytes, limit: int) -> bool:
    remaining = max(0, limit - len(target))
    target.extend(block[:remaining])
    return len(block) > remaining


def run_bounded_guest(argv: list[str], input_frame: bytes) -> tuple[int, bytes, bytes, int]:
    if len(input_frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("guest stdin exceeds its hard ceiling")
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise GateError("pidfd signaling is unavailable")
    process = subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
        close_fds=True,
        start_new_session=True,
    )
    try:
        leader_pidfd = os.pidfd_open(process.pid, 0)
    except BaseException:
        # The unreaped direct child PID cannot be reused here, so Popen.kill is
        # the only safe fallback when a birth-time pidfd cannot be retained.
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
    failure: str | None = None
    cleanup_failure: str | None = None
    deadline = time.monotonic() + OUTER_DEADLINE_SECONDS
    try:
        if process.stdin is None or process.stdout is None or process.stderr is None:
            raise GateError("guest conduit pipes are unavailable")
        descriptors = {"stdin": process.stdin.fileno(), "stdout": process.stdout.fileno(), "stderr": process.stderr.fileno()}
        for descriptor in descriptors.values():
            os.set_blocking(descriptor, False)
        selector = selectors.DefaultSelector()
        selector.register(descriptors["stdin"], selectors.EVENT_WRITE, "stdin")
        selector.register(descriptors["stdout"], selectors.EVENT_READ, "stdout")
        selector.register(descriptors["stderr"], selectors.EVENT_READ, "stderr")
        while selector.get_map():
            if time.monotonic() >= deadline:
                failure = failure or "guest conduit exceeded its outer deadline"
                break
            events = selector.select(timeout=0.1)
            for key, _mask in events:
                descriptor = key.fd
                channel = key.data
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, input_frame[position:position + 4096])
                    except BrokenPipeError:
                        count = 0
                    except BlockingIOError:
                        continue
                    if count:
                        position += count
                    if not count or position == len(input_frame):
                        selector.unregister(descriptor)
                        process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 4096)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    (process.stdout if channel == "stdout" else process.stderr).close()
                    continue
                target = stdout if channel == "stdout" else stderr
                limit = MAX_SUCCESS_FRAME_BYTES if channel == "stdout" else MAX_CHILD_STDERR_BYTES
                if _bounded_extend(target, block, limit):
                    failure = f"guest {channel} exceeded its hard byte ceiling"
                    break
            if failure is not None:
                break
    finally:
        if selector is not None:
            selector.close()
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                stream.close()
            except (OSError, AttributeError):
                pass
        try:
            members = _proc_labeled_members()
            if process.poll() is None or members:
                _terminate_probe_tree(process, leader_pidfd)
        except (GateError, OSError) as exc:
            cleanup_failure = f"bounded labeled-task cleanup failed: {exc}"
        finally:
            if process.poll() is None:
                try:
                    _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                except OSError as exc:
                    cleanup_failure = cleanup_failure or f"final leader pidfd kill failed: {exc}"
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                cleanup_failure = cleanup_failure or "guest leader did not reap after final pidfd kill"
            os.close(leader_pidfd)
        if cleanup_failure is not None:
            raise GateError(cleanup_failure)
    returncode = process.returncode
    if returncode is None:
        raise GateError("guest leader return code is unavailable after reap")
    if failure is not None:
        raise GateError(failure)
    return returncode, bytes(stdout), bytes(stderr), position


def internal_run() -> tuple[int, bytes, bytes]:
    if os.environ.get(ADMISSION_ENV) != ADMISSION_TOKEN:
        raise GateError("explicit one-shot admission token is absent")
    verify_snapshot_runtime()
    verify_host_identity()
    consume_root_admission_capability()
    review = verify_static()
    for name in TRUSTED_TOOLS:
        require_tool(name)
    frame = build_stdin_frame()
    if parse_stdin_frame(frame)["sha256"] != review["stdin_frame"]["sha256"]:
        raise GateError("runtime stdin frame differs from static review")
    returncode, stdout, stderr, consumed = run_bounded_guest(bwrap_argv(), frame)
    if returncode == 0:
        if consumed != len(frame):
            raise GateError("successful guest did not consume the complete fixed stdin frame")
        if stderr:
            raise GateError("successful guest emitted stderr")
        parse_success_frame(stdout)
        return returncode, stdout, b""
    if stdout:
        raise GateError("failed guest emitted forbidden stdout")
    return returncode, b"", stderr


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "internal-run"))
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = verify_static()
            print(json.dumps({"status": "PASS_V11_STATIC_DRAFT_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        returncode, stdout, stderr = internal_run()
        if stdout:
            sys.stdout.buffer.write(stdout)
            sys.stdout.buffer.flush()
        if stderr:
            sys.stderr.buffer.write(stderr)
            sys.stderr.buffer.flush()
        return 0 if returncode == 0 else min(125, max(1, returncode))
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        stream = sys.stderr if arguments.command == "internal-run" else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=stream)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
