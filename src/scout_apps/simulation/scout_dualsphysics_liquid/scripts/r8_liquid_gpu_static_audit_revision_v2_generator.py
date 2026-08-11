#!/usr/bin/env python3
"""Create-new v2 audit revision after the v1 dump exceeded 256 MiB.

The only semantic change is replacing raw ``cuobjdump --dump-elf`` output
with in-sandbox extraction plus bounded ``readelf`` summaries for the exact 11
cubins.  Extracted files live only in guest tmpfs.  The host still receives
text/JSON through pipes and never parses binary content.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import textwrap
from pathlib import Path
from typing import Any


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
V1_POLICY = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v1.json"
)
V1_POLICY_SHA256 = "e81b735acb74d3a73fd81a384576f65ffb1835e18f2caecbbad7a93de0d7e347"
V1_PROFILE = PACKAGE / (
    "config/apparmor_drafts/"
    "r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile"
)
V1_PROFILE_SHA256 = "d38092826e9b17d59c20d9ef1be3b30517c5cc106e316c394877c45c72f693ed"
V1_FAILURE = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v1_lifecycle.json"
)
V1_FAILURE_SHA256 = "48d9ef83a0673df5a2befa3d3e5fa1074218268699928abc9cc3fb13340ed656"
V1_OVERSIZE_LOG = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v1.evidence/"
    "006_cuobjdump_dump_elf.stdout.log"
)

V2_PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2"
V2_PROFILE = PACKAGE / f"config/apparmor_drafts/{V2_PROFILE_NAME}.profile"
V2_POLICY = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v2.json"
)

EXPECTED_CUBINS = [f"DualSPHysics5.{index}.sm_120.cubin" for index in range(1, 12)]


CUBIN_HELPER = textwrap.dedent(
    r'''
    import hashlib
    import json
    import os
    import re
    import stat
    import subprocess
    import sys

    ROOT = "/audit/tmp"
    CANDIDATE = "/audit/input/DualSPHysics5.4_linux64"
    EXPECTED = [f"DualSPHysics5.{index}.sm_120.cubin" for index in range(1, 12)]

    extracted = subprocess.run(
        ["/usr/local/cuda-12.8/bin/cuobjdump", "--extract-elf", "all", CANDIDATE],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=240,
        check=False,
    )
    if extracted.returncode != 0:
        print(json.dumps({"status": "FAIL_EXTRACT", "returncode": extracted.returncode,
                          "stdout_sha256": hashlib.sha256(extracted.stdout).hexdigest(),
                          "stderr_sha256": hashlib.sha256(extracted.stderr).hexdigest()}))
        raise SystemExit(20)
    actual = sorted(os.listdir(ROOT))
    if actual != sorted(EXPECTED):
        print(json.dumps({"status": "FAIL_FILE_SET", "actual": actual}))
        raise SystemExit(21)

    records = []
    total_text = 0
    for name in EXPECTED:
        path = os.path.join(ROOT, name)
        info = os.lstat(path)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0:
            print(json.dumps({"status": "FAIL_FILE_TYPE", "name": name}))
            raise SystemExit(22)
        digest = hashlib.sha256()
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            while True:
                block = os.read(descriptor, 1024 * 1024)
                if not block:
                    break
                digest.update(block)
        finally:
            os.close(descriptor)
        parsed = subprocess.run(
            ["/usr/bin/readelf", "-hW", "-SW", path],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
        if parsed.returncode != 0 or len(parsed.stdout) > 8 * 1024 * 1024 or len(parsed.stderr) > 8 * 1024 * 1024:
            print(json.dumps({"status": "FAIL_READELF", "name": name,
                              "returncode": parsed.returncode,
                              "stdout_bytes": len(parsed.stdout),
                              "stderr_bytes": len(parsed.stderr)}))
            raise SystemExit(23)
        text = parsed.stdout.decode("utf-8", "strict")
        type_match = re.search(r"^\s*Type:\s*(.+?)\s*$", text, re.MULTILINE)
        machine_match = re.search(r"^\s*Machine:\s*(.+?)\s*$", text, re.MULTILINE)
        if type_match is None or machine_match is None:
            print(json.dumps({"status": "FAIL_HEADER", "name": name}))
            raise SystemExit(24)
        nonzero = 0
        for line in text.splitlines():
            match = re.match(
                r"^\s*\[\s*\d+\]\s+(\.text\.\S+)\s+PROGBITS\s+\S+\s+\S+\s+([0-9a-fA-F]+)\s+\S+\s+([A-Z]+)",
                line,
            )
            if match and int(match.group(2), 16) > 0 and "X" in match.group(3):
                nonzero += 1
        if type_match.group(1) != "EXEC (Executable file)" or machine_match.group(1) != "NVIDIA CUDA architecture" or nonzero < 1:
            print(json.dumps({"status": "FAIL_CUBIN_STRUCTURE", "name": name,
                              "type": type_match.group(1), "machine": machine_match.group(1),
                              "nonzero_executable_text": nonzero}))
            raise SystemExit(25)
        total_text += nonzero
        records.append({
            "name": name,
            "size_bytes": info.st_size,
            "sha256": digest.hexdigest(),
            "elf_type": type_match.group(1),
            "machine": machine_match.group(1),
            "nonzero_executable_text": nonzero,
            "readelf_stdout_sha256": hashlib.sha256(parsed.stdout).hexdigest(),
            "readelf_stderr_sha256": hashlib.sha256(parsed.stderr).hexdigest(),
            "readelf_stderr_bytes": len(parsed.stderr),
        })
    print(json.dumps({
        "status": "PASS_EXTRACTED_CUBIN_STATIC_METADATA",
        "cubin_count": len(records),
        "architecture": "sm_120",
        "nonzero_executable_text_section_count": total_text,
        "extract_stdout_sha256": hashlib.sha256(extracted.stdout).hexdigest(),
        "extract_stderr_sha256": hashlib.sha256(extracted.stderr).hexdigest(),
        "records": records,
        "guest_tmpfs_only": True,
    }, sort_keys=True, separators=(",", ":")))
    '''
).lstrip()


class RevisionError(RuntimeError):
    pass


def sha256_regular(path: Path) -> str:
    info = path.lstat()
    if path.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise RevisionError(f"unsafe revision input: {path}")
    digest = hashlib.sha256()
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            digest.update(block)
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def render_profile() -> bytes:
    if sha256_regular(V1_PROFILE) != V1_PROFILE_SHA256:
        raise RevisionError("v1 exact profile hash drift")
    text = V1_PROFILE.read_text(encoding="utf-8")
    old_name = "r8-liquid-u3-gpu-static-audit-20260810T170339Z"
    if text.count(old_name) < 3 or text.count("/usr/bin/prlimit rix,") != 1:
        raise RevisionError("v1 profile anchors drift")
    text = text.replace(old_name, V2_PROFILE_NAME)
    text = text.replace(
        "  /usr/bin/prlimit rix,\n",
        "  /usr/bin/prlimit rix,\n  /usr/bin/python3 rix,\n  /usr/bin/python3.12 rix,\n",
    )
    if text.count("/usr/bin/python3 rix,") != 1 or text.count("/usr/bin/python3.12 rix,") != 1:
        raise RevisionError("Python helper execute delta is not exact")
    if any(token in text for token in ("/bin/sh ", "/usr/bin/make ", "/usr/local/cuda-12.8/bin/nvcc ", "/dev/nvidia")):
        raise RevisionError("v2 profile broadened to a forbidden tool/device")
    return text.encode("utf-8")


def render_policy(profile_payload: bytes) -> bytes:
    if sha256_regular(V1_POLICY) != V1_POLICY_SHA256:
        raise RevisionError("v1 static-audit policy hash drift")
    if sha256_regular(V1_FAILURE) != V1_FAILURE_SHA256:
        raise RevisionError("v1 failure lifecycle hash drift")
    lifecycle = json.loads(V1_FAILURE.read_text(encoding="utf-8"))
    if (
        lifecycle.get("status") != "FAIL_STATIC_AUDIT_PROFILE_LIFECYCLE_OR_GATE"
        or lifecycle.get("child_returncode") != 2
        or lifecycle.get("zero_residue") is not True
        or "cuobjdump_dump_elf rc=-9" not in json.dumps(lifecycle)
    ):
        raise RevisionError("v1 failure is not the exact bounded-output root cause")
    if V1_OVERSIZE_LOG.lstat().st_size != 268435456:
        raise RevisionError("v1 raw dump did not hit the exact 256 MiB limit")

    policy = copy.deepcopy(json.loads(V1_POLICY.read_text(encoding="utf-8")))
    policy["schema_version"] = "r8-liquid-gpu-static-audit-policy-v2"
    policy["policy_id"] = "LIQUID_ZRJ_MSI_U2404_GPU_STATIC_AUDIT_20260810T170339Z_V2"
    policy["revision_evidence"] = {
        "parent_policy": {"path": str(V1_POLICY), "sha256": V1_POLICY_SHA256},
        "parent_profile": {"path": str(V1_PROFILE), "sha256": V1_PROFILE_SHA256},
        "failed_lifecycle": {"path": str(V1_FAILURE), "sha256": V1_FAILURE_SHA256},
        "oversize_output": {
            "path": str(V1_OVERSIZE_LOG),
            "size_bytes": 268435456,
            "sha256": sha256_regular(V1_OVERSIZE_LOG),
        },
        "root_cause": "RAW_CUOBJDUMP_DUMP_ELF_EXCEEDED_FROZEN_256_MIB_OUTPUT_LIMIT",
        "single_semantic_delta": "EXTRACT_11_CUBINS_TO_GUEST_TMPFS_AND_EMIT_BOUNDED_READELF_JSON",
        "candidate_rebuilt": False,
        "candidate_executed": False,
        "network_or_gpu_added": False,
    }
    policy["implementation"]["gate_path"] = (
        "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/"
        "r8_liquid_gpu_static_audit_gate_v2.py"
    )
    policy["implementation"]["tests_path"] = (
        "src/scout_apps/simulation/scout_dualsphysics_liquid/tests/"
        "test_gpu_static_audit_gate_v2.py"
    )
    policy["implementation"]["supervisor_path"] = (
        "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/"
        "r8_liquid_gpu_static_audit_supervisor_v2.py"
    )
    policy["implementation"]["profile_generator"] = {
        "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_gpu_static_audit_revision_v2_generator.py",
        "sha256": sha256_regular(Path(__file__).resolve()),
    }
    policy["profile"] = {
        "name": V2_PROFILE_NAME,
        "path": f"src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/{V2_PROFILE_NAME}.profile",
        "mode_octal": "0600",
        "size_bytes": len(profile_payload),
        "sha256": hashlib.sha256(profile_payload).hexdigest(),
    }
    policy["trusted_tools"].extend(
        [
            {
                "path": "/usr/bin/python3",
                "realpath": "/usr/bin/python3.12",
                "sha256": "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118",
            },
            {
                "path": "/usr/bin/python3.12",
                "realpath": "/usr/bin/python3.12",
                "sha256": "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118",
            },
        ]
    )
    matches = [item for item in policy["candidate_commands"] if item["id"] == "cuobjdump_dump_elf"]
    if len(matches) != 1:
        raise RevisionError("v1 dump-elf command anchor drift")
    matches[0]["argv"] = ["/usr/bin/python3", "-I", "-S", "-c", CUBIN_HELPER]
    policy["outputs"] = {
        "evidence_root": "/home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2.evidence",
        "receipt": "/home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2.json",
        "lifecycle_receipt": "/home/zrj/scout_liquid_lab/audits/u3_source_gpu_build_sm120_20260810T170339Z_a_static_audit_v2_lifecycle.json",
    }
    return json.dumps(policy, sort_keys=True, indent=2, ensure_ascii=False).encode("utf-8") + b"\n"


def write_new(path: Path, payload: bytes, mode: int) -> None:
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )
    try:
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise RevisionError("short create-new revision write")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "write"))
    args = parser.parse_args()
    try:
        profile = render_profile()
        policy = render_policy(profile)
        if args.command == "write":
            if os.path.lexists(V2_PROFILE) or os.path.lexists(V2_POLICY):
                raise RevisionError("v2 profile/policy output already exists")
            write_new(V2_PROFILE, profile, 0o600)
            try:
                write_new(V2_POLICY, policy, 0o640)
            except Exception:
                # Preserve the already-created profile; no destructive rollback.
                raise
        print(json.dumps({
            "status": "PASS_STATIC_AUDIT_V2_REVISION_RENDER",
            "profile_path": str(V2_PROFILE),
            "profile_size": len(profile),
            "profile_sha256": hashlib.sha256(profile).hexdigest(),
            "policy_path": str(V2_POLICY),
            "policy_size": len(policy),
            "policy_sha256": hashlib.sha256(policy).hexdigest(),
            "helper_sha256": hashlib.sha256(CUBIN_HELPER.encode("utf-8")).hexdigest(),
            "expected_cubin_count": len(EXPECTED_CUBINS),
        }, sort_keys=True))
        return 0
    except (OSError, UnicodeError, ValueError, KeyError, json.JSONDecodeError, RevisionError) as exc:
        print(json.dumps({"status": "FAIL_STATIC_AUDIT_V2_REVISION_RENDER", "error": str(exc)}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
