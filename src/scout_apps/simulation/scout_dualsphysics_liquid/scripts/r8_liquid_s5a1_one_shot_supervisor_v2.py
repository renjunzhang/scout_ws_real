#!/usr/bin/env python3
"""Static-first S5A1 v2 one-shot supervisor; no run is admitted by self-check."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v2_policy.json"
SCHEMA = ROOT / "schema/target_host_s5a1_static_admission_v2.json"
WORKER = ROOT / "scripts/r8_liquid_s5a1_extractor_worker_v2.py"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T193016Z_v2"
EXPECTED = {
    POLICY: "b4783229be62bb981d529cf365ba610d134b9e5aa9f34f37c4c233250534499d",
    SCHEMA: "566da6d5ef548ed5197e1b4451df6bcaafaefbeab348d77016bf3d66cb305cbd",
    WORKER: "795f2983a0438adbb2f2a6e2a5fc006f2b64f51e45e1037a198bece6c8e9e465",
}


class SupervisorV2Error(ValueError):
    pass


def sha256_regular(path: Path, maximum: int = 4 * 1024 * 1024) -> str:
    metadata = path.stat()
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > maximum:
        raise SupervisorV2Error(f"unsafe static input: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_static() -> tuple[dict[str, Any], dict[str, Any], str]:
    for path, digest in EXPECTED.items():
        if sha256_regular(path) != digest:
            raise SupervisorV2Error(f"static parent hash drift: {path}")
    policy = json.loads(POLICY.read_bytes())
    schema = json.loads(SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    if policy["execution_id"] != EXECUTION_ID or policy["selection"]["optional_authorized"] is not False:
        raise SupervisorV2Error("execution identity or primary-only selection drifted")
    for section in (policy["s5a0_v2"], policy["v1_parents"], policy["tools"]):
        for value in section.values():
            if isinstance(value, Mapping) and "path" in value and sha256_regular(Path(value["path"]), 64 * 1024 * 1024) != value["sha256"]:
                raise SupervisorV2Error(f"parent/tool hash drift: {value['path']}")
    return policy, schema, hashlib.sha256(POLICY.read_bytes()).hexdigest()


def build_bwrap_argv(policy: Mapping[str, Any]) -> list[str]:
    limits = policy["execution_contract"]
    argv = [
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all", "--unshare-net",
        "--clearenv", "--cap-drop", "ALL", "--ro-bind", "/usr", "/usr", "--tmpfs", "/tmp",
        "--dir", "/app", "--ro-bind", str(WORKER), "/app/worker.py",
        "--setenv", "PATH", "/usr/bin", "--setenv", "HOME", "/nonexistent",
        "--setenv", "PYTHONDONTWRITEBYTECODE", "1", "--setenv", "S5A1_V2_SANDBOX", "1",
        "--", "/usr/bin/timeout", "--foreground", "--signal=KILL", "--kill-after=2s", "180s",
        "/usr/bin/prlimit", "--rss=536870912:536870912", "--as=805306368:805306368",
        f"--fsize={max(limits['stdout_max_bytes'], limits['stderr_max_bytes'])}:{max(limits['stdout_max_bytes'], limits['stderr_max_bytes'])}",
        "--nofile=64:64", "--nproc=1:1", *policy["sandbox"]["python_argv"],
        "--token", f"{EXECUTION_ID}:sandbox-worker",
    ]
    if any(token in argv for token in ("--proc", "--dev", "sudo", "nvidia-smi")):
        raise SupervisorV2Error("forbidden sandbox exposure")
    if argv.count("--cap-drop") != 1 or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise SupervisorV2Error("capability drop differs")
    if policy["sandbox"]["python_argv"][:2] != ["/usr/bin/python3", "-I"] or "-B" not in policy["sandbox"]["python_argv"]:
        raise SupervisorV2Error("isolated Python argv differs")
    return argv


def reserve_failure_receipt(path: Path) -> int:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    os.fchmod(descriptor, 0o600)
    return descriptor


def publish_reserved(descriptor: int, partial: Path, final: Path, value: Mapping[str, Any]) -> None:
    raw = json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")).encode() + b"\n"
    if len(raw) > 65536:
        raise SupervisorV2Error("reserved failure receipt exceeds bound")
    view = memoryview(raw)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            raise SupervisorV2Error("reserved receipt write stalled")
        view = view[written:]
    os.fsync(descriptor)
    os.close(descriptor)
    os.link(partial, final, follow_symlinks=False)
    os.unlink(partial)
    parent_fd = os.open(final.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def self_check() -> dict[str, Any]:
    policy, schema, policy_sha = load_static()
    build_bwrap_argv(policy)
    final_present = os.path.lexists(policy["s5a0_v2"]["outer_final_receipt"])
    result = {
        "schema_version": "smpcc-r8-liquid-s5a1-static-admission-v2",
        "execution_id": EXECUTION_ID,
        "status": "READY_FOR_SEPARATE_ONE_SHOT_AUTHORIZATION" if final_present else "NOT_ADMITTED_S5A0_V2_FINAL_RECEIPT_ABSENT",
        "policy_sha256": policy_sha,
        "parents_verified": True,
        "s5a0_outer_final_present": final_present,
        "real_input_read": False,
        "external_write": False,
        "bwrap_run": False,
        "next": "NEW_EXACT_S5A1_V2_EXECUTION_AUTHORIZATION_REQUIRED" if final_present else "WAIT_FOR_S5A0_V2_FINAL_RECEIPT",
    }
    Draft202012Validator(schema).validate(result)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    run = commands.add_parser("run")
    run.add_argument("--authorize-execution-id", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "run":
            if args.authorize_execution_id != EXECUTION_ID:
                raise SupervisorV2Error("exact execution authorization differs")
            state = self_check()
            raise SupervisorV2Error(f"execution not admitted in static revision: {state['status']}")
        result = self_check()
    except (SupervisorV2Error, OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
