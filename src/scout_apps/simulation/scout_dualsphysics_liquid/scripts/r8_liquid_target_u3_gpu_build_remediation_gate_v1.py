#!/usr/bin/env python3
"""Static-only gate and deterministic profile generator for fresh GPU campaigns.

This revision cannot create a campaign, build root, persistent profile instance,
receipt, or authorization hash.  It proves that the offline remediation template
is the frozen G1 build template plus exactly one audit-backed pre-pivot tmp rule.
The only executable command besides self-check is a non-loading AppArmor query
against an ephemeral, explicitly query-only rendering.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True

PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_DIR.parents[3]

PLAN_PATH = WORKSPACE_ROOT / (
    "docs/实物实验注意事项/对比试验/仿真接入液体/"
    "20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
)
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"

SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gpu_build_remediation_draft_v1.json"
SCHEMA_SHA256 = "ccd5cda44e5c5cdffbad3f144a305ebedab9f1bface2202db1073aa2d5895729"
DRAFT_PATH = PACKAGE_DIR / (
    "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_remediation_draft_v1.json"
)
DRAFT_SHA256 = "71fee5381730c42e1b1f02beb0d687e597de0a3a17a4d95e88a087e74c4781d1"
BASE_TEMPLATE_PATH = PACKAGE_DIR / (
    "config/apparmor_drafts/r8-liquid-u3-gpu-build-v1.profile.template"
)
BASE_TEMPLATE_SHA256 = "11c175f63c901b337c5e8899aceedbc898fde0716c07a323746d723b15021fc4"
REMEDIATION_TEMPLATE_PATH = PACKAGE_DIR / (
    "config/apparmor_drafts/r8-liquid-u3-gpu-build-remediation-v1.profile.template"
)
REMEDIATION_TEMPLATE_SHA256 = "dd28bd5e75c1fec0e4861390ac6d2c196412c1b9ff3de9b375bf2e8a9bf51429"

N1_EVIDENCE_INDEX = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "u3_source_gpu_build_sm120_20260810T102641Z_g7_timebox_evidence_index_v1.json"
)
N1_EVIDENCE_INDEX_SHA256 = "69c4eca4f944435562946d4e4ea0977802545addb45d5372400d3c8a79946334"
N1_FINAL_FAILURE = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "u3_source_gpu_build_sm120_20260810T102641Z_g7_final_failure_v1.json"
)
N1_FINAL_FAILURE_SHA256 = "4c07823fc2319e1f1973cc5042066ee079c3806fb10d1461600f708e57ed58e3"

APPARMOR_PARSER = Path("/usr/sbin/apparmor_parser")
APPARMOR_PARSER_SHA256 = "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"
QUERY_PROFILE_NAME = "r8-liquid-u3-gpu-build-remediation-query-only-v1"
QUERY_ATTEMPT_ROOT = Path(
    "/home/zrj/scout_liquid_lab/build/__r8_offline_query_only_not_materialized__.partial"
)

SOURCE_CAMPAIGN_ID = "u3_source_gpu_build_sm120_20260810T102641Z"
OLD_ROOT_A = Path(
    "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_a.partial"
)
OLD_ROOT_B = Path(
    "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_b.partial"
)
LIQUID_BUILD_ROOT = Path("/home/zrj/scout_liquid_lab/build")

PROFILE_TOKEN = "@@PROFILE_NAME@@"
ROOT_TOKEN = "@@ATTEMPT_ROOT@@"
ANCHOR_BEFORE = "  /newroot/work/ rw,\n"
ANCHOR_AFTER = "  /newroot/work/output/ rw,\n"
DELTA_RULE = "  /newroot/work/tmp/ rw,\n"
DELTA_RULE_SHA256 = "bb6f1474949a3cdc6c6cd8478df4d7966ff7c542b76461610c3af16a2b2fb6ba"

G1_PARENT_HASHES = {
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json": "f3831fdfe09716fe5b8f0371bda25d087a6f10e0b0570b90eebea2caf90b7f57",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_gpu_build_execution_policy_v1.json": "250d32def4a6a0c37405d8cc9d9920b8f5882abe7bb36de7ca95e7308de54cad",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_gpu_build_gate_v1.py": "e34ab0facc3c665c6befbb792c7344e0af6e652fd3c6b30dfa81ca8cbea5b502",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_gpu_build_gate_v1.py": "a26602c650557044bab6e84d54197f91b44e5e854548522043962c01b7b399e9",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-copy-v1.profile.template": "67359e87d8b243c9d31e03634998369ff6530bf8617ad3d5f9c5e4b0fc9b613b",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-build-v1.profile.template": "11c175f63c901b337c5e8899aceedbc898fde0716c07a323746d723b15021fc4",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-copy-20260810T102641Z-a.profile": "2461550a249284dafc72e473b745ed829463c239e93e805fff623e54a29de8e1",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T102641Z-a.profile": "fbcc9f417103f17f59003b6da1b8bb7f08e59b2a68015b7ac38cdf1db8c19b10",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T102641Z.profile": "d23da58b35c69f458d7f9a7fcf03b9fc2493283cd580191d58f7ced460b45168",
}

G2_PARENT_HASHES = {
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v2.json": "17472c30021bd70d81180fca6be1317a84cf5ae6f104e0c7cc707dc88ed4b58d",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_gpu_build_execution_receipt_v1.json": "10ede832aae19632b9b5543b9916a7d1803e210c743db90910a66a37a1e02173",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_gpu_build_gate_v2.py": "d9d02b2aa6a237ed49b7e64cbf0f19d769a32f22c372dbc56b18be47e376eb7f",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_gpu_build_gate_v2.py": "54e96cb0b90b56353419ca1cce6f181ca6ea024318a5f897e8e6d7f14b85c9e2",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_gpu_build_gate_v3.py": "2ea0eaa66a88a8c6fa448472e9c603f22f24691133e573abd14e3038ac4b62e7",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_gpu_build_gate_v3.py": "d60db7c0465fd063775b7edb0ba22f2f0313480972ee3edc16c075fa66b96618",
}

PREEXISTING_UNTRACKED = {
    *G1_PARENT_HASHES,
    *G2_PARENT_HASHES,
}
N2_NEW_FILES = {
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-build-remediation-v1.profile.template",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_remediation_draft_v1.json",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_gpu_build_remediation_draft_v1.json",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_gpu_build_remediation_gate_v1.py",
    "src/scout_apps/simulation/scout_dualsphysics_liquid/tests/test_target_u3_gpu_build_remediation_gate_v1.py",
}

PROFILE_NAME_RE = re.compile(r"^r8-liquid-u3-gpu-build-[a-z0-9][a-z0-9-]{0,95}$")


class RemediationError(RuntimeError):
    """Closed validation failure."""


def identity_slug(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_regular_file(path: Path, *, limit: int = 64 * 1024 * 1024) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > limit:
            raise RemediationError(f"unsafe immutable input: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                return digest.hexdigest()
            digest.update(block)
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RemediationError(f"JSON root is not an object: {path}")
    return value


def verify_hash(path: Path, expected: str) -> None:
    actual = sha256_regular_file(path)
    if actual != expected:
        raise RemediationError(f"SHA-256 drift: {path}: {actual}")


def verify_recursive_schema_closure(node: Any, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object":
            properties = node.get("properties")
            required = node.get("required")
            if node.get("additionalProperties") is not False or not isinstance(properties, dict):
                raise RemediationError(f"open object schema at {location}")
            if not isinstance(required, list) or set(required) != set(properties):
                raise RemediationError(f"required/properties drift at {location}")
        for key, value in node.items():
            verify_recursive_schema_closure(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            verify_recursive_schema_closure(value, f"{location}/{index}")


def verify_frozen_inputs() -> None:
    verify_hash(PLAN_PATH, PLAN_SHA256)
    for relative, digest in {**G1_PARENT_HASHES, **G2_PARENT_HASHES}.items():
        verify_hash(WORKSPACE_ROOT / relative, digest)
    verify_hash(N1_EVIDENCE_INDEX, N1_EVIDENCE_INDEX_SHA256)
    verify_hash(N1_FINAL_FAILURE, N1_FINAL_FAILURE_SHA256)
    verify_hash(BASE_TEMPLATE_PATH, BASE_TEMPLATE_SHA256)
    verify_hash(REMEDIATION_TEMPLATE_PATH, REMEDIATION_TEMPLATE_SHA256)
    verify_hash(SCHEMA_PATH, SCHEMA_SHA256)
    verify_hash(DRAFT_PATH, DRAFT_SHA256)
    verify_hash(APPARMOR_PARSER, APPARMOR_PARSER_SHA256)


def verify_schema_and_draft() -> dict[str, Any]:
    schema = read_json(SCHEMA_PATH)
    draft = read_json(DRAFT_PATH)
    Draft202012Validator.check_schema(schema)
    verify_recursive_schema_closure(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(draft), key=lambda item: list(item.path))
    if errors:
        first = errors[0]
        raise RemediationError(
            f"closed-schema rejection at /{'/'.join(map(str, first.path))}: {first.message}"
        )
    index = read_json(N1_EVIDENCE_INDEX)
    final = read_json(N1_FINAL_FAILURE)
    if (
        final.get("status") != "TIMEBOX_EXHAUSTED"
        or final.get("exact_blocker") != "T_PLUS_6_REACHED_WITHOUT_COMPLETE_CANDIDATE"
        or final.get("attempt_b", {}).get("retry_used") is not False
        or final.get("next_allowed_stage") != "FRESH_CAMPAIGN_REQUIRES_NEW_AUTHORIZATION"
    ):
        raise RemediationError("N1 final failure receipt drift")
    g4 = index.get("g4_evidence", {})
    if (
        g4.get("state") != "RETRY_B_ELIGIBLE"
        or g4.get("candidate") != "ABSENT"
        or g4.get("kernel_audit_normalized_line_sha256")
        != "6aff32b36b40233301ee853c23b9f98c925c48a42ca1232acef753e51b7b5b69"
    ):
        raise RemediationError("G4 source evidence drift")
    return draft


def verify_template_delta(base: bytes | None = None, remediated: bytes | None = None) -> dict[str, Any]:
    base = BASE_TEMPLATE_PATH.read_bytes() if base is None else base
    remediated = REMEDIATION_TEMPLATE_PATH.read_bytes() if remediated is None else remediated
    before = ANCHOR_BEFORE.encode("utf-8")
    after = ANCHOR_AFTER.encode("utf-8")
    rule = DELTA_RULE.encode("utf-8")
    anchor = before + after
    if base.count(anchor) != 1:
        raise RemediationError("base template anchor is not unique")
    expected = base.replace(anchor, before + rule + after, 1)
    if remediated != expected:
        raise RemediationError("remediation template is not the exact one-rule insertion")
    if remediated.count(rule) != 1 or sha256_bytes(rule) != DELTA_RULE_SHA256:
        raise RemediationError("exact tmp rule count/hash drift")
    base_lines = collections.Counter(base.splitlines(keepends=True))
    remediation_lines = collections.Counter(remediated.splitlines(keepends=True))
    inserted = list((remediation_lines - base_lines).elements())
    deleted = list((base_lines - remediation_lines).elements())
    if inserted != [rule] or deleted:
        raise RemediationError("semantic diff contains more than one insertion or any deletion")
    return {
        "operation": "INSERT_EXACT_APPARMOR_FILE_RULE",
        "path": "/newroot/work/tmp/",
        "access_kind": "rw_guest_tmp_only",
        "apparmor_rule_utf8": DELTA_RULE,
        "apparmor_rule_sha256": DELTA_RULE_SHA256,
        "insertion_count": 1,
        "deletion_count": 0,
        "base_template_sha256": sha256_bytes(base),
        "remediation_template_sha256": sha256_bytes(remediated),
    }


def validate_future_identity(profile_name: str, attempt_root: str) -> None:
    source_slug = identity_slug(SOURCE_CAMPAIGN_ID)
    if (
        not PROFILE_NAME_RE.fullmatch(profile_name)
        or SOURCE_CAMPAIGN_ID in profile_name
        or source_slug in identity_slug(profile_name)
    ):
        raise RemediationError("profile name is not a fresh-campaign build identity")
    if "query-only" not in profile_name and "remediation" in profile_name:
        raise RemediationError("persistent profile name cannot masquerade as an offline fixture")
    root = PurePosixPath(attempt_root)
    if (
        not root.is_absolute()
        or str(root) != attempt_root
        or root.parent != PurePosixPath(str(LIQUID_BUILD_ROOT))
        or not root.name.endswith(".partial")
        or SOURCE_CAMPAIGN_ID in attempt_root
        or source_slug in identity_slug(attempt_root)
        or root in {PurePosixPath(str(OLD_ROOT_A)), PurePosixPath(str(OLD_ROOT_B))}
    ):
        raise RemediationError("attempt root is not a fresh exact direct child of the build root")
    if os.path.lexists(attempt_root):
        raise RemediationError("offline rendering refuses an existing attempt root")


def active_lines(profile: bytes) -> list[str]:
    result: list[str] = []
    for raw in profile.decode("utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            result.append(line)
    return result


def validate_rendered_profile(profile: bytes, profile_name: str, attempt_root: str) -> None:
    text = profile.decode("utf-8")
    if PROFILE_TOKEN in text or ROOT_TOKEN in text:
        raise RemediationError("rendered profile retains a template token")
    if text.count(profile_name) != 3 or text.count(attempt_root) != 4:
        raise RemediationError("rendered identity replacement count drift")
    lines = active_lines(profile)
    if lines.count(DELTA_RULE.strip()) != 1:
        raise RemediationError("rendered profile lacks its single exact tmp rule")
    active = "\n".join(lines)
    forbidden = (
        "g++-13",
        "/dev/nvidia",
        "network stream",
        "network packet",
        "flags=(unconfined)",
        " ux,",
        "/home/** w",
    )
    if any(token in active for token in forbidden):
        raise RemediationError("rendered profile crosses a forbidden boundary")


def render_profile(profile_name: str, attempt_root: str) -> bytes:
    verify_template_delta()
    validate_future_identity(profile_name, attempt_root)
    template = REMEDIATION_TEMPLATE_PATH.read_text(encoding="utf-8")
    if template.count(PROFILE_TOKEN) != 3 or template.count(ROOT_TOKEN) != 4:
        raise RemediationError("template token count drift")
    rendered = template.replace(PROFILE_TOKEN, profile_name).replace(ROOT_TOKEN, attempt_root)
    value = rendered.encode("utf-8")
    validate_rendered_profile(value, profile_name, attempt_root)
    return value


def verify_git_boundary() -> dict[str, Any]:
    tracked = subprocess.run(
        ["git", "diff", "--name-only"], cwd=WORKSPACE_ROOT, check=True, capture_output=True, text=True
    ).stdout.splitlines()
    staged = subprocess.run(
        ["git", "diff", "--cached", "--name-only"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    status = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    unexpected_status = [line for line in status if not line.startswith("?? ")]
    untracked = {line[3:] for line in status if line.startswith("?? ")}
    expected = PREEXISTING_UNTRACKED | N2_NEW_FILES
    if tracked or staged or unexpected_status or untracked != expected:
        raise RemediationError(
            f"Git boundary drift: tracked={tracked} staged={staged} "
            f"unexpected={unexpected_status} missing={sorted(expected-untracked)} "
            f"extra={sorted(untracked-expected)}"
        )
    return {
        "tracked_diff_count": 0,
        "staged_diff_count": 0,
        "untracked_preexisting_count": len(PREEXISTING_UNTRACKED),
        "untracked_n2_count": len(N2_NEW_FILES),
    }


def verify_non_materialization() -> None:
    if os.path.lexists(OLD_ROOT_B) or os.path.lexists(QUERY_ATTEMPT_ROOT):
        raise RemediationError("offline remediation created or inherited a forbidden root")
    for path in PACKAGE_DIR.glob("config/apparmor_drafts/r8-liquid-u3-gpu-build-remediation-*.profile"):
        raise RemediationError(f"persistent remediation profile instance exists: {path}")


def self_check() -> dict[str, Any]:
    verify_frozen_inputs()
    draft = verify_schema_and_draft()
    delta = verify_template_delta()
    verify_non_materialization()
    git_boundary = verify_git_boundary()
    if draft["future_identity"] != {
        "campaign_id": "NOT_FROZEN",
        "build_id": "NOT_FROZEN",
        "attempt_root": "NOT_FROZEN",
        "profile_name": "NOT_FROZEN",
        "profile_instance_path": "NOT_MATERIALIZED",
        "profile_instance_sha256": "NOT_MATERIALIZED",
    }:
        raise RemediationError("future identity was fabricated")
    return {
        "status": "PASS_FRESH_CAMPAIGN_OFFLINE_REMEDIATION_SELF_CHECK",
        "remediation_id": draft["remediation_id"],
        "schema_sha256": SCHEMA_SHA256,
        "draft_sha256": DRAFT_SHA256,
        "base_template_sha256": BASE_TEMPLATE_SHA256,
        "remediation_template_sha256": REMEDIATION_TEMPLATE_SHA256,
        "parent_count": len(G1_PARENT_HASHES) + len(G2_PARENT_HASHES),
        "n1_evidence_index_sha256": N1_EVIDENCE_INDEX_SHA256,
        "n1_final_failure_sha256": N1_FINAL_FAILURE_SHA256,
        "semantic_delta": delta,
        "future_identity": draft["future_identity"],
        "git_boundary": git_boundary,
        "profile_instance_persisted": False,
        "profile_instance_authorization_eligible": False,
        "new_campaign_created": False,
        "build_root_created": False,
        "source_copied": False,
        "sudo_used": False,
        "profile_loaded": False,
        "make_nvcc_candidate_run": False,
        "next_allowed_stage": "FRESH_CAMPAIGN_AND_EXACT_PROFILE_HASH_USER_AUTHORIZATION_REQUIRED",
    }


def write_all(descriptor: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            raise RemediationError("short ephemeral profile write")
        view = view[count:]


def query_template() -> dict[str, Any]:
    self_check()
    before_status = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
    ).stdout
    fixture = render_profile(QUERY_PROFILE_NAME, str(QUERY_ATTEMPT_ROOT))
    with tempfile.TemporaryDirectory(prefix="r8-u3-gpu-remediation-query-") as directory:
        fixture_path = Path(directory) / "query-only.profile"
        descriptor = os.open(
            fixture_path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
        )
        try:
            write_all(descriptor, fixture)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        argv = [str(APPARMOR_PARSER), "-Q", "-K", "-T", "--", str(fixture_path)]
        completed = subprocess.run(
            argv,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            timeout=30,
            check=False,
            env={"PATH": "/usr/sbin:/usr/bin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8"},
        )
        if len(completed.stdout) > 65536 or len(completed.stderr) > 65536:
            raise RemediationError("AppArmor query output exceeds 64 KiB")
        if completed.returncode != 0:
            raise RemediationError(
                "non-loading AppArmor query failed: "
                + completed.stderr.decode("utf-8", "replace")[:4096]
            )
        if fixture_path.read_bytes() != fixture or stat.S_IMODE(fixture_path.stat().st_mode) != 0o600:
            raise RemediationError("ephemeral query fixture changed")
        query_result = {
            "argv": [
                str(APPARMOR_PARSER),
                "-Q",
                "-K",
                "-T",
                "--",
                "<EPHEMERAL_QUERY_ONLY_PROFILE>",
            ],
            "return_code": completed.returncode,
            "stdout_bytes": len(completed.stdout),
            "stdout_sha256": sha256_bytes(completed.stdout),
            "stderr_bytes": len(completed.stderr),
            "stderr_sha256": sha256_bytes(completed.stderr),
            "fixture_byte_identity_verified": True,
        }
    after_status = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
    ).stdout
    if before_status != after_status or os.path.lexists(QUERY_ATTEMPT_ROOT):
        raise RemediationError("query left a persistent file, root, or Git change")
    return {
        "status": "PASS_NON_LOADING_APPARMOR_REMEDIATION_TEMPLATE_QUERY",
        "query": query_result,
        "profile_loaded": False,
        "profile_unloaded": False,
        "sudo_used": False,
        "persistent_profile_instance": False,
        "future_identity_frozen": False,
        "authorization_eligible": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "query-template"))
    args = parser.parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else query_template()
    except (RemediationError, OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        print(
            json.dumps(
                {"status": "FAIL_OFFLINE_REMEDIATION_VALIDATION", "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
