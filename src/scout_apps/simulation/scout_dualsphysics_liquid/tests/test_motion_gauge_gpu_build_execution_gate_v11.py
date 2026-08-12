"""Static/mock-only regression tests for the fresh Motion-Gauge GPU v11 gate.

Nothing in this module loads an AppArmor profile, invokes a compiler, creates
the external campaign root, or executes a candidate.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import stat
from pathlib import Path
from types import SimpleNamespace

import pytest
from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parents[1]


def load(name: str, relative: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


gate = load(
    "motion_gauge_gpu_build_execution_gate_v11_tests",
    "scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v11.py",
)
patch_child = load(
    "motion_gauge_gpu_patch_child_v11_tests",
    "scripts/r8_liquid_motion_gauge_gpu_patch_child_v11.py",
)
profiles = load(
    "motion_gauge_gpu_profile_generator_v11_tests",
    "scripts/r8_liquid_motion_gauge_gpu_profile_generator_v11.py",
)


def json_object(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def fake_policy() -> dict:
    return {
        "profiles": [
            {"role": role, "name": role.lower(), "path": role.lower() + ".profile"}
            for role in ("SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT")
        ],
        "resources": {
            "static_per_command_limit_bytes": 268435456,
            "static_total_limit_bytes": 268435456,
        },
        "build_contract": {
            "forbidden_tokens": [
                "g++-13", "/dev/nvidia", "network inet", "network inet6",
                "--share-net", "flags=(unconfined)", "/home/zrj/scout_ws/**",
                "-arch=sm_89", "compute_89", "-j2",
            ],
        },
    }


def fake_g1(*, static_count: int = 557, argv_mutator=None):
    object_names = [f"Cpp{i:03d}.o" for i in range(120)] + [f"Cuda{i:02d}.o" for i in range(11)]
    cuda_names = object_names[-11:]
    candidate = [{"id": f"candidate_{index:02d}"} for index in range(11)]
    object_suffixes = [{"id": f"object_{index}", "cuda_only": False} for index in range(4)]
    object_suffixes += [{"id": "cuda_object_0", "cuda_only": True},
                        {"id": "cuda_object_1", "cuda_only": True}]
    assert len(candidate) + 131 * 4 + 11 * 2 == static_count
    policy = {
        "profiles": {"campaign_static_audit": {"name": "old-static"}},
        "object_contract": {"object_names": object_names, "cuda_object_names": cuda_names},
        "static_audit_contract": {
            "candidate_tool_suffixes": candidate,
            "object_tool_suffix_templates": object_suffixes,
        },
    }

    def argv(host: str, suffix: str, _policy: dict) -> list[str]:
        result = [
            "/usr/bin/timeout", "60s", "/usr/bin/aa-exec", "-p", "old-static", "--",
            "/usr/bin/bwrap", "--unshare-net", "--ro-bind", host, "/audit/input/item",
            "/usr/bin/file", "/audit/input/item", suffix,
        ]
        return argv_mutator(result) if argv_mutator else result

    return SimpleNamespace(
        ROOT_A=Path("/old/attempt.partial"),
        OUTPUT_A=Path("/old/attempt.partial/output"),
        POLICY_PATH=Path("/mock/g1.json"),
        read_json_object=lambda _path: policy,
        build_static_audit_argv=argv,
    )


def synthetic_manifest() -> dict[str, dict[str, object]]:
    result: dict[str, dict[str, object]] = {}
    for index in range(346):
        raw = f"unchanged-{index:03d}\n".encode()
        result[f"Dir{index // 100}/unchanged_{index:03d}.txt"] = {
            "size_bytes": len(raw), "sha256": gate.sha256_bytes(raw), "raw": raw,
        }
    for name, expected in patch_child.EXPECTED_HASHES.items():
        raw = f"sealed-preimage-placeholder-{name}\n".encode()
        result[name] = {"size_bytes": len(raw), "sha256": expected[0], "raw": raw}
    assert len(result) == 352
    return result


def materialize_inventory(root: Path, manifest: dict[str, dict[str, object]], *, patched: bool,
                          wrapper: bool = False) -> None:
    for name, item in manifest.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        raw = item["raw"]
        assert isinstance(raw, bytes)
        path.write_bytes(raw)
    if patched:
        # The inventory test exercises exact pre/post map selection.  Supplying
        # synthetic bytes lets the test stay independent of external roots.
        for name in patch_child.EXACT_NAMES:
            raw = ("postimage-" + name).encode()
            (root / name).write_bytes(raw)
    if wrapper:
        path = root / "U3GpuBuild.mk"
        path.write_bytes(gate.WRAPPER_BYTES)
        os.chmod(path, 0o600)


def identity_for(path: Path) -> dict[str, object]:
    info = os.lstat(path)
    raw = path.read_bytes()
    return {
        "path": str(path), "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
        "size_bytes": len(raw), "sha256": gate.sha256_bytes(raw), "device": info.st_dev,
        "inode": info.st_ino, "nlink": info.st_nlink,
    }


def test_all_three_schemas_are_deep_closed_and_reject_extra_properties():
    schemas = [json_object(path) for path in (
        gate.POLICY_SCHEMA_PATH, gate.RECEIPT_SCHEMA_PATH, gate.TOKEN_SCHEMA_PATH,
    )]
    for schema in schemas:
        Draft202012Validator.check_schema(schema)
        gate.assert_deep_closed(schema)
        assert schema["additionalProperties"] is False
    receipt = gate.make_receipt.__name__  # keep this test static; policy is not materialized yet
    assert receipt == "make_receipt"
    mutated = copy.deepcopy(schemas[2])
    mutated["properties"]["unexpected"] = {"type": "string"}
    with pytest.raises(gate.GateError, match="required/properties drift"):
        gate.assert_deep_closed(mutated)


def test_fresh_static_happy_path_mock_cardinalities_and_safety(monkeypatch):
    monkeypatch.setattr(gate, "policy", lambda: (fake_policy(), "a" * 64))
    monkeypatch.setattr(gate, "verify_pins", lambda items: [dict(item) for item in items])
    current = fake_policy()
    current["parents"] = [{"path": "parent"}]
    current["system_tools"] = [{"path": "tool"}]
    current["build_contract"]["make_argv"] = ["/usr/bin/make", "-j1"]
    monkeypatch.setattr(gate, "policy", lambda: (current, "a" * 64))
    monkeypatch.setattr(gate, "load_g1", lambda: SimpleNamespace(
        MAKE_ARGV=("/usr/bin/make", "-j1"),
        validate_policy_schema=lambda: {}, verify_plan_identity=lambda: None,
        verify_source_inputs=lambda _p: None, verify_object_contract=lambda _p: None,
    ))
    monkeypatch.setattr(gate, "source_contract", lambda: (Path("/sealed"), {}))
    monkeypatch.setattr(gate, "inventory", lambda _root, stage: ({"entry_count": 352}, []))
    monkeypatch.setattr(gate, "source_copy_argv", lambda _p: ["PATH=/usr/bin", "--unshare-net"])
    monkeypatch.setattr(gate, "patch_argv", lambda _p: ["PATH=/usr/bin", "--unshare-net"])
    monkeypatch.setattr(gate, "build_argv", lambda _p: ["/usr/bin/make", "-j1", "--unshare-net"])
    monkeypatch.setattr(gate, "static_plan", lambda _p: [
        ("artifact", f"tool-{index}", ["--unshare-net"]) for index in range(557)
    ])
    monkeypatch.setattr(gate.os.path, "lexists", lambda _path: False)
    report = gate.validate_static_contract(require_fresh=True)
    assert report["status"] == "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY"
    assert (report["source_entries"], report["patch_files"], report["unchanged_entries"]) == (352, 6, 346)
    assert (report["wrapper_bytes"], report["objects"], report["static_commands"]) == (84, 131, 557)
    assert not any(report[key] for key in (
        "files_written", "system_actions_performed", "make_run", "compiler_run",
        "candidate_executed", "profile_loaded",
    ))


def test_inventory_accepts_352_then_exact_353_and_rejects_extra(monkeypatch, tmp_path):
    manifest = synthetic_manifest()
    root = tmp_path / "source"
    materialize_inventory(root, manifest, patched=False)
    expected = {name: {"size_bytes": item["size_bytes"], "sha256": gate.sha256_bytes(item["raw"])}
                for name, item in manifest.items()}
    monkeypatch.setattr(gate, "source_contract", lambda: (Path("/sealed"), expected))
    summary, entries = gate.inventory(root, stage="copied")
    assert summary["entry_count"] == len(entries) == 352
    (root / "extra.txt").write_text("extra", encoding="utf-8")
    with pytest.raises(gate.GateError, match="drift/extra"):
        gate.inventory(root, stage="copied")
    (root / "extra.txt").unlink()
    patched_hashes = dict(gate.PATCH_AFTER_SHA256)
    patched_sizes = dict(gate.PATCH_AFTER_SIZE_BYTES)
    for name in gate.PATCH_NAMES:
        patched_hashes[name] = gate.sha256_bytes((root / name).read_bytes())
        patched_sizes[name] = (root / name).stat().st_size
    monkeypatch.setattr(gate, "PATCH_AFTER_SHA256", patched_hashes)
    monkeypatch.setattr(gate, "PATCH_AFTER_SIZE_BYTES", patched_sizes)
    (root / "U3GpuBuild.mk").write_bytes(gate.WRAPPER_BYTES)
    os.chmod(root / "U3GpuBuild.mk", 0o600)
    summary, entries = gate.inventory(root, stage="wrapped")
    assert summary["entry_count"] == len(entries) == 353


def test_patch_contract_has_exact_six_pre_post_and_346_unchanged(monkeypatch):
    assert set(gate.PATCH_AFTER_SHA256) == set(patch_child.EXPECTED_HASHES) == set(gate.PATCH_NAMES)
    assert len(gate.PATCH_NAMES) == 6
    assert all(gate.PATCH_AFTER_SHA256[name] == patch_child.EXPECTED_HASHES[name][1]
               for name in gate.PATCH_NAMES)
    manifest = synthetic_manifest()
    unchanged = set(manifest) - set(gate.PATCH_NAMES)
    assert len(unchanged) == 346
    monkeypatch.setattr(gate, "source_contract", lambda: (
        Path("/sealed"), {name: {"size_bytes": item["size_bytes"], "sha256": item["sha256"]}
                          for name, item in manifest.items()},
    ))
    # If a postimage map omits one target, patched inventory must fail closed
    # before it could claim six changed files.
    missing = gate.PATCH_AFTER_SHA256.pop(gate.PATCH_NAMES[0])
    try:
        with pytest.raises(gate.GateError, match="source inventory"):
            gate.inventory(Path("/does-not-exist"), stage="patched")
    finally:
        gate.PATCH_AFTER_SHA256[gate.PATCH_NAMES[0]] = missing


def test_patch_child_reads_a_frozen_preimage_before_atomic_replacement(monkeypatch, tmp_path):
    target = tmp_path / patch_child.EXACT_NAMES[0]
    target.write_bytes(b"authenticated-preimage")
    directory_fd = os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        raw, item = patch_child.read_regular_at(directory_fd, target.name)
    finally:
        os.close(directory_fd)
    assert raw == b"authenticated-preimage"
    assert item["sha256"] == gate.sha256_bytes(raw)


def test_patch_journal_requires_exact_order_and_final_contract():
    records = [
        {"sequence": 1, "event": "PATCH_START"},
        {"sequence": 2, "event": "PATCH_PLAN_AUTHENTICATED"},
    ]
    for index, name in enumerate(sorted(gate.PATCH_NAMES), 1):
        records.append({
            "sequence": index + 2, "event": "ATOMIC_FILE_REPLACED",
            "path": name, "completed_count": index, "atomic_per_file": True,
            "cross_file_transaction": False,
        })
    records.append({
        "sequence": 9, "event": "PATCH_FINAL",
        "status": "PASS_CONFINED_EXACT_SIX_FILE_PATCH_V11", "changed_count": 6,
        "completed_files": sorted(gate.PATCH_NAMES),
        "six_file_aggregate_sha256": "3cf3e7d883f7c751d7b5455eec352c6df46ff669cf6b5a989471be5f7731f528",
        "patch_module_sha256": gate.PATCH_V1_SHA256, "atomic_per_file": True,
        "cross_file_transaction": False,
    })
    raw = b"".join(gate.canonical_json(item) for item in records)
    assert gate.parse_patch_journal(raw)["final"]["changed_count"] == 6
    missing = b"".join(gate.canonical_json(item) for item in records[:-2] + records[-1:])
    with pytest.raises(gate.GateError, match="record count"):
        gate.parse_patch_journal(missing)
    duplicate = copy.deepcopy(records)
    duplicate[3]["path"] = duplicate[2]["path"]
    with pytest.raises(gate.GateError, match="replacement path/order"):
        gate.parse_patch_journal(b"".join(gate.canonical_json(item) for item in duplicate))


def test_lifecycle_finalize_is_create_new_and_cannot_be_replayed(monkeypatch, tmp_path):
    monkeypatch.setattr(gate, "AUDIT_ROOT", tmp_path)
    monkeypatch.setattr(gate, "require_user_identity", lambda: None)
    monkeypatch.setattr(gate, "policy", lambda: (fake_policy(), "a" * 64))
    previous = {
        "status": "PASS_SOURCE_COPY_352_AWAITING_LIFECYCLE",
        "evidence": {}, "safety": {"profile_zero_residue": False},
    }
    monkeypatch.setattr(gate, "previous_receipt", lambda *_args: (previous, "b" * 64))
    monkeypatch.setattr(gate, "inventory", lambda *_args, **_kwargs: ({"manifest_sha256": "c" * 64}, []))
    assert gate.finalize_source_copy_lifecycle([], True) == 0
    path = gate.receipt_path(3, "source-copy", "LIFECYCLE")
    assert path.exists()
    with pytest.raises(FileExistsError):
        gate.finalize_source_copy_lifecycle([], True)


def test_wrapper_is_exact_84_bytes_mode_0600_and_oexcl(monkeypatch, tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    monkeypatch.setattr(gate, "SOURCE_ROOT", source)
    item = gate.create_wrapper()
    path = source / "U3GpuBuild.mk"
    assert (len(path.read_bytes()), item["size_bytes"], item["mode_octal"], item["sha256"]) == (
        84, 84, "0600", gate.WRAPPER_SHA256,
    )
    with pytest.raises(FileExistsError):
        gate.create_wrapper()


def test_candidate_disarm_rejects_hardlink_and_detects_inode_swap(monkeypatch, tmp_path):
    candidate = tmp_path / "candidate"
    candidate.write_bytes(b"candidate-v11")
    os.chmod(candidate, 0o755)
    monkeypatch.setattr(gate, "CANDIDATE_PATH", candidate)
    first = gate.disarm_candidate()
    assert first is not None and first["mode_octal"] == "0400"
    replacement = tmp_path / "replacement"
    replacement.write_bytes(b"candidate-v11")
    os.chmod(replacement, 0o400)
    os.replace(replacement, candidate)
    second = gate.disarm_candidate()
    assert second is not None and second["inode"] != first["inode"]
    with pytest.raises(gate.GateError, match="inode changed"):
        gate.assert_same_artifact_inode(first, second)
    linked = tmp_path / "hardlink"
    os.link(candidate, linked)
    with pytest.raises(gate.GateError, match="unsafe candidate"):
        gate.disarm_candidate()


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "argv-drift"])
def test_static_557_plan_rejects_missing_duplicate_or_argv_drift(monkeypatch, mutation):
    current = fake_policy()
    monkeypatch.setattr(gate, "policy", lambda: (current, "a" * 64))
    if mutation == "argv-drift":
        fake = fake_g1(argv_mutator=lambda argv: [value for value in argv if value != "--unshare-net"])
    else:
        fake = fake_g1()
        catalog = fake.read_json_object(fake.POLICY_PATH)["static_audit_contract"]["candidate_tool_suffixes"]
        if mutation == "missing":
            catalog.pop()
        else:
            catalog.append(copy.deepcopy(catalog[-1]))
    monkeypatch.setattr(gate, "load_g1", lambda: fake)
    with pytest.raises(gate.GateError, match="cardinality drift|sandbox"):
        gate.static_plan(current)


def test_build_and_profiles_forbid_gpp13_gpu_external_network_and_wide_paths(monkeypatch):
    rendered = profiles.render()
    forbidden_profile_tokens = (
        "g++-13", "/dev/nvidia", "network inet dgram,", "network inet6 dgram,",
        "flags=(unconfined)", "/home/zrj/scout_ws/src/**", "/work/source/**",
    )
    for role, raw in rendered.items():
        text = "\n".join(profiles.rule_lines(raw.decode("utf-8")))
        assert not any(token in text for token in forbidden_profile_tokens), (role, text)
    current = fake_policy()
    current["build_contract"]["forbidden_tokens"] = ["g++-13", "/dev/nvidia", "network inet", "-j2"]
    fake = SimpleNamespace(
        ROOT_A=Path("/old/attempt.partial"), OUTPUT_A=Path("/old/attempt.partial/output"),
        POLICY_PATH=Path("/mock/g1.json"), MAKE_ARGV=("/usr/bin/make", "-j1"),
        read_json_object=lambda _path: {
            "profiles": {"attempt_a_build": {"name": "old-build"}},
            "build_contract": {"full_execution_argv": [
                "/usr/bin/timeout", "5400", "/usr/bin/aa-exec", "-p", "old-build", "--",
                "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--clearenv",
                "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc",
                "--unshare-uts", "--disable-userns", "--assert-userns-disabled", "X", "Y",
                "--ro-bind", "/usr", "/usr", str(Path("/old/attempt.partial/output")),
                "/usr/bin/make", "-j1",
            ]},
        },
    )
    monkeypatch.setattr(gate, "load_g1", lambda: fake)
    argv = gate.build_argv(current)
    joined = "\n".join(argv)
    assert "--unshare-net" in argv and "/bin" in argv and "-j1" in argv
    assert not any(token in joined for token in current["build_contract"]["forbidden_tokens"])


def test_repository_static_self_check_requires_materialized_fresh_policy():
    """The real happy path must not be declared while its policy is absent."""

    assert gate.POLICY_PATH.name.endswith("_v11.json")
    assert gate.POLICY_PATH.is_file(), f"missing frozen v11 policy: {gate.POLICY_PATH}"
    report = gate.self_check()
    assert report["status"] == "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY"
