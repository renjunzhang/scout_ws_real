"""Static, non-executing checks for the U3 C1M GenCase seed v3 gate."""

from __future__ import annotations

import ast
import copy
import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts/r8_liquid_target_u3_gencase_seed_gate_v3.py"
POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_seed_materialization_policy_v3.json"
SCHEMA = PACKAGE / "schema/target_host_u3_gencase_seed_materialization_policy_v3.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gencase_seed_gate_v3_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_template_mutation_rejected(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mutate: Callable[[ET.Element], None],
) -> None:
    gate = load_gate()
    original = gate.CASE_TEMPLATE.read_bytes()
    root = ET.fromstring(original)
    mutate(root)
    changed = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    assert changed != original
    candidate = tmp_path / "C1M_changed.xml"
    candidate.write_bytes(changed)
    monkeypatch.setattr(gate, "CASE_TEMPLATE", candidate)
    monkeypatch.setattr(gate, "CASE_TEMPLATE_SHA256", hashlib.sha256(changed).hexdigest())
    monkeypatch.setattr(gate, "CASE_TEMPLATE_SIZE_BYTES", len(changed))
    with pytest.raises(gate.GateError, match="C1M"):
        gate.template_facts()


def test_review_policy_schema_and_predecessor_evidence_pass() -> None:
    gate = load_gate()
    review = gate.verify_review_artifacts()
    template = review["template"]
    predecessors = review["nonreusable_predecessors"]
    assert template["sha256"] == gate.CASE_TEMPLATE_SHA256
    assert template["moving_reference"] == "0"
    assert template["motion_begin"] == {"mov": "1", "start": "0"}
    assert template["motion"] == "mvnull_id_1"
    assert template["shifting"] == "1"
    assert template["dt_all_particles"] == "1"
    assert predecessors["rejected_v1"]["receipt_sha256"] == gate.V1_RECEIPT_SHA256
    assert predecessors["fixed_boundary_v2"]["receipt_sha256"] == gate.V2_RECEIPT_SHA256
    assert predecessors["rejected_v1"]["used_as_v3_source"] is False
    assert predecessors["fixed_boundary_v2"]["used_as_v3_source"] is False


def test_policy_and_schema_are_closed_exact_v3_contracts() -> None:
    gate = load_gate()
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert policy == gate.expected_policy()
    assert schema["additionalProperties"] is False
    assert set(policy) == set(schema["required"]) == set(schema["properties"])
    gate.validate_schema_instance(policy, schema)
    assert policy["materialization_contract"]["predecessor_seeds_as_source"] == "forbidden"
    assert policy["materialization_contract"]["case_source"] == (
        "pinned_workspace_c1m_moving_zero_v1_template_only"
    )
    assert policy["source_provenance"]["case_template"]["host_filename"] == (
        "C1M_moving_zero_Def.xml"
    )


def test_schema_rejects_changed_identity_predecessor_or_extra_field() -> None:
    gate = load_gate()
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))

    changed_identity = copy.deepcopy(gate.expected_policy())
    changed_identity["frozen_attempt"]["seed_id"] = gate.V2_SEED_ID
    with pytest.raises(gate.GateError):
        gate.validate_schema_instance(changed_identity, schema)

    changed_predecessor = copy.deepcopy(gate.expected_policy())
    changed_predecessor["nonreusable_predecessors"]["fixed_boundary_v2"][
        "is_v3_materialization_source"
    ] = True
    with pytest.raises(gate.GateError):
        gate.validate_schema_instance(changed_predecessor, schema)

    extra = copy.deepcopy(gate.expected_policy())
    extra["unreviewed_override"] = True
    with pytest.raises(gate.GateError):
        gate.validate_schema_instance(extra, schema)


def test_v3_identity_paths_are_disjoint_from_v1_and_v2() -> None:
    gate = load_gate()
    v3_paths = {gate.SEED_ROOT, gate.INPUT_ROOT, gate.RECEIPT, gate.PARTIAL_RECEIPT}
    v1_paths = {gate.V1_SEED_ROOT, gate.V1_INPUT_ROOT, gate.V1_RECEIPT, gate.V1_PARTIAL_RECEIPT}
    v2_paths = {gate.V2_SEED_ROOT, gate.V2_INPUT_ROOT, gate.V2_RECEIPT, gate.V2_PARTIAL_RECEIPT}
    assert v3_paths.isdisjoint(v1_paths)
    assert v3_paths.isdisjoint(v2_paths)
    assert gate.SEED_ID not in {gate.V1_SEED_ID, gate.V2_SEED_ID}
    predecessors = gate.expected_nonreusable_predecessors()
    assert predecessors["rejected_v1"]["is_v3_materialization_source"] is False
    assert predecessors["fixed_boundary_v2"]["is_v3_materialization_source"] is False


def test_gate_subprocess_surface_is_only_fixed_git_cat_file_blob() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    subprocess_calls: list[ast.Call] = []
    forbidden_calls: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
            qualified = f"{node.func.value.id}.{node.func.attr}"
            if qualified.startswith("subprocess."):
                subprocess_calls.append(node)
            if qualified in {
                "os.system",
                "os.execv",
                "os.execve",
                "os.spawnv",
                "os.spawnve",
                "os.posix_spawn",
            }:
                forbidden_calls.append(qualified)
    assert len(subprocess_calls) == 1
    call = subprocess_calls[0]
    assert isinstance(call.func, ast.Attribute) and call.func.attr == "run"
    assert call.args and isinstance(call.args[0], ast.List)
    command_source = ast.get_source_segment(source, call.args[0])
    assert command_source is not None
    assert "str(GIT_PATH)" in command_source
    assert '"--no-replace-objects"' in command_source
    assert 'f"--git-dir={BARE_REPOSITORY}"' in command_source
    assert '"cat-file"' in command_source
    assert '"blob"' in command_source
    assert 'str(entry["blob_sha1"])' in command_source
    assert "shell" not in {keyword.arg for keyword in call.keywords}
    assert forbidden_calls == []


def test_preflight_is_read_only_and_new_attempt_remains_absent() -> None:
    gate = load_gate()
    paths = (gate.SEED_ROOT, gate.RECEIPT, gate.PARTIAL_RECEIPT)
    before = {path: path.exists() for path in paths}
    assert before == {path: False for path in paths}
    result = gate.preflight()
    after = {path: path.exists() for path in paths}
    assert after == before
    assert result["predecessor_read_purpose"] == (
        "append_only_nonreusable_evidence_verification_only"
    )
    assert result["predecessor_used_as_materialization_source"] is False
    assert result["trusted_tools"]["git"]["sha256"] == gate.GIT_SHA256


def test_materialization_sources_exclude_v1_and_v2_seed_inputs() -> None:
    gate = load_gate()
    assert [entry["kind"] for entry in gate.SEED_FILES] == [
        "git_blob",
        "git_blob",
        "workspace_template_c1m_v1",
    ]
    assert gate.SEED_FILES[2]["destination"] == "C1M_moving_zero_Def.xml"
    assert gate.CASE_TEMPLATE not in {gate.V1_INPUT_ROOT, gate.V2_INPUT_ROOT}
    source = SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    materialization_functions = {
        node.name: ast.get_source_segment(source, node) or ""
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in {"materialize_git_blob", "materialize_template", "materialize"}
    }
    combined = "\n".join(materialization_functions.values())
    assert "V1_INPUT_ROOT" not in combined
    assert "V2_INPUT_ROOT" not in combined
    assert "expected_rejected_predecessor" not in source
    assert '"rejected_predecessor"' not in source
    assert source.count('"nonreusable_predecessors"') >= 3


def test_template_rejects_missing_motion(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(root: ET.Element) -> None:
        casedef = root.find("./casedef")
        motion = root.find("./casedef/motion")
        assert casedef is not None and motion is not None
        casedef.remove(motion)

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


def test_template_rejects_motion_under_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(root: ET.Element) -> None:
        casedef = root.find("./casedef")
        execution = root.find("./execution")
        motion = root.find("./casedef/motion")
        assert casedef is not None and execution is not None and motion is not None
        casedef.remove(motion)
        execution.append(motion)

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


def test_template_rejects_wrong_reference_or_multiple_objects(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def wrong_reference(root: ET.Element) -> None:
        objreal = root.find("./casedef/motion/objreal")
        assert objreal is not None
        objreal.set("ref", "1")

    assert_template_mutation_rejected(tmp_path, monkeypatch, wrong_reference)

    def multiple_objects(root: ET.Element) -> None:
        motion = root.find("./casedef/motion")
        objreal = root.find("./casedef/motion/objreal")
        assert motion is not None and objreal is not None
        motion.append(copy.deepcopy(objreal))

    assert_template_mutation_rejected(tmp_path, monkeypatch, multiple_objects)


@pytest.mark.parametrize(
    "xpath,attribute,replacement",
    [
        ("./casedef/motion/objreal/begin", "mov", "2"),
        ("./casedef/motion/objreal/begin", "start", "0.1"),
        ("./casedef/motion/objreal/mvnull", "id", "2"),
    ],
)
def test_template_rejects_wrong_zero_motion_attributes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    xpath: str,
    attribute: str,
    replacement: str,
) -> None:
    def mutate(root: ET.Element) -> None:
        element = root.find(xpath)
        assert element is not None
        element.set(attribute, replacement)

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


@pytest.mark.parametrize("replacement", ["wait", "mvrect", "mvfile", "mvrectsinu"])
def test_template_rejects_wait_file_rectilinear_or_sinusoidal_motion(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    replacement: str,
) -> None:
    def mutate(root: ET.Element) -> None:
        mvnull = root.find("./casedef/motion/objreal/mvnull")
        assert mvnull is not None
        mvnull.tag = replacement

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


@pytest.mark.parametrize(
    "key,replacement",
    [("Shifting", "2"), ("DtAllParticles", "0")],
)
def test_template_rejects_moving_boundary_solver_setting_drift(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    key: str,
    replacement: str,
) -> None:
    def mutate(root: ET.Element) -> None:
        parameter = root.find(f"./execution/parameters/parameter[@key='{key}']")
        assert parameter is not None
        parameter.set("value", replacement)

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


@pytest.mark.parametrize(
    "xpath,attribute,replacement",
    [
        ("./casedef/geometry/definition", "dp", "0.003"),
        ("./casedef/geometry/commands/mainlist/setmkbound", "mk", "1"),
        ("./casedef/geometry/commands/mainlist/drawcylinder[2]", "radius", "0.019"),
        ("./execution/parameters/simulationdomain/posmax", "x", "0.022"),
        ("./casedef/constantsdef/gravity", "z", "-9.8"),
        ("./casedef/constantsdef/rhop0", "value", "998"),
    ],
)
def test_template_rejects_geometry_dp_mk_domain_or_physics_drift(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    xpath: str,
    attribute: str,
    replacement: str,
) -> None:
    def mutate(root: ET.Element) -> None:
        element = root.find(xpath)
        assert element is not None
        element.set(attribute, replacement)

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


def test_template_rejects_extra_parameter(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(root: ET.Element) -> None:
        parameters = root.find("./execution/parameters")
        assert parameters is not None
        parameters.insert(len(parameters) - 1, ET.Element("parameter", key="Unreviewed", value="1"))

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


def test_template_rejects_external_asset(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(root: ET.Element) -> None:
        mainlist = root.find("./casedef/geometry/commands/mainlist")
        assert mainlist is not None
        mainlist.append(ET.Element("drawfilestl", file="unreviewed.stl"))

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)


def test_closed_xml_contract_rejects_unreviewed_extra_attribute(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def mutate(root: ET.Element) -> None:
        objreal = root.find("./casedef/motion/objreal")
        assert objreal is not None
        objreal.set("unreviewed", "true")

    assert_template_mutation_rejected(tmp_path, monkeypatch, mutate)
