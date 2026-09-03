"""Deterministic C++14 projection of a validated D4 artifact contract."""

from __future__ import annotations

import json
import re
from typing import Any

from .artifact_contract import (
    ArtifactContract,
    model_contract_json_raw_sha256,
    require_artifact_contract,
)

HEADER_GUARD = "SPMPC_LOCAL_PLANNER_MODEL_CONTRACT_GENERATED_H_"
HEADER_NAMESPACE = ("spmpc_local_planner", "mainline", "generated")


class ModelContractHeaderError(ValueError):
    """The artifact contract cannot be represented by the C++14 header."""


def _cpp_string(value: Any, label: str) -> str:
    if type(value) is not str:
        raise ModelContractHeaderError(f"{label} must be a string")
    return json.dumps(value, ensure_ascii=True)


def _identifier(prefix: str, value: Any) -> str:
    if type(value) is not str or not value:
        raise ModelContractHeaderError("contract names must be non-empty strings")
    words = re.findall(r"[A-Za-z]+|[0-9]+", value)
    if not words:
        raise ModelContractHeaderError(f"cannot map contract name to C++: {value!r}")
    suffix = "".join(
        word[0].upper() + word[1:] if not word.isdigit() else word for word in words
    )
    return f"k{prefix}{suffix}"


def _enumerators(
    kind: str,
    ordered: Any,
    offsets: Any,
) -> tuple[tuple[str, str, int], ...]:
    if type(ordered) is not list or type(offsets) is not dict:
        raise ModelContractHeaderError(f"{kind} layout is malformed")
    values: list[tuple[str, str, int]] = []
    identifiers: set[str] = set()
    for index, name in enumerate(ordered):
        identifier = _identifier(kind, name)
        if identifier in identifiers:
            raise ModelContractHeaderError(
                f"{kind} names collide after C++ identifier conversion"
            )
        identifiers.add(identifier)
        if offsets.get(name) != index:
            raise ModelContractHeaderError(f"{kind} offsets are not contiguous")
        values.append((identifier, name, index))
    if not values:
        raise ModelContractHeaderError(f"{kind} layout cannot be empty")
    return tuple(values)


def _enum_lines(name: str, values: tuple[tuple[str, str, int], ...]) -> list[str]:
    lines = [f"enum class {name} : std::size_t {{"]
    lines.extend(f"  {identifier} = {offset}U," for identifier, _, offset in values)
    lines.append("};")
    return lines


def _name_array_lines(
    name: str,
    dimension: str,
    values: tuple[tuple[str, str, int], ...],
) -> list[str]:
    lines = [f"constexpr const char* const {name}[{dimension}] = {{"]
    lines.extend(f"  {_cpp_string(original, name)}," for _, original, _ in values)
    lines.append("};")
    return lines


def _checked_document(value: ArtifactContract) -> dict[str, Any]:
    checked = require_artifact_contract(value)
    document = checked.to_dict()
    try:
        dimensions = document["dimensions"]
        layouts = document["layouts"]
        capacity = document["development_capacity"]
        outputs = document["contract_outputs"]
        artifact = document["artifact"]
        solver_library = artifact["solver_library"]
        identities = (document["semantic_identity"], document["artifact_identity"])
    except (
        KeyError,
        TypeError,
    ) as exc:  # pragma: no cover - contract validator owns this
        raise ModelContractHeaderError(
            "model contract projection fields are missing"
        ) from exc
    if any(
        type(item) is not dict
        for item in (
            dimensions,
            layouts,
            capacity,
            outputs,
            artifact,
            solver_library,
            *identities,
        )
    ):
        raise ModelContractHeaderError("model contract projection fields are malformed")
    return document


def _render_document(
    document: dict[str, Any], model_contract_json_raw_sha256_value: str
) -> str:
    dimensions = document["dimensions"]
    layouts = document["layouts"]
    capacity = document["development_capacity"]
    outputs = document["contract_outputs"]
    solver_library = document["artifact"]["solver_library"]
    state = _enumerators(
        "State",
        layouts["state"]["ordered"],
        layouts["state"]["offsets"],
    )
    control = _enumerators(
        "Control",
        layouts["control"]["ordered"],
        layouts["control"]["offsets"],
    )
    parameter = _enumerators(
        "Parameter",
        layouts["parameter"]["ordered"],
        layouts["parameter"]["offsets"],
    )
    block_identifiers: set[str] = set()
    block_lines: list[str] = []
    ordered_blocks = sorted(
        layouts["parameter_blocks"].items(),
        key=lambda item: item[1]["begin"],
    )
    for block_name, block in ordered_blocks:
        identifier = _identifier("ParameterBlock", block_name)
        if identifier in block_identifiers:
            raise ModelContractHeaderError(
                "parameter blocks collide after C++ identifier conversion"
            )
        block_identifiers.add(identifier)
        block_lines.extend(
            (
                f"constexpr std::size_t {identifier}Begin = {block['begin']}U;",
                (f"constexpr std::size_t {identifier}End = {block['end_exclusive']}U;"),
            )
        )

    lines = [
        "// Generated from model_contract.json. Do not edit.",
        f"#ifndef {HEADER_GUARD}",
        f"#define {HEADER_GUARD}",
        "",
        "#include <cstddef>",
        "",
    ]
    for namespace in HEADER_NAMESPACE:
        lines.append(f"namespace {namespace} {{")
    lines.extend(
        (
            "",
            f"constexpr char kModelId[] = {_cpp_string(document['model_id'], 'model_id')};",
            (
                "constexpr char kModelContractSemanticSha256[] = "
                f"{_cpp_string(document['semantic_identity']['sha256'], 'semantic identity')};"
            ),
            (
                "constexpr char kArtifactSha256[] = "
                f"{_cpp_string(document['artifact_identity']['sha256'], 'artifact identity')};"
            ),
            (
                "constexpr char kModelContractJsonRawSha256[] = "
                f"{_cpp_string(model_contract_json_raw_sha256_value, 'model contract raw identity')};"
            ),
            (
                "constexpr char kModelContractFilename[] = "
                f"{_cpp_string(outputs['model_contract_json'], 'model contract filename')};"
            ),
            (
                "constexpr char kModelContractHeaderFilename[] = "
                f"{_cpp_string(outputs['generated_cpp_header'], 'generated header filename')};"
            ),
            (
                "constexpr char kSolverLibraryRelativePath[] = "
                f"{_cpp_string(solver_library['relative_path'], 'solver library path')};"
            ),
            (
                "constexpr char kSolverLibraryRawSha256[] = "
                f"{_cpp_string(solver_library['raw_sha256'], 'solver library identity')};"
            ),
            (
                "constexpr std::size_t kSolverLibrarySizeBytes = "
                f"{solver_library['size_bytes']}U;"
            ),
            "",
            f"constexpr std::size_t N = {dimensions['N']}U;",
            f"constexpr std::size_t NX = {dimensions['NX']}U;",
            f"constexpr std::size_t NU = {dimensions['NU']}U;",
            f"constexpr std::size_t NP = {dimensions['NP']}U;",
            f"constexpr std::size_t NP_EXEC = {dimensions['NP_exec']}U;",
            (
                "constexpr std::size_t PARAMETER_VECTOR_COUNT = "
                f"{dimensions['parameter_vector_count']}U;"
            ),
            f"constexpr std::size_t R_V = {capacity['release_intervals']['v']}U;",
            (
                "constexpr std::size_t R_OMEGA = "
                f"{capacity['release_intervals']['omega']}U;"
            ),
            f"constexpr std::size_t D_V = {capacity['delay_state_count']['v']}U;",
            (
                "constexpr std::size_t D_OMEGA = "
                f"{capacity['delay_state_count']['omega']}U;"
            ),
            f"constexpr std::size_t NQ_V = {capacity['selector_width']['v']}U;",
            (
                "constexpr std::size_t NQ_OMEGA = "
                f"{capacity['selector_width']['omega']}U;"
            ),
            (
                "constexpr std::size_t EXECUTION_SUBSEGMENT_SLOTS = "
                f"{capacity['execution_subsegment_slots']}U;"
            ),
            "",
        )
    )
    lines.extend(_enum_lines("StateOffset", state))
    lines.append("")
    lines.extend(_enum_lines("ControlOffset", control))
    lines.append("")
    lines.extend(_enum_lines("ParameterOffset", parameter))
    lines.append("")
    lines.extend(block_lines)
    lines.append("")
    lines.extend(_name_array_lines("STATE_NAMES", "NX", state))
    lines.append("")
    lines.extend(_name_array_lines("CONTROL_NAMES", "NU", control))
    lines.append("")
    lines.extend(_name_array_lines("PARAMETER_NAMES", "NP", parameter))
    lines.extend(
        (
            "",
            'static_assert(N == 60U, "mainline horizon drifted");',
            'static_assert(NX == 48U, "mainline state dimension drifted");',
            'static_assert(NU == 3U, "mainline control dimension drifted");',
            'static_assert(NP == 162U, "mainline parameter dimension drifted");',
            'static_assert(NP_EXEC == 121U, "execution prefix dimension drifted");',
            'static_assert(PARAMETER_VECTOR_COUNT == N + 1U, "parameter row count drifted");',
            (
                "static_assert(static_cast<std::size_t>(StateOffset::"
                f'{state[-1][0]}) + 1U == NX, "state offsets drifted");'
            ),
            (
                "static_assert(static_cast<std::size_t>(ControlOffset::"
                f'{control[-1][0]}) + 1U == NU, "control offsets drifted");'
            ),
            (
                "static_assert(static_cast<std::size_t>(ParameterOffset::"
                f'{parameter[-1][0]}) + 1U == NP, "parameter offsets drifted");'
            ),
            'static_assert(sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0]) == NX, "state names drifted");',
            'static_assert(sizeof(CONTROL_NAMES) / sizeof(CONTROL_NAMES[0]) == NU, "control names drifted");',
            'static_assert(sizeof(PARAMETER_NAMES) / sizeof(PARAMETER_NAMES[0]) == NP, "parameter names drifted");',
            'static_assert(sizeof(kModelContractSemanticSha256) == 65U, "semantic SHA-256 drifted");',
            'static_assert(sizeof(kArtifactSha256) == 65U, "artifact SHA-256 drifted");',
            'static_assert(sizeof(kModelContractJsonRawSha256) == 65U, "model contract raw SHA-256 drifted");',
            'static_assert(sizeof(kSolverLibraryRawSha256) == 65U, "solver library SHA-256 drifted");',
            'static_assert(kSolverLibrarySizeBytes > 0U, "solver library must not be empty");',
            "",
        )
    )
    for namespace in reversed(HEADER_NAMESPACE):
        lines.append(f"}}  // namespace {namespace}")
    lines.extend(("", f"#endif  // {HEADER_GUARD}", ""))
    return "\n".join(lines)


def render_model_contract_header(value: ArtifactContract) -> str:
    """Render a standalone header from the exact serialized JSON authority."""

    return _render_document(
        _checked_document(value),
        model_contract_json_raw_sha256(value),
    )


def validate_model_contract_header(
    value: ArtifactContract,
    rendered: Any,
) -> str:
    """Require a candidate header to equal the deterministic projection."""

    if type(rendered) is not str:
        raise ModelContractHeaderError("model contract header must be text")
    if rendered != render_model_contract_header(value):
        raise ModelContractHeaderError("model contract header is stale or modified")
    return rendered


__all__ = [
    "HEADER_GUARD",
    "HEADER_NAMESPACE",
    "ModelContractHeaderError",
    "render_model_contract_header",
    "validate_model_contract_header",
]
