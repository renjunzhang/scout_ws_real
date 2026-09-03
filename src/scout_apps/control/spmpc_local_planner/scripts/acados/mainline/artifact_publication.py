"""Safe contract-output writing and no-overwrite directory publication."""

from __future__ import annotations

import ctypes
import errno
import os
import stat
import tempfile
from pathlib import Path

from .artifact_contract import (
    ArtifactContract,
    artifact_contract_from_dict,
    render_model_contract_json,
    require_artifact_contract,
    validate_model_contract_json,
)
from .artifact_files import (
    generated_file_records_from_dict,
    require_artifact_directory,
    validate_generated_tree,
)
from .codegen_options import GENERATED_HEADER_FILENAME, MODEL_CONTRACT_FILENAME
from .identity import (
    IdentityError,
    read_stable_regular_file,
    read_strict_json,
    strict_json_equal,
)
from .model_contract_header import (
    render_model_contract_header,
    validate_model_contract_header,
)

RENAME_NOREPLACE = 1


class ArtifactPublicationError(RuntimeError):
    """Contract output or atomic directory publication failed closed."""


class ArtifactPublicationCommittedError(ArtifactPublicationError):
    """The no-replace rename committed, but post-commit verification failed."""

    def __init__(self, published_target: Path, detail: str) -> None:
        self.published_target = published_target
        super().__init__(
            f"artifact was atomically published at {published_target}, but {detail}"
        )


def require_unpublished_artifact_target(value: Path | str) -> Path:
    """Require an absent absolute target below a real existing parent."""

    if not isinstance(value, (str, Path)):
        raise ArtifactPublicationError("artifact target must be str or Path")
    target = Path(value)
    if not target.is_absolute() or ".." in target.parts or not target.name:
        raise ArtifactPublicationError(
            "artifact target must be a named canonical absolute path"
        )
    normalized = Path(os.path.normpath(str(target)))
    if normalized != target:
        raise ArtifactPublicationError("artifact target path is not canonical")
    try:
        parent = require_artifact_directory(target.parent)
    except ValueError as exc:
        raise ArtifactPublicationError("artifact target parent is unsafe") from exc
    if parent != target.parent:
        raise ArtifactPublicationError("artifact target parent identity drifted")
    try:
        target.lstat()
    except FileNotFoundError:
        return target
    except OSError as exc:
        raise ArtifactPublicationError(
            f"cannot inspect artifact target {target}: {exc}"
        ) from exc
    raise ArtifactPublicationError(f"artifact target already exists: {target}")


def create_sibling_staging_directory(target: Path | str) -> Path:
    """Create one unique staging directory beside an absent target."""

    checked_target = require_unpublished_artifact_target(target)
    try:
        staging = Path(
            tempfile.mkdtemp(
                prefix=f".{checked_target.name}.staging-",
                dir=str(checked_target.parent),
            )
        )
        staging.chmod(0o755)
        checked_staging = require_artifact_directory(staging)
    except (OSError, ValueError) as exc:
        raise ArtifactPublicationError(
            "cannot create the sibling artifact staging directory"
        ) from exc
    if checked_staging.parent != checked_target.parent:
        raise ArtifactPublicationError("artifact staging escaped the target parent")
    return checked_staging


def _embedded_generated_files(contract: ArtifactContract):
    try:
        checked = require_artifact_contract(contract)
        serialized = checked.to_dict()["artifact"]["generated_tree"]["files"]
        return generated_file_records_from_dict(serialized)
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactPublicationError(
            "artifact contract generated-file inventory is malformed"
        ) from exc


def _write_new_regular(path: Path, payload: bytes) -> None:
    if type(payload) is not bytes:
        raise ArtifactPublicationError("artifact output payload must be bytes")
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(path, flags, 0o644)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ArtifactPublicationError(
                f"artifact output is not a regular file: {path}"
            )
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except ArtifactPublicationError:
        raise
    except OSError as exc:
        raise ArtifactPublicationError(
            f"cannot create artifact output {path}: {exc}"
        ) from exc
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass


def _fsync_directory(directory: Path) -> None:
    flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(directory, flags)
        os.fsync(descriptor)
    except OSError as exc:
        raise ArtifactPublicationError(
            f"cannot durably sync artifact directory {directory}: {exc}"
        ) from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)


def load_artifact_contract_directory(value: Path | str) -> ArtifactContract:
    """Reload and validate a complete staged or published artifact directory."""

    try:
        root = require_artifact_directory(value)
        document, json_bytes = read_strict_json(
            root / MODEL_CONTRACT_FILENAME,
            label="D4 model contract",
        )
        contract = artifact_contract_from_dict(document)
        validate_model_contract_json(contract, json_bytes)
        header_bytes = read_stable_regular_file(
            root / GENERATED_HEADER_FILENAME,
            label="D4 generated model-contract header",
        )
        try:
            header = header_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ArtifactPublicationError(
                "generated model-contract header is not UTF-8"
            ) from exc
        validate_model_contract_header(contract, header)
        validate_generated_tree(root, _embedded_generated_files(contract))
        return contract
    except ArtifactPublicationError:
        raise
    except (IdentityError, OSError, TypeError, ValueError) as exc:
        raise ArtifactPublicationError(
            "artifact directory contract, header, or generated tree is invalid"
        ) from exc


def write_artifact_contract_outputs(
    staging_directory: Path | str,
    contract: ArtifactContract,
) -> ArtifactContract:
    """Create the two deterministic contract outputs and verify them from disk."""

    checked_contract = require_artifact_contract(contract)
    try:
        root = require_artifact_directory(staging_directory)
        validate_generated_tree(root, _embedded_generated_files(checked_contract))
    except (TypeError, ValueError) as exc:
        raise ArtifactPublicationError(
            "artifact staging tree differs from the contract"
        ) from exc
    json_path = root / MODEL_CONTRACT_FILENAME
    header_path = root / GENERATED_HEADER_FILENAME
    for path in (json_path, header_path):
        try:
            path.lstat()
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise ArtifactPublicationError(
                f"cannot inspect artifact contract output {path}: {exc}"
            ) from exc
        raise ArtifactPublicationError(
            f"artifact contract output already exists: {path}"
        )

    _write_new_regular(json_path, render_model_contract_json(checked_contract))
    _write_new_regular(
        header_path,
        render_model_contract_header(checked_contract).encode("utf-8"),
    )
    _fsync_directory(root)
    reloaded = load_artifact_contract_directory(root)
    if not strict_json_equal(reloaded.to_dict(), checked_contract.to_dict()):
        raise ArtifactPublicationError("reloaded artifact contract identity drifted")
    return reloaded


def _renameat2_noreplace(parent_descriptor: int, source: str, target: str) -> None:
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        renameat2 = libc.renameat2
    except (AttributeError, OSError) as exc:
        raise ArtifactPublicationError(
            "atomic no-replace directory publication is unavailable"
        ) from exc
    renameat2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    result = renameat2(
        parent_descriptor,
        os.fsencode(source),
        parent_descriptor,
        os.fsencode(target),
        RENAME_NOREPLACE,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        if error_number in {errno.EEXIST, errno.ENOTEMPTY}:
            raise ArtifactPublicationError(
                "artifact target appeared before atomic publication"
            )
        raise ArtifactPublicationError(
            f"atomic artifact publication failed: {os.strerror(error_number)}"
        )


def publish_staging_directory(
    staging_directory: Path | str,
    target_directory: Path | str,
) -> Path:
    """Atomically rename a sibling staging directory without replacing a target."""

    try:
        staging = require_artifact_directory(staging_directory)
    except ValueError as exc:
        raise ArtifactPublicationError("artifact staging directory is unsafe") from exc
    target = require_unpublished_artifact_target(target_directory)
    if staging.parent != target.parent or staging == target:
        raise ArtifactPublicationError(
            "artifact staging and target must be distinct siblings"
        )
    flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor: int | None = None
    try:
        descriptor = os.open(staging.parent, flags)
        _renameat2_noreplace(descriptor, staging.name, target.name)
    except ArtifactPublicationError:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise
    except OSError as exc:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise ArtifactPublicationError(
            f"artifact publication failed before commit: {exc}"
        ) from exc
    sync_error: OSError | None = None
    try:
        os.fsync(descriptor)
    except OSError as exc:
        sync_error = exc
    close_error: OSError | None = None
    try:
        os.close(descriptor)
    except OSError as exc:
        close_error = exc
    if sync_error is not None:
        raise ArtifactPublicationCommittedError(
            target,
            f"its parent directory could not be durably synced: {sync_error}",
        ) from sync_error
    if close_error is not None:
        raise ArtifactPublicationCommittedError(
            target,
            f"its parent directory descriptor could not be closed: {close_error}",
        ) from close_error
    try:
        published = require_artifact_directory(target)
    except ValueError as exc:
        raise ArtifactPublicationCommittedError(
            target,
            "the published directory cannot be reopened",
        ) from exc
    if staging.exists() or staging.is_symlink():
        raise ArtifactPublicationCommittedError(
            target,
            "the old staging pathname still exists",
        )
    return published


__all__ = [
    "RENAME_NOREPLACE",
    "ArtifactPublicationCommittedError",
    "ArtifactPublicationError",
    "create_sibling_staging_directory",
    "load_artifact_contract_directory",
    "publish_staging_directory",
    "require_unpublished_artifact_target",
    "write_artifact_contract_outputs",
]
