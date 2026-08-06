#!/usr/bin/env python3
"""Fail-closed, append-only audit of the installed QEMU host state.

The only state-changing subcommand creates one new PASS receipt below the
fixed liquid audit directory.  This module never invokes QEMU/qemu-img, APT,
sudo, a shell, a VM, an upstream tool, or the DualSPHysics source tree.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from jsonschema import Draft202012Validator
from jsonschema.exceptions import SchemaError


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_safety as safety  # noqa: E402


SCHEMA_VERSION = "smpcc-r8-liquid-qemu-install-preflight-v2"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_QEMU_INSTALL_PREFLIGHT"
POLICY_PATH = PACKAGE_ROOT / "config/sandbox/p0b_qemu_install_admission_v2.json"
ADMISSION_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_install_admission_v2.json"
PREFLIGHT_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_install_preflight_v2.json"
LEGACY_ARTIFACTS = (
    (
        "config/sandbox/p0b_vm_policy_v1.json",
        "c0670e5f3a2b7bea943fc2e29ee988921ac3fbfa1c93bc1f845bff3ea619a516",
    ),
    (
        "scripts/r8_liquid_sandbox_gate.py",
        "10709cffa6e9e0c0aff35dde370c4550c810bc21da783f33958512eb3efdcb09",
    ),
    (
        "schema/sandbox_preflight_v1.json",
        "1d272e601133cf21092fa5eb60a217c5f66a89a045d7dcda94cf1503345ef9fb",
    ),
)
LEGACY_RECEIPTS = (
    (
        "/data/a/scout_sim_replacement/r8_liquid/audits/sandbox/sandbox_preflight_vm-host_20260805T035755Z.json",
        "87ea81b036fc8a53454e5f859e95de4141831f60a60f43e5682f1e406097dcd9",
        "19f57eb59a16e75b9aa160372d29be7012dbc9a0086060f040f8dd0f2154ac81",
    ),
    (
        "/data/a/scout_sim_replacement/r8_liquid/audits/sandbox/sandbox_preflight_cpu-build_20260805T035756Z.json",
        "9cbe09208ea7c3318fb83e5e14feb39379835413a711ba23e7ef59c4f0416e05",
        "b6861f6dd439ac6bede2a833f9d9ca9ab5c5b29f150ec08f7ddcaec64598d42b",
    ),
)
EXPECTED_PACKAGES = (
    ("ipxe-qemu", "all", "1.0.0+git-20190109.133f4c4-0ubuntu3.2"),
    ("ipxe-qemu-256k-compat-efi-roms", "all", "1.0.0+git-20150424.a25a16d-0ubuntu4"),
    ("libaio1", "amd64", "0.3.112-5"),
    ("libcacard0", "amd64", "1:2.6.1-1"),
    ("libfdt1", "amd64", "1.5.1-1"),
    ("libiscsi7", "amd64", "1.18.0-2"),
    ("libpmem1", "amd64", "1.8-1ubuntu1"),
    ("librados2", "amd64", "15.2.17-0ubuntu0.20.04.6+esm1"),
    ("librbd1", "amd64", "15.2.17-0ubuntu0.20.04.6+esm1"),
    ("libslirp0", "amd64", "4.1.0-2ubuntu2.2"),
    ("libspice-server1", "amd64", "0.14.2-4ubuntu3.1"),
    ("libusbredirparser1", "amd64", "0.8.0-1ubuntu0.1"),
    ("libvirglrenderer1", "amd64", "0.8.2-1ubuntu1.1"),
    ("qemu-block-extra", "amd64", "1:4.2-3ubuntu6.30+esm3"),
    ("qemu-system-common", "amd64", "1:4.2-3ubuntu6.30+esm3"),
    ("qemu-system-data", "all", "1:4.2-3ubuntu6.30+esm3"),
    ("qemu-system-x86", "amd64", "1:4.2-3ubuntu6.30+esm3"),
    ("qemu-utils", "amd64", "1:4.2-3ubuntu6.30+esm3"),
    ("seabios", "all", "1.13.0-1ubuntu1.1"),
)
EXPECTED_QEMU_NAMESPACE = (
    "qemu-block-extra",
    "qemu-system-common",
    "qemu-system-data",
    "qemu-system-x86",
    "qemu-utils",
)
EXPECTED_ABSENT_PACKAGES = (
    "bridge-utils",
    "libvirt-daemon",
    "libvirt-daemon-system",
    "ovmf",
    "qemu-system-gui",
    "qemu-user",
    "qemu-user-binfmt",
    "qemu-user-static",
)
EXPECTED_CRITICAL_FILES = (
    (
        "/usr/bin/qemu-system-x86_64",
        "qemu-system-x86",
        "0755",
        16287496,
        "6e8225fd4ffcdc6c4696dd5bf181cade84876689965946e46a50834ad4c5a718",
    ),
    (
        "/usr/bin/qemu-img",
        "qemu-utils",
        "0755",
        1997112,
        "21348cee8cf446f7253011ba4d349db438f554590b0456894f56342ea24e5806",
    ),
    (
        "/usr/share/qemu/init/qemu-kvm-init",
        "qemu-system-common",
        "0755",
        2496,
        "c769a6a149897575767c5a4377cc7cb15dcbb586a9a1eec1d6248dba098b08f2",
    ),
    (
        "/lib/systemd/system/qemu-kvm.service",
        "qemu-system-common",
        "0644",
        367,
        "da0544deed8c3e9f9f8221c99f5e1ca67135f15f1f3f070767cb8966f705304a",
    ),
    (
        "/etc/default/qemu-kvm",
        "qemu-system-common",
        "0644",
        319,
        "4096435ea01fef66dd5720445a38d02a6d5b00083e3604dcd41922556f14797d",
    ),
)
EXPECTED_PACKAGE_NAMES = tuple(item[0] for item in EXPECTED_PACKAGES)
EXPECTED_QUERY_PACKAGE_NAMES = frozenset(EXPECTED_PACKAGE_NAMES + EXPECTED_ABSENT_PACKAGES)
EXPECTED_CRITICAL_PATHS = frozenset(item[0] for item in EXPECTED_CRITICAL_FILES)
DPKG = Path("/usr/bin/dpkg")
DPKG_QUERY = Path("/usr/bin/dpkg-query")
DPKG_DIVERT = Path("/usr/bin/dpkg-divert")
SYSTEMCTL = Path("/usr/bin/systemctl")
PRO = Path("/usr/bin/pro")
OS_RELEASE = Path("/usr/lib/os-release")
APT_HISTORY = Path("/var/log/apt/history.log")
KSM_RUN = Path("/sys/kernel/mm/ksm/run")
QEMU_KVM_AUTOSTART_LINK = Path(
    "/etc/systemd/system/multi-user.target.wants/qemu-kvm.service"
)
ALLOWED_QUERY_EXECUTABLES = frozenset((DPKG, DPKG_QUERY, DPKG_DIVERT, SYSTEMCTL, PRO))
MAX_JSON_BYTES = 1024 * 1024
MAX_QUERY_OUTPUT_BYTES = 1024 * 1024
PROCESS_VISIBILITY_UID_MIN = 1000
UNREADABLE_UNPRIVILEGED_PROCESS_ALLOWLIST = (
    {
        "pid": 1927,
        "uid": 1000,
        "starttime_ticks": 2606,
        "ppid": 1926,
        "parent_starttime_ticks": 2606,
        "cgroup_sha256": "583bb7a90dfbd0220f3ed0852e7962b99eb56445f23c07569578495222229acd",
        "comm": "(sd-pam)",
        "argv0": "(sd-pam)",
    },
    {
        "pid": 1940,
        "uid": 1000,
        "starttime_ticks": 2615,
        "ppid": 1,
        "parent_starttime_ticks": 12,
        "cgroup_sha256": "f66813ef47a3728a1f7fcda618ded948b39df66ae64d44ac54c195231f00c2de",
        "comm": "gnome-keyring-d",
        "argv0": "/usr/bin/gnome-keyring-daemon",
    },
    {
        "pid": 2015,
        "uid": 1000,
        "starttime_ticks": 2797,
        "ppid": 1963,
        "parent_starttime_ticks": 2789,
        "cgroup_sha256": "f66813ef47a3728a1f7fcda618ded948b39df66ae64d44ac54c195231f00c2de",
        "comm": "ssh-agent",
        "argv0": "/usr/bin/ssh-agent",
    },
)
RECEIPT_RE = re.compile(r"^qemu_install_preflight_v2_[0-9]{8}T[0-9]{6}Z\.json$")


class QemuInstallGateError(RuntimeError):
    """Any ambiguity in the observed installation is a hard NO-GO."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise QemuInstallGateError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def _reject_duplicate_keys(pairs: Iterable[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise QemuInstallGateError(f"duplicate JSON key is forbidden: {key}")
        result[key] = value
    return result


def _reject_nonfinite(value: str) -> None:
    raise QemuInstallGateError(f"non-finite JSON number is forbidden: {value}")


def _stable_open(path: Path) -> Tuple[int, os.stat_result]:
    path = Path(path)
    require(path.is_absolute(), f"path is not absolute: {path}")
    lexical = path
    if path == Path("/lib/systemd/system/qemu-kvm.service"):
        lib_stat = os.lstat("/lib")
        require(stat.S_ISLNK(lib_stat.st_mode), "/lib is no longer the expected symlink")
        require(os.readlink("/lib") == "usr/lib", "/lib symlink target differs")
        require(lib_stat.st_uid == 0 and lib_stat.st_gid == 0, "/lib symlink ownership differs")
        lexical = Path("/usr/lib/systemd/system/qemu-kvm.service")
    require(not lexical.is_symlink(), f"final path is a symlink: {path}")
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    parent_fd = safety.open_directory_nofollow(lexical.parent)
    try:
        fd = os.open(lexical.name, flags, dir_fd=parent_fd)
    finally:
        os.close(parent_fd)
    before = os.fstat(fd)
    require(stat.S_ISREG(before.st_mode), f"file is not regular: {path}")
    return fd, before


def _capabilities(fd: int, path: Path) -> List[str]:
    try:
        value = os.getxattr(fd, "security.capability")
    except OSError as exc:
        missing = {errno.ENODATA}
        if hasattr(errno, "ENOATTR"):
            missing.add(errno.ENOATTR)
        if exc.errno not in missing:
            raise QemuInstallGateError(
                f"cannot inspect file capabilities for {path}: {exc}"
            ) from exc
        return []
    return [value.hex()] if value else []


def file_hash_and_observation(path: Path) -> Tuple[str, Dict[str, Any]]:
    fd, before = _stable_open(path)
    digest = hashlib.sha256()
    try:
        capabilities = _capabilities(fd, path)
        with os.fdopen(fd, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
            after = os.fstat(stream.fileno())
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            f"file changed while hashing: {path}",
        )
    except Exception:
        try:
            os.close(fd)
        except OSError:
            pass
        raise
    digest_text = digest.hexdigest()
    return digest_text, {
        "path": str(path),
        "file_type": "regular",
        "symlink": False,
        "uid": after.st_uid,
        "gid": after.st_gid,
        "mode": f"{stat.S_IMODE(after.st_mode):04o}",
        "size": after.st_size,
        "sha256": digest_text,
        "security_capabilities": capabilities,
        "device_id": after.st_dev,
        "inode": after.st_ino,
        "mtime_ns": after.st_mtime_ns,
    }


def read_json_and_hash(path: Path) -> Tuple[Dict[str, Any], str]:
    fd, before = _stable_open(path)
    try:
        require(before.st_size <= MAX_JSON_BYTES, f"JSON file exceeds size limit: {path}")
        with os.fdopen(fd, "rb") as stream:
            payload = stream.read(MAX_JSON_BYTES + 1)
            after = os.fstat(stream.fileno())
        require(len(payload) <= MAX_JSON_BYTES, f"JSON file exceeds size limit: {path}")
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            f"JSON file changed while reading: {path}",
        )
    except Exception:
        try:
            os.close(fd)
        except OSError:
            pass
        raise
    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QemuInstallGateError(f"invalid JSON document {path}: {exc}") from exc
    require(isinstance(value, dict), f"JSON document is not an object: {path}")
    return value, hashlib.sha256(payload).hexdigest()


def validate_with_schema(instance: Mapping[str, Any], schema: Mapping[str, Any], name: str) -> None:
    try:
        Draft202012Validator.check_schema(schema)
    except SchemaError as exc:
        raise QemuInstallGateError(f"{name} schema is invalid: {exc.message}") from exc
    errors = sorted(
        Draft202012Validator(schema).iter_errors(instance),
        key=lambda item: tuple(str(part) for part in item.path),
    )
    if errors:
        first = errors[0]
        location = ".".join(str(item) for item in first.absolute_path) or "<root>"
        raise QemuInstallGateError(f"{name} schema validation failed at {location}: {first.message}")


def _argv_is_readonly(argv: Sequence[str]) -> bool:
    values = tuple(argv)
    fmt_package = "${Package}\\t${Architecture}\\t${Version}\\t${Status}\\n"
    fmt_namespace = "${Package}\\t${Status}\\n"
    static = {
        (str(DPKG), "--print-architecture"),
        (str(DPKG_QUERY), "--show", f"--showformat={fmt_namespace}", "qemu*"),
        (str(DPKG), "--verify", *EXPECTED_PACKAGE_NAMES),
        (
            str(SYSTEMCTL),
            "show",
            "qemu-kvm.service",
            "-p",
            "LoadState",
            "-p",
            "UnitFileState",
            "-p",
            "ActiveState",
            "-p",
            "SubState",
            "-p",
            "FragmentPath",
            "--no-pager",
        ),
        (str(PRO), "status", "--format", "json"),
    }
    if values in static:
        return True
    if (
        len(values) == 4
        and values[:3] == (str(DPKG_QUERY), "--show", f"--showformat={fmt_package}")
        and values[3] in EXPECTED_QUERY_PACKAGE_NAMES
    ):
        return True
    if (
        len(values) == 3
        and values[:2] == (str(DPKG_QUERY), "--search")
        and values[2] in EXPECTED_CRITICAL_PATHS
    ):
        return True
    if (
        len(values) == 3
        and values[:2] == (str(DPKG_DIVERT), "--list")
        and values[2] in EXPECTED_CRITICAL_PATHS
    ):
        return True
    return False


def _execute_readonly_query(
    argv: Sequence[str],
    *,
    accepted_returncodes: Sequence[int] = (0,),
) -> subprocess.CompletedProcess[str]:
    require(_argv_is_readonly(argv), f"query argv is not on the fixed read-only allowlist: {list(argv)!r}")
    require(argv and Path(argv[0]) in ALLOWED_QUERY_EXECUTABLES, "query executable is not admitted")
    try:
        completed = subprocess.run(
            list(argv),
            check=False,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=15,
            shell=False,
            env={"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"},
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise QemuInstallGateError(f"fixed read-only query failed: {list(argv)!r}: {exc}") from exc
    require(
        len(completed.stdout.encode("utf-8")) <= MAX_QUERY_OUTPUT_BYTES
        and len(completed.stderr.encode("utf-8")) <= MAX_QUERY_OUTPUT_BYTES,
        "query output exceeds limit",
    )
    require(
        completed.returncode in accepted_returncodes,
        f"fixed read-only query returned {completed.returncode}: {list(argv)!r}",
    )
    return completed


def run_collector(name: str, argument: Optional[str] = None) -> subprocess.CompletedProcess[str]:
    fmt_package = "${Package}\\t${Architecture}\\t${Version}\\t${Status}\\n"
    fmt_namespace = "${Package}\\t${Status}\\n"
    if name == "dpkg-architecture" and argument is None:
        return _execute_readonly_query((str(DPKG), "--print-architecture"))
    if name == "qemu-namespace" and argument is None:
        return _execute_readonly_query(
            (str(DPKG_QUERY), "--show", f"--showformat={fmt_namespace}", "qemu*"),
            accepted_returncodes=(0, 1),
        )
    if name == "dpkg-verify" and argument is None:
        return _execute_readonly_query(
            (str(DPKG), "--verify", *EXPECTED_PACKAGE_NAMES),
            accepted_returncodes=(0, 1, 2),
        )
    if name == "qemu-kvm-service" and argument is None:
        return _execute_readonly_query(
            (
                str(SYSTEMCTL),
                "show",
                "qemu-kvm.service",
                "-p",
                "LoadState",
                "-p",
                "UnitFileState",
                "-p",
                "ActiveState",
                "-p",
                "SubState",
                "-p",
                "FragmentPath",
                "--no-pager",
            )
        )
    if name == "ubuntu-pro-status" and argument is None:
        return _execute_readonly_query((str(PRO), "status", "--format", "json"))
    if name == "package" and argument in EXPECTED_QUERY_PACKAGE_NAMES:
        return _execute_readonly_query(
            (str(DPKG_QUERY), "--show", f"--showformat={fmt_package}", str(argument)),
            accepted_returncodes=(0, 1),
        )
    if name == "owner" and argument in EXPECTED_CRITICAL_PATHS:
        return _execute_readonly_query((str(DPKG_QUERY), "--search", str(argument)))
    if name == "diversion" and argument in EXPECTED_CRITICAL_PATHS:
        return _execute_readonly_query((str(DPKG_DIVERT), "--list", str(argument)))
    raise QemuInstallGateError(f"unknown or malformed fixed collector request: {name!r}, {argument!r}")


def package_observation(name: str) -> Dict[str, Any]:
    require(re.fullmatch(r"[a-z0-9][a-z0-9+.-]*", name) is not None, f"invalid package name: {name}")
    completed = run_collector("package", name)
    line = completed.stdout.rstrip("\n")
    if completed.returncode == 1 or not line:
        return {"name": name, "installed": False, "architecture": None, "version": None, "status": None}
    fields = line.split("\t")
    require(len(fields) == 4, f"dpkg-query returned a malformed package record: {name}")
    package_name, architecture, version, package_status = fields
    return {
        "name": package_name or name,
        "installed": package_status == "install ok installed",
        "architecture": architecture or None,
        "version": version or None,
        "status": package_status or None,
    }


def qemu_namespace() -> List[str]:
    completed = run_collector("qemu-namespace")
    installed = []
    for line in completed.stdout.splitlines():
        fields = line.split("\t")
        require(len(fields) == 2, "dpkg-query returned malformed QEMU namespace data")
        if fields[1] == "install ok installed":
            installed.append(fields[0])
    return sorted(set(installed))


def package_owner(path: Path) -> str:
    completed = run_collector("owner", str(path))
    owners = []
    for line in completed.stdout.splitlines():
        require(": " in line, f"dpkg-query returned malformed ownership data: {path}")
        owner, owned_path = line.rsplit(": ", 1)
        if owned_path == str(path):
            owners.append(owner.split(":", 1)[0])
    owners = sorted(set(owners))
    require(len(owners) == 1, f"critical file ownership is missing or ambiguous: {path}")
    return owners[0]


def is_diverted(path: Path) -> bool:
    completed = run_collector("diversion", str(path))
    return bool(completed.stdout.strip())


def service_observation() -> Dict[str, str]:
    properties = ("LoadState", "UnitFileState", "ActiveState", "SubState", "FragmentPath")
    completed = run_collector("qemu-kvm-service")
    raw: Dict[str, str] = {}
    for line in completed.stdout.splitlines():
        require("=" in line, "systemctl returned malformed property data")
        key, value = line.split("=", 1)
        require(key in properties and key not in raw, f"unexpected systemctl property: {key}")
        raw[key] = value
    require(set(raw) == set(properties), "systemctl property set is incomplete")
    return {
        "load_state": raw["LoadState"],
        "unit_file_state": raw["UnitFileState"],
        "active_state": raw["ActiveState"],
        "sub_state": raw["SubState"],
        "fragment_path": raw["FragmentPath"],
    }


def read_ksm() -> str:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(str(KSM_RUN), flags)
    with os.fdopen(fd, "rb") as stream:
        payload = stream.read(16)
        require(stream.read(1) == b"", "KSM state output is unexpectedly large")
    require(payload in (b"0\n", b"1\n", b"2\n"), "KSM state has an invalid encoding")
    return payload.decode("ascii").strip()


def _critical_inode_set(files: Sequence[Mapping[str, Any]]) -> set[Tuple[int, int]]:
    return {
        (int(item["device_id"]), int(item["inode"]))
        for item in files
        if item["path"] in ("/usr/bin/qemu-system-x86_64", "/usr/bin/qemu-img")
    }


def _proc_mount_visibility(proc_root: Path) -> List[str]:
    if proc_root != Path("/proc"):
        return []
    try:
        lines = Path("/proc/self/mountinfo").read_text(
            encoding="utf-8", errors="strict"
        ).splitlines()
    except OSError as exc:
        return [f"cannot read process mount metadata: {exc}"]
    matches = []
    for line in lines:
        fields = line.split()
        if len(fields) < 10 or "-" not in fields:
            return ["process mount metadata is malformed"]
        separator = fields.index("-")
        if fields[4] == "/proc":
            if separator + 3 >= len(fields):
                return ["process mount metadata is incomplete"]
            matches.append(
                {
                    "mount_options": fields[5].split(","),
                    "fstype": fields[separator + 1],
                    "super_options": fields[separator + 3].split(","),
                }
            )
    if len(matches) != 1 or matches[0]["fstype"] != "proc":
        return ["system /proc mount is missing or ambiguous"]
    options = set(matches[0]["mount_options"] + matches[0]["super_options"])
    forbidden = sorted(
        item
        for item in options
        if item == "subset=pid" or (item.startswith("hidepid=") and item != "hidepid=0")
    )
    return [f"system /proc visibility is restricted: {forbidden}"] if forbidden else []


def _pid_details(entry: Path) -> Optional[Tuple[int, int, int]]:
    try:
        raw = (entry / "stat").read_text(encoding="ascii", errors="strict")
    except (FileNotFoundError, ProcessLookupError):
        return None
    except (PermissionError, OSError) as exc:
        raise QemuInstallGateError(f"cannot establish stable PID identity for {entry.name}: {exc}") from exc
    try:
        pid_text, remainder = raw.split(" (", 1)
        fields = remainder.rsplit(") ", 1)[1].split()
        pid = int(pid_text)
        ppid = int(fields[1])
        starttime = int(fields[19])
    except (IndexError, ValueError) as exc:
        raise QemuInstallGateError(f"malformed process stat identity for PID {entry.name}") from exc
    require(pid == int(entry.name), f"process stat PID differs for {entry.name}")
    return pid, ppid, starttime


def _pid_identity(entry: Path) -> Optional[Tuple[int, int]]:
    details = _pid_details(entry)
    return None if details is None else (details[0], details[2])


def _process_cgroup_hash(entry: Path) -> str:
    try:
        payload = (entry / "cgroup").read_bytes()
    except (FileNotFoundError, ProcessLookupError):
        raise QemuInstallGateError(f"process cgroup disappeared for PID {entry.name}")
    except (PermissionError, OSError) as exc:
        raise QemuInstallGateError(f"process cgroup is unreadable for PID {entry.name}: {exc}") from exc
    require(len(payload) <= 65536, f"process cgroup is unexpectedly large for PID {entry.name}")
    return hashlib.sha256(payload).hexdigest()


def qemu_process_snapshot(
    critical_files: Sequence[Mapping[str, Any]],
    *,
    proc_root: Path = Path("/proc"),
) -> Dict[str, Any]:
    admitted_inodes = _critical_inode_set(critical_files)
    admitted_paths = {"/usr/bin/qemu-system-x86_64", "/usr/bin/qemu-img"}
    found = []
    visibility_errors = _proc_mount_visibility(proc_root)
    visibility_exceptions = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        identity_before = _pid_identity(entry)
        if identity_before is None:
            continue
        try:
            process_uid = os.stat(entry).st_uid
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (PermissionError, OSError) as exc:
            identity_after = _pid_identity(entry)
            if identity_after is not None and identity_after == identity_before:
                visibility_errors.append(
                    f"stable PID {entry.name} ownership is unreadable: {exc}"
                )
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            identity_after = _pid_identity(entry)
            if identity_after is not None and identity_after == identity_before:
                visibility_errors.append(f"stable PID {entry.name} comm is unreadable")
            continue
        try:
            raw_cmdline = (entry / "cmdline").read_bytes()
            argv = [
                token.decode("utf-8", errors="replace")
                for token in raw_cmdline.split(b"\0")
                if token
            ]
            cmdline_permission_denied = False
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            argv = []
            cmdline_permission_denied = True
        exe_path: Optional[str]
        exe_identity: Optional[Tuple[int, int]]
        try:
            exe_path = os.readlink(entry / "exe")
            if exe_path.endswith(" (deleted)"):
                exe_path = exe_path[: -len(" (deleted)")]
            exe_stat = os.stat(entry / "exe")
            exe_identity = (exe_stat.st_dev, exe_stat.st_ino)
            exe_permission_denied = False
        except PermissionError:
            exe_path = None
            exe_identity = None
            exe_permission_denied = True
        except (FileNotFoundError, ProcessLookupError, OSError):
            exe_path = None
            exe_identity = None
            exe_permission_denied = False
        identity_after = _pid_identity(entry)
        if identity_after is None or identity_after != identity_before:
            continue
        if cmdline_permission_denied and exe_permission_denied:
            visibility_errors.append(
                f"stable PID {entry.name} has no readable cmdline or executable identity"
            )
        reasons = []
        lowered = comm.lower()
        if lowered.startswith("qemu") or lowered in ("qemu-kvm", "kvm"):
            reasons.append("comm")
        argv0_text = argv[0] if argv else ""
        if argv0_text:
            argv0_name = Path(argv0_text).name.lower()
            if argv0_name.startswith("qemu") or argv0_name in ("qemu-kvm", "kvm"):
                reasons.append("argv0")
        if exe_path in admitted_paths or (
            exe_path is not None and exe_path.startswith("/usr/bin/qemu-")
        ):
            reasons.append("exe_path")
        if exe_identity in admitted_inodes:
            reasons.append("exe_inode")
        if exe_permission_denied and process_uid >= PROCESS_VISIBILITY_UID_MIN and not reasons:
            details = _pid_details(entry)
            if details is None:
                continue
            _, parent_pid, starttime_ticks = details
            parent_identity = _pid_identity(proc_root / str(parent_pid))
            if parent_identity is None:
                visibility_errors.append(
                    f"stable unprivileged PID {entry.name} parent identity is unavailable"
                )
                continue
            candidate = {
                "pid": int(entry.name),
                "uid": process_uid,
                "starttime_ticks": starttime_ticks,
                "ppid": parent_pid,
                "parent_starttime_ticks": parent_identity[1],
                "cgroup_sha256": _process_cgroup_hash(entry),
                "comm": comm,
                "argv0": argv0_text,
            }
            if candidate in UNREADABLE_UNPRIVILEGED_PROCESS_ALLOWLIST:
                visibility_exceptions.append(candidate)
            else:
                visibility_errors.append(
                    f"stable unprivileged PID {entry.name} executable identity is unreadable"
                )
        if reasons:
            found.append(
                {
                    "pid": int(entry.name),
                    "uid": process_uid,
                    "comm": comm,
                    "exe": exe_path,
                    "reasons": sorted(set(reasons)),
                }
            )
    return {
        "offenders": sorted(found, key=lambda item: item["pid"]),
        "visibility_errors": sorted(set(visibility_errors)),
        "visibility_exceptions": sorted(
            visibility_exceptions, key=lambda item: item["pid"]
        ),
    }


def qemu_process_observations(
    critical_files: Sequence[Mapping[str, Any]],
    *,
    proc_root: Path = Path("/proc"),
) -> List[Dict[str, Any]]:
    snapshot = qemu_process_snapshot(critical_files, proc_root=proc_root)
    require(not snapshot["visibility_errors"], f"process visibility is incomplete: {snapshot['visibility_errors']}")
    return snapshot["offenders"]


def quiescence_errors(
    *,
    service: Mapping[str, Any],
    autostart_exists: bool,
    autostart_is_symlink: bool,
    ksm_start: Optional[str],
    ksm_end: Optional[str],
    processes: Sequence[Mapping[str, Any]],
    process_visibility_errors: Sequence[str] = (),
    required: Mapping[str, Any],
) -> List[str]:
    errors = []
    if service != required["qemu_kvm_service"]:
        errors.append("qemu-kvm service state differs")
    if autostart_exists or autostart_is_symlink:
        errors.append("qemu-kvm autostart link exists")
    if (ksm_start, ksm_end) != (
        required["ksm_required_start_value"],
        required["ksm_required_end_value"],
    ):
        errors.append("KSM was enabled or changed during collection")
    if processes:
        errors.append("QEMU process set is not empty")
    if process_visibility_errors:
        errors.append("system process visibility is incomplete")
    return errors


def host_platform() -> Dict[str, str]:
    fd, before = _stable_open(OS_RELEASE)
    with os.fdopen(fd, "r", encoding="utf-8") as stream:
        payload = stream.read(32769)
        after = os.fstat(stream.fileno())
    require(len(payload) <= 32768, "os-release exceeds size limit")
    require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
        "os-release changed while reading",
    )
    values: Dict[str, str] = {}
    for line in payload.splitlines():
        if not line or line.startswith("#"):
            continue
        require("=" in line, "os-release line is malformed")
        key, value = line.split("=", 1)
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
        values[key] = value
    architecture = run_collector("dpkg-architecture").stdout.strip()
    return {
        "os_id": values.get("ID", ""),
        "os_version_id": values.get("VERSION_ID", ""),
        "dpkg_architecture": architecture,
    }


def ubuntu_pro_posture() -> Dict[str, Any]:
    completed = run_collector("ubuntu-pro-status")
    try:
        status = json.loads(
            completed.stdout,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite,
        )
    except json.JSONDecodeError as exc:
        raise QemuInstallGateError(f"Ubuntu Pro status JSON is invalid: {exc}") from exc
    require(isinstance(status, dict), "Ubuntu Pro status is not an object")
    services = status.get("services")
    require(isinstance(services, list), "Ubuntu Pro service list is missing")
    enabled = sorted(
        item.get("name")
        for item in services
        if isinstance(item, dict) and item.get("status") == "enabled" and isinstance(item.get("name"), str)
    )
    return {
        "attached": status.get("attached") is True,
        "enabled_service_names": enabled,
        "operation_in_progress": status.get("execution_status") != "inactive",
    }


def verify_install_history(policy: Mapping[str, Any]) -> bool:
    fd, before = _stable_open(APT_HISTORY)
    with os.fdopen(fd, "r", encoding="utf-8", errors="replace") as stream:
        payload = stream.read(MAX_JSON_BYTES + 1)
        after = os.fstat(stream.fileno())
    require(len(payload) <= MAX_JSON_BYTES, "APT history exceeds audit size limit")
    require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
        "APT history changed while reading",
    )
    transaction = policy["recorded_install_transaction"]
    start_local = datetime.fromisoformat(transaction["started_utc"]).astimezone().strftime("%Y-%m-%d  %H:%M:%S")
    end_local = datetime.fromisoformat(transaction["ended_utc"]).astimezone().strftime("%Y-%m-%d  %H:%M:%S")
    matches = []
    for block in payload.split("\n\n"):
        if (
            f"Start-Date: {start_local}" in block
            and f"End-Date: {end_local}" in block
            and "Commandline: apt-get install --no-install-recommends qemu-system-x86 qemu-utils" in block
        ):
            matches.append(block)
    require(len(matches) == 1, "exact APT installation history block is missing or ambiguous")
    block = matches[0]
    require("Upgrade:" not in block and "Remove:" not in block and "Purge:" not in block, "APT history contains mutation outside install")
    install_lines = [line for line in block.splitlines() if line.startswith("Install: ")]
    require(len(install_lines) == 1, "APT history install line is missing or ambiguous")
    names = re.findall(r"(?:^|, )([a-z0-9][a-z0-9+.-]*)(?::[a-z0-9]+)? \(", install_lines[0][len("Install: "):])
    require(len(names) == 19 and set(names) == {item[0] for item in EXPECTED_PACKAGES}, "APT history package closure differs")
    return True


def validate_policy_semantics(policy: Mapping[str, Any]) -> None:
    require(policy["admission_scope"] == "EXACT_OBSERVED_INSTALLATION_STATE_ONLY", "policy scope differs")
    require(policy["development_only"] is True and policy["formal"] is False, "policy classification differs")
    require(policy["physical_primary_eligible"] is False, "policy physical eligibility differs")
    historical = policy["historical_evidence"]
    require(historical["predecessor_preserved"] is True, "predecessor is not preserved")
    require(historical["supersedes_predecessor"] is False, "policy unexpectedly supersedes predecessor")
    for key, (relative, digest) in zip(
        ("vm_policy_v1", "sandbox_gate_v1", "sandbox_schema_v1"), LEGACY_ARTIFACTS
    ):
        require(historical[key]["path"] == relative, f"legacy path differs: {key}")
        require(historical[key]["sha256"] == digest, f"legacy hash differs: {key}")
    require(historical["vm_policy_v1"]["system_install_authorized"] is False, "legacy install=false evidence changed")
    receipts = historical["final_preinstall_receipts"]
    require(
        [(item["path"], item["file_sha256"], item["receipt_hash"]) for item in receipts]
        == list(LEGACY_RECEIPTS),
        "legacy receipt set differs",
    )
    require(policy["host_platform"] == {"os_id": "ubuntu", "os_version_id": "20.04", "dpkg_architecture": "amd64"}, "host platform policy differs")
    require(
        policy["ubuntu_pro_posture"]
        == {
            "attached": True,
            "enabled_service_names_exact": ["esm-infra"],
            "operation_in_progress": False,
            "raw_status_must_not_be_recorded": True,
        },
        "Ubuntu Pro policy differs",
    )
    transaction = policy["recorded_install_transaction"]
    require(
        transaction["evidence_only"] is True
        and transaction["requested_packages"] == ["qemu-system-x86", "qemu-utils"]
        and transaction["no_install_recommends"] is True
        and (transaction["upgraded_count"], transaction["newly_installed_count"], transaction["removed_count"]) == (0, 19, 0)
        and transaction["future_package_mutation_authorized"] is False,
        "recorded installation transaction differs",
    )
    require(
        [(item["name"], item["architecture"], item["version"]) for item in policy["required_packages_exact"]]
        == list(EXPECTED_PACKAGES),
        "exact package policy differs",
    )
    require(all(item["status"] == "install ok installed" for item in policy["required_packages_exact"]), "package status policy differs")
    require(tuple(policy["required_qemu_namespace_exact"]) == EXPECTED_QEMU_NAMESPACE, "QEMU namespace policy differs")
    require(tuple(policy["required_absent_packages_exact"]) == EXPECTED_ABSENT_PACKAGES, "absent package policy differs")
    require(
        [
            (item["path"], item["dpkg_owner"], item["mode"], item["size"], item["sha256"])
            for item in policy["critical_files_exact"]
        ]
        == list(EXPECTED_CRITICAL_FILES),
        "critical file policy differs",
    )
    for item in policy["critical_files_exact"]:
        require(
            item["file_type"] == "regular"
            and item["symlink"] is False
            and item["uid"] == 0
            and item["gid"] == 0
            and item["security_capabilities"] == []
            and item["diverted"] is False,
            f"critical file invariant differs: {item['path']}",
        )
    require(
        policy["required_quiescence"]
        == {
            "qemu_kvm_service": {
                "load_state": "loaded",
                "unit_file_state": "disabled",
                "active_state": "inactive",
                "sub_state": "dead",
                "fragment_path": "/lib/systemd/system/qemu-kvm.service",
            },
            "qemu_kvm_autostart_link": str(QEMU_KVM_AUTOSTART_LINK),
            "qemu_kvm_autostart_link_required_absent": True,
            "ksm_run_path": str(KSM_RUN),
            "ksm_required_start_value": "0",
            "ksm_required_end_value": "0",
            "qemu_processes_required_empty": True,
            "unprivileged_process_visibility_uid_min": PROCESS_VISIBILITY_UID_MIN,
            "privileged_processes_are_configuration_trust_boundary": True,
            "unreadable_unprivileged_process_allowlist_exact": [
                dict(item) for item in UNREADABLE_UNPRIVILEGED_PROCESS_ALLOWLIST
            ],
        },
        "quiescence policy differs",
    )
    decision = policy["decision"]
    require(decision["observed_installation_state_admitted"] is True, "install-state decision differs")
    for key in (
        "future_package_mutation_authorized",
        "qemu_executed",
        "qemu_img_executed",
        "qemu_execution_admitted",
        "qemu_img_execution_admitted",
        "vm_started",
        "image_created",
        "image_creation_admitted",
        "build_started",
        "build_admitted",
        "gencase_started",
        "gencase_admitted",
        "upstream_code_executed",
    ):
        require(decision[key] is False, f"policy unexpectedly permits or records action: {key}")
    require(decision["status"] == "PASS_INSTALL_STATE_ONLY_EXECUTION_NOT_ADMITTED", "decision status differs")


def _normalized_host_safety(report: Mapping[str, Any]) -> Dict[str, Any]:
    simulation = report.get("simulation_process_check", {})
    filesystem = report.get("filesystem", {})
    return {
        "status": report.get("status"),
        "errors": list(report.get("errors", [])),
        "receipt_hash": report.get("receipt_hash"),
        "approved_root": report.get("approved_root"),
        "simulation_active_ports": list(simulation.get("active_ports", [])),
        "simulation_active_process_count": len(simulation.get("active_processes", [])),
        "storage_target": filesystem.get("target"),
        "storage_uuid": filesystem.get("uuid"),
    }


def collect_install_snapshot(policy: Mapping[str, Any]) -> Dict[str, Any]:
    """Collect one complete read-only A/B-comparable host snapshot."""

    ksm_start = read_ksm()
    service_start = service_observation()
    autostart_start = {
        "exists": QEMU_KVM_AUTOSTART_LINK.exists(),
        "is_symlink": QEMU_KVM_AUTOSTART_LINK.is_symlink(),
    }
    required_packages = [
        package_observation(item["name"])
        for item in policy["required_packages_exact"]
    ]
    absent_packages = [
        package_observation(name)
        for name in policy["required_absent_packages_exact"]
    ]
    namespace = qemu_namespace()
    verify = run_collector("dpkg-verify")
    verify_evidence = {
        "returncode": verify.returncode,
        "stdout_empty": not bool(verify.stdout),
        "stderr_empty": not bool(verify.stderr),
    }
    critical_files = []
    for expected in policy["critical_files_exact"]:
        path = Path(expected["path"])
        _, actual = file_hash_and_observation(path)
        actual["dpkg_owner"] = package_owner(path)
        actual["diverted"] = is_diverted(path)
        critical_files.append(actual)
    process_start = qemu_process_snapshot(critical_files)
    ksm_end = read_ksm()
    service_end = service_observation()
    autostart_end = {
        "exists": QEMU_KVM_AUTOSTART_LINK.exists(),
        "is_symlink": QEMU_KVM_AUTOSTART_LINK.is_symlink(),
    }
    process_end = qemu_process_snapshot(critical_files)
    return {
        "required_packages": required_packages,
        "required_absent_packages": absent_packages,
        "qemu_namespace_installed": namespace,
        "dpkg_verify": verify_evidence,
        "critical_files": critical_files,
        "service_start": service_start,
        "service_end": service_end,
        "autostart_start": autostart_start,
        "autostart_end": autostart_end,
        "ksm_start": ksm_start,
        "ksm_end": ksm_end,
        "process_start": process_start,
        "process_end": process_end,
    }


def install_snapshot_errors(
    snapshot: Mapping[str, Any],
    policy: Mapping[str, Any],
) -> List[str]:
    errors = []
    for expected, actual in zip(
        policy["required_packages_exact"], snapshot["required_packages"]
    ):
        if actual != {
            "name": expected["name"],
            "installed": True,
            "architecture": expected["architecture"],
            "version": expected["version"],
            "status": expected["status"],
        }:
            errors.append(f"required package differs: {expected['name']}")
    for actual in snapshot["required_absent_packages"]:
        if actual["installed"]:
            errors.append(f"forbidden package is installed: {actual['name']}")
    if snapshot["qemu_namespace_installed"] != list(
        policy["required_qemu_namespace_exact"]
    ):
        errors.append("installed QEMU namespace differs")
    verify = snapshot["dpkg_verify"]
    if verify != {"returncode": 0, "stdout_empty": True, "stderr_empty": True}:
        errors.append("dpkg verification is not clean")
    for expected, actual in zip(
        policy["critical_files_exact"], snapshot["critical_files"]
    ):
        for key in (
            "path",
            "dpkg_owner",
            "file_type",
            "symlink",
            "uid",
            "gid",
            "mode",
            "size",
            "sha256",
            "security_capabilities",
            "diverted",
        ):
            if actual[key] != expected[key]:
                errors.append(f"critical file differs at {expected['path']}:{key}")
    required = policy["required_quiescence"]
    process_map = {
        item["pid"]: item
        for item in (
            snapshot["process_start"]["offenders"]
            + snapshot["process_end"]["offenders"]
        )
    }
    processes = [process_map[pid] for pid in sorted(process_map)]
    visibility_errors = sorted(
        set(
            snapshot["process_start"]["visibility_errors"]
            + snapshot["process_end"]["visibility_errors"]
        )
    )
    for phase in ("start", "end"):
        errors.extend(
            quiescence_errors(
                service=snapshot[f"service_{phase}"],
                autostart_exists=snapshot[f"autostart_{phase}"]["exists"],
                autostart_is_symlink=snapshot[f"autostart_{phase}"]["is_symlink"],
                ksm_start=snapshot[f"ksm_{phase}"],
                ksm_end=snapshot[f"ksm_{phase}"],
                processes=processes,
                process_visibility_errors=visibility_errors,
                required=required,
            )
        )
    return sorted(set(errors))


def public_quiescence(snapshot: Mapping[str, Any]) -> Dict[str, Any]:
    process_map = {
        item["pid"]: item
        for item in (
            snapshot["process_start"]["offenders"]
            + snapshot["process_end"]["offenders"]
        )
    }
    return {
        "qemu_kvm_service": snapshot["service_end"],
        "autostart_link_exists": snapshot["autostart_end"]["exists"],
        "autostart_link_is_symlink": snapshot["autostart_end"]["is_symlink"],
        "ksm_start_value": snapshot["ksm_start"],
        "ksm_end_value": snapshot["ksm_end"],
        "qemu_processes": [process_map[pid] for pid in sorted(process_map)],
        "process_visibility_errors": sorted(
            set(
                snapshot["process_start"]["visibility_errors"]
                + snapshot["process_end"]["visibility_errors"]
            )
        ),
        "process_visibility_exceptions": sorted(
            {
                item["pid"]: item
                for item in (
                    snapshot["process_start"]["visibility_exceptions"]
                    + snapshot["process_end"]["visibility_exceptions"]
                )
            }.values(),
            key=lambda item: item["pid"],
        ),
        "process_visibility_uid_min": PROCESS_VISIBILITY_UID_MIN,
    }


def validate_report_semantics(
    report: Mapping[str, Any],
    policy: Mapping[str, Any],
) -> None:
    report_core = dict(report)
    receipt_hash = report_core.pop("receipt_hash", None)
    require(receipt_hash == safety.canonical_hash(report_core), "report receipt hash is invalid")
    require(
        (report["status"] == "PASS") == (report["errors"] == []),
        "PASS/error invariant differs",
    )
    require(
        report["observed_installation_state_admitted"]
        == (report["status"] == "PASS"),
        "admission/status invariant differs",
    )
    for key in (
        "future_package_mutation_authorized",
        "qemu_executed",
        "qemu_img_executed",
        "qemu_execution_admitted",
        "qemu_img_execution_admitted",
        "vm_started",
        "image_created",
        "image_creation_admitted",
        "build_started",
        "build_admitted",
        "gencase_started",
        "gencase_admitted",
        "upstream_code_executed",
    ):
        require(report[key] is False, f"report unexpectedly permits or records action: {key}")
    if report["status"] != "PASS":
        return

    require(report["host_platform"] == policy["host_platform"], "PASS report host platform differs")
    expected_pro = policy["ubuntu_pro_posture"]
    require(
        report["ubuntu_pro_posture"]
        == {
            "attached": expected_pro["attached"],
            "enabled_service_names": expected_pro["enabled_service_names_exact"],
            "operation_in_progress": expected_pro["operation_in_progress"],
        },
        "PASS report Ubuntu Pro posture differs",
    )
    host_safety = report["host_safety"]
    require(
        host_safety["status"] == "PASS"
        and host_safety["errors"] == []
        and host_safety["approved_root"] == str(safety.APPROVED_ROOT)
        and host_safety["simulation_active_ports"] == []
        and host_safety["simulation_active_process_count"] == 0,
        "PASS report host safety differs",
    )
    historical = report["historical_evidence"]
    require(
        historical["predecessor_preserved"] is True
        and historical["supersedes_predecessor"] is False,
        "PASS report predecessor semantics differ",
    )
    require(
        [
            (item["path"], item["expected_sha256"], item["actual_sha256"])
            for item in historical["artifacts"]
        ]
        == [(relative, digest, digest) for relative, digest in LEGACY_ARTIFACTS],
        "PASS report legacy artifacts differ",
    )
    require(
        [
            (
                item["path"],
                item["expected_file_sha256"],
                item["actual_file_sha256"],
                item["expected_receipt_hash"],
                item["actual_receipt_hash"],
            )
            for item in historical["receipts"]
        ]
        == [
            (path, file_hash, file_hash, inner_hash, inner_hash)
            for path, file_hash, inner_hash in LEGACY_RECEIPTS
        ],
        "PASS report legacy receipts differ",
    )
    package_state = report["package_state"]
    require(
        [
            (item["name"], item["architecture"], item["version"], item["status"], item["installed"])
            for item in package_state["required_packages"]
        ]
        == [
            (name, architecture, version, "install ok installed", True)
            for name, architecture, version in EXPECTED_PACKAGES
        ],
        "PASS report required package state differs",
    )
    require(
        [item["name"] for item in package_state["required_absent_packages"]]
        == list(EXPECTED_ABSENT_PACKAGES)
        and all(not item["installed"] for item in package_state["required_absent_packages"]),
        "PASS report absent package state differs",
    )
    require(
        package_state["qemu_namespace_installed"] == list(EXPECTED_QEMU_NAMESPACE),
        "PASS report QEMU namespace differs",
    )
    require(
        package_state["dpkg_verify"]
        == {"returncode": 0, "stdout_empty": True, "stderr_empty": True}
        and package_state["install_history_verified"] is True,
        "PASS report package verification differs",
    )
    require(len(report["critical_files"]) == len(policy["critical_files_exact"]), "PASS report critical file count differs")
    for expected, actual in zip(policy["critical_files_exact"], report["critical_files"]):
        for key in (
            "path",
            "dpkg_owner",
            "file_type",
            "symlink",
            "uid",
            "gid",
            "mode",
            "size",
            "sha256",
            "security_capabilities",
            "diverted",
        ):
            require(actual[key] == expected[key], f"PASS report critical file differs: {expected['path']}:{key}")
    quiescence = report["quiescence"]
    required_quiescence = policy["required_quiescence"]
    require(quiescence["qemu_kvm_service"] == required_quiescence["qemu_kvm_service"], "PASS report service state differs")
    require(
        quiescence["autostart_link_exists"] is False
        and quiescence["autostart_link_is_symlink"] is False,
        "PASS report autostart state differs",
    )
    require(
        (quiescence["ksm_start_value"], quiescence["ksm_end_value"])
        == ("0", "0"),
        "PASS report KSM state differs",
    )
    require(quiescence["qemu_processes"] == [], "PASS report has QEMU processes")
    require(quiescence["process_visibility_errors"] == [], "PASS report process visibility differs")
    require(
        quiescence["process_visibility_exceptions"]
        == [dict(item) for item in UNREADABLE_UNPRIVILEGED_PROCESS_ALLOWLIST],
        "PASS report process visibility exception identities differ",
    )
    require(
        quiescence["process_visibility_uid_min"] == PROCESS_VISIBILITY_UID_MIN,
        "PASS report process visibility scope differs",
    )
    consistency = report["collection_consistency"]
    canonical_snapshots = []
    for label in ("a", "b"):
        canonical_text = consistency[f"snapshot_{label}_canonical"]
        require(
            len(canonical_text.encode("utf-8")) <= 131072,
            f"PASS report snapshot {label.upper()} exceeds size limit",
        )
        try:
            snapshot = json.loads(
                canonical_text,
                object_pairs_hook=_reject_duplicate_keys,
                parse_constant=_reject_nonfinite,
            )
        except json.JSONDecodeError as exc:
            raise QemuInstallGateError(
                f"PASS report snapshot {label.upper()} is invalid JSON: {exc}"
            ) from exc
        require(isinstance(snapshot, dict), f"PASS report snapshot {label.upper()} is not an object")
        require(
            safety.canonical_bytes(snapshot).decode("utf-8") == canonical_text,
            f"PASS report snapshot {label.upper()} is not canonical",
        )
        require(
            safety.canonical_hash(snapshot) == consistency[f"snapshot_{label}_sha256"],
            f"PASS report snapshot {label.upper()} hash is not anchored",
        )
        canonical_snapshots.append(snapshot)
    require(
        consistency["snapshots_equal"] is True
        and consistency["snapshot_a_sha256"] == consistency["snapshot_b_sha256"]
        and canonical_snapshots[0] == canonical_snapshots[1],
        "PASS report A/B collection differs",
    )
    snapshot_b = canonical_snapshots[1]
    require(
        snapshot_b["required_packages"] == package_state["required_packages"]
        and snapshot_b["required_absent_packages"]
        == package_state["required_absent_packages"]
        and snapshot_b["qemu_namespace_installed"]
        == package_state["qemu_namespace_installed"]
        and snapshot_b["dpkg_verify"] == package_state["dpkg_verify"],
        "PASS report package evidence is detached from snapshot B",
    )
    require(
        snapshot_b["critical_files"] == report["critical_files"],
        "PASS report critical file evidence is detached from snapshot B",
    )
    require(
        public_quiescence(snapshot_b) == quiescence,
        "PASS report quiescence evidence is detached from snapshot B",
    )


def build_report() -> Dict[str, Any]:
    errors: List[str] = []
    admission_schema, admission_schema_hash = read_json_and_hash(ADMISSION_SCHEMA_PATH)
    preflight_schema, preflight_schema_hash = read_json_and_hash(PREFLIGHT_SCHEMA_PATH)
    policy, policy_hash = read_json_and_hash(POLICY_PATH)
    validate_with_schema(policy, admission_schema, "admission policy")
    validate_policy_semantics(policy)

    host_safety_raw = safety.build_preflight(
        estimated_case_bytes=0,
        require_simulation_stopped=True,
        require_gpu_idle=False,
    )
    host_safety = _normalized_host_safety(host_safety_raw)
    if host_safety["status"] != "PASS":
        errors.extend(f"host safety: {item}" for item in host_safety["errors"])

    artifact_checks = []
    for relative, expected_hash in LEGACY_ARTIFACTS:
        path = PACKAGE_ROOT / relative
        try:
            actual_hash = file_hash_and_observation(path)[0]
        except (QemuInstallGateError, safety.LiquidSafetyError, OSError) as exc:
            actual_hash = None
            errors.append(f"legacy artifact cannot be verified: {relative}: {exc}")
        artifact_checks.append(
            {"path": relative, "expected_sha256": expected_hash, "actual_sha256": actual_hash}
        )
        if actual_hash != expected_hash:
            errors.append(f"legacy artifact hash differs: {relative}")

    receipt_checks = []
    for path_text, expected_file_hash, expected_receipt_hash in LEGACY_RECEIPTS:
        path = Path(path_text)
        try:
            receipt, actual_file_hash = read_json_and_hash(path)
            actual_receipt_hash = receipt.get("receipt_hash")
            receipt_core = dict(receipt)
            receipt_core.pop("receipt_hash", None)
            if actual_receipt_hash != safety.canonical_hash(receipt_core):
                errors.append(f"legacy receipt self-hash is invalid: {path}")
        except (QemuInstallGateError, safety.LiquidSafetyError, OSError, ValueError) as exc:
            actual_file_hash = None
            actual_receipt_hash = None
            errors.append(f"legacy receipt cannot be verified: {path}: {exc}")
        receipt_checks.append(
            {
                "path": path_text,
                "expected_file_sha256": expected_file_hash,
                "actual_file_sha256": actual_file_hash,
                "expected_receipt_hash": expected_receipt_hash,
                "actual_receipt_hash": actual_receipt_hash,
            }
        )
        if actual_file_hash != expected_file_hash or actual_receipt_hash != expected_receipt_hash:
            errors.append(f"legacy receipt identity differs: {path}")

    platform = host_platform()
    if platform != policy["host_platform"]:
        errors.append("host platform differs from admission")
    pro_posture = ubuntu_pro_posture()
    expected_pro = policy["ubuntu_pro_posture"]
    if pro_posture != {
        "attached": expected_pro["attached"],
        "enabled_service_names": expected_pro["enabled_service_names_exact"],
        "operation_in_progress": expected_pro["operation_in_progress"],
    }:
        errors.append("Ubuntu Pro posture differs from admission")
    try:
        history_match = verify_install_history(policy)
    except (QemuInstallGateError, OSError) as exc:
        history_match = False
        errors.append(f"APT install history differs: {exc}")

    if not history_match:
        errors.append("recorded APT transaction is not verified")
    try:
        snapshot_a = collect_install_snapshot(policy)
        snapshot_b = collect_install_snapshot(policy)
    except (QemuInstallGateError, safety.LiquidSafetyError, OSError) as exc:
        raise QemuInstallGateError(f"installation snapshot cannot be collected: {exc}") from exc
    snapshot_a_hash = safety.canonical_hash(snapshot_a)
    snapshot_b_hash = safety.canonical_hash(snapshot_b)
    snapshot_a_canonical = safety.canonical_bytes(snapshot_a).decode("utf-8")
    snapshot_b_canonical = safety.canonical_bytes(snapshot_b).decode("utf-8")
    snapshots_equal = snapshot_a == snapshot_b
    if not snapshots_equal:
        errors.append("installation state changed across A/B collection")
    errors.extend(f"snapshot A: {item}" for item in install_snapshot_errors(snapshot_a, policy))
    errors.extend(f"snapshot B: {item}" for item in install_snapshot_errors(snapshot_b, policy))
    quiescence = public_quiescence(snapshot_b)

    core: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "approved_root": str(safety.APPROVED_ROOT),
        "policy": {
            "path": str(POLICY_PATH),
            "sha256": policy_hash,
            "admission_schema_path": str(ADMISSION_SCHEMA_PATH),
            "admission_schema_sha256": admission_schema_hash,
            "preflight_schema_path": str(PREFLIGHT_SCHEMA_PATH),
            "preflight_schema_sha256": preflight_schema_hash,
        },
        "historical_evidence": {
            "artifacts": artifact_checks,
            "receipts": receipt_checks,
            "predecessor_preserved": True,
            "supersedes_predecessor": False,
        },
        "host_platform": platform,
        "ubuntu_pro_posture": pro_posture,
        "host_safety": host_safety,
        "package_state": {
            "required_packages": snapshot_b["required_packages"],
            "required_absent_packages": snapshot_b["required_absent_packages"],
            "qemu_namespace_installed": snapshot_b["qemu_namespace_installed"],
            "dpkg_verify": snapshot_b["dpkg_verify"],
            "install_history_verified": history_match,
        },
        "critical_files": snapshot_b["critical_files"],
        "quiescence": quiescence,
        "collection_consistency": {
            "snapshot_a_sha256": snapshot_a_hash,
            "snapshot_b_sha256": snapshot_b_hash,
            "snapshot_a_canonical": snapshot_a_canonical,
            "snapshot_b_canonical": snapshot_b_canonical,
            "snapshots_equal": snapshots_equal,
        },
        "observed_installation_state_admitted": not errors,
        "future_package_mutation_authorized": False,
        "qemu_executed": False,
        "qemu_img_executed": False,
        "qemu_execution_admitted": False,
        "qemu_img_execution_admitted": False,
        "vm_started": False,
        "image_created": False,
        "image_creation_admitted": False,
        "build_started": False,
        "build_admitted": False,
        "gencase_started": False,
        "gencase_admitted": False,
        "upstream_code_executed": False,
        "status": "PASS" if not errors else "NO_GO",
        "errors": errors,
    }
    report = dict(core, receipt_hash=safety.canonical_hash(core))
    validate_with_schema(report, preflight_schema, "preflight report")
    validate_report_semantics(report, policy)
    return report


def validate_receipt_path(path: Path) -> Path:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    path = safety.ensure_within_approved_root(path, root=root)
    require(path.parent == root / "audits/sandbox", "receipt must be directly under audits/sandbox")
    require(RECEIPT_RE.fullmatch(path.name) is not None, "receipt filename is invalid")
    require(not path.exists() and not path.is_symlink(), f"receipt already exists: {path}")
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check", help="read-only installed-QEMU audit")
    receipt = sub.add_parser("write-receipt", help="create one new PASS-only receipt")
    receipt.add_argument("--receipt", type=Path, required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        receipt_path = validate_receipt_path(args.receipt) if args.command == "write-receipt" else None
        report = build_report()
        if receipt_path is not None:
            require(report["status"] == "PASS", "NO_GO result is never published as an admission receipt")
            safety.atomic_write_json_new(receipt_path, report)
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if report["status"] == "PASS" else 2
    except (
        QemuInstallGateError,
        safety.LiquidSafetyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
        SchemaError,
    ) as exc:
        print(f"R8_LIQUID_QEMU_INSTALL_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
