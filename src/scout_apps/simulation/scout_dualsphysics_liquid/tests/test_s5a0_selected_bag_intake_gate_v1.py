#!/usr/bin/env python3
"""Mock/static-only tests for the S5A0 selected-bag intake contract."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import stat
import struct
import sys
import tempfile
import unittest
from dataclasses import asdict
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
GATE_PATH = SCRIPTS / "r8_liquid_s5a0_selected_bag_intake_gate_v1.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_ros1_bag_v2_reader_v1 as reader  # noqa: E402
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate  # noqa: E402


def field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = b"".join(field(name, value) for name, value in fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def ros_time(value_ns: int) -> bytes:
    return struct.pack("<II", value_ns // 1_000_000_000, value_ns % 1_000_000_000)


def minimal_bag(*, conflict: bool = False, corrupt_index: bool = False) -> bytes:
    specs = [(0, "/fixture", "fixture_msgs/Value", "1" * 32, b"float32 x\n", b"\x00")]
    if conflict:
        specs.append((1, "/fixture", "fixture_msgs/Value", "1" * 32, b"float32 y\n", b"\x01"))
    inner_parts: list[bytes] = []
    offsets: list[int] = []
    times: list[int] = []
    offset = 0
    for index, (connection, _topic, _type, _md5, _definition, payload) in enumerate(specs):
        time_ns = 1_000_000_000 + index
        message = record(
            [("op", bytes([reader.OP_MSG_DATA])), ("conn", struct.pack("<I", connection)), ("time", ros_time(time_ns))],
            payload,
        )
        offsets.append(offset)
        times.append(time_ns)
        inner_parts.append(message)
        offset += len(message)
    inner = b"".join(inner_parts)
    header_zero = record(
        [
            ("op", bytes([reader.OP_FILE_HEADER])),
            ("index_pos", struct.pack("<Q", 0)),
            ("conn_count", struct.pack("<I", len(specs))),
            ("chunk_count", struct.pack("<I", 1)),
        ]
    )
    chunk_pos = len(reader.MAGIC) + len(header_zero)
    chunk = record(
        [("op", bytes([reader.OP_CHUNK])), ("compression", b"none"), ("size", struct.pack("<I", len(inner)))],
        inner,
    )
    indexes = []
    for index, (connection, *_rest) in enumerate(specs):
        message_offset = offsets[index] + (1 if corrupt_index and index == 0 else 0)
        indexes.append(
            record(
                [
                    ("op", bytes([reader.OP_INDEX_DATA])),
                    ("ver", struct.pack("<I", 1)),
                    ("conn", struct.pack("<I", connection)),
                    ("count", struct.pack("<I", 1)),
                ],
                ros_time(times[index]) + struct.pack("<I", message_offset),
            )
        )
    index_pos = chunk_pos + len(chunk) + sum(map(len, indexes))
    file_header = record(
        [
            ("op", bytes([reader.OP_FILE_HEADER])),
            ("index_pos", struct.pack("<Q", index_pos)),
            ("conn_count", struct.pack("<I", len(specs))),
            ("chunk_count", struct.pack("<I", 1)),
        ]
    )
    connections = []
    for connection, topic, message_type, md5sum, definition, _payload in specs:
        data = b"".join(
            [
                field("type", message_type.encode()),
                field("md5sum", md5sum.encode()),
                field("message_definition", definition),
                field("callerid", b"/fixture"),
            ]
        )
        connections.append(
            record(
                [("op", bytes([reader.OP_CONNECTION])), ("conn", struct.pack("<I", connection)), ("topic", topic.encode())],
                data,
            )
        )
    counts = b"".join(struct.pack("<II", connection, 1) for connection, *_ in specs)
    chunk_info = record(
        [
            ("op", bytes([reader.OP_CHUNK_INFO])),
            ("ver", struct.pack("<I", 1)),
            ("chunk_pos", struct.pack("<Q", chunk_pos)),
            ("start_time", ros_time(min(times))),
            ("end_time", ros_time(max(times))),
            ("count", struct.pack("<I", len(specs))),
        ],
        counts,
    )
    return reader.MAGIC + file_header + chunk + b"".join(indexes + connections) + chunk_info


def time_range(value: int) -> dict[str, object]:
    return {"first_ns": value, "last_ns": value, "minimum_ns": value, "maximum_ns": value, "monotonic": True}


class S5A0SelectedBagIntakeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.source_root = root / "source"
        self.attempt = self.source_root / "SIM-TEST-C1-Bsmooth-b01-r01"
        self.audit = root / "audit"
        self.attempt.mkdir(parents=True)
        self.audit.mkdir()
        self.source = self.attempt / "capture.bag"
        self.source.write_bytes(b"fixture-bag-identity")
        self.source.chmod(0o755)
        frozen, _ = gate.load_policy()
        self.policy = copy.deepcopy(frozen)
        selection = self.policy["selection"]
        selection["source_root"] = str(self.source_root)
        selection["attempt_id"] = self.attempt.name
        selection["relative_path"] = f"{self.attempt.name}/capture.bag"
        selection["absolute_path"] = str(self.source)
        selection["forbidden_optional_path"] = str(self.source_root / "SIM-TEST-C1-Bslosh-b01-r01/capture.bag")
        selection["expected_size_bytes"] = self.source.stat().st_size
        selection["expected_mode"] = "0755"
        selection["expected_sha256"] = hashlib.sha256(self.source.read_bytes()).hexdigest()
        self.policy["sandbox_contract"]["audit_root"] = str(self.audit)
        gate.validate_policy(self.policy)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def fake_inspection(self) -> dict[str, object]:
        metadata = self.source.stat()
        digest = hashlib.sha256(self.source.read_bytes()).hexdigest()
        identity = {
            "path": str(self.source), "sha256": digest, "size_bytes": metadata.st_size,
            "mode": "0755", "uid": metadata.st_uid, "gid": metadata.st_gid,
            "device": metadata.st_dev, "inode": metadata.st_ino, "nlink": 1,
            "mtime_ns": metadata.st_mtime_ns, "ctime_ns": metadata.st_ctime_ns,
        }
        topics = []
        connections = []
        for connection_id, expected in enumerate(sorted(self.policy["topic_contract"]["required_topics"], key=lambda row: row["topic"])):
            definition_hash = hashlib.sha256(expected["type"].encode()).hexdigest()
            row = {
                **expected, "message_definition_sha256": definition_hash,
                "connection_ids": [connection_id], "message_count": 1,
                "record_time_range": time_range(1_000_000_000 + connection_id),
                "header_time_range": time_range(1_000_000_000 + connection_id) if expected["topic"] == "/odom" else None,
                "header_time_monotonic": True if expected["topic"] == "/odom" else None,
            }
            topics.append(row)
            connections.append({
                "connection_id": connection_id, **expected,
                "message_definition_sha256": definition_hash, "callerid": None,
                "latching": None, "message_count": 1,
                "record_time_range": time_range(1_000_000_000 + connection_id),
            })
        bag = {
            "format": "ROS1_BAG_V2", "version": "2.0", "indexed": True,
            "file_header": {"index_pos": 100, "connection_count": 13, "chunk_count": 1},
            "compression": ["none"], "total_message_count": 13,
            "record_time_range": time_range(1_000_000_000), "connections": connections,
            "topics": topics,
            "odom": {"message_count": 1, "record_time_range": time_range(1_000_000_002),
                     "header_time_range": time_range(1_000_000_002), "record_time_monotonic": True,
                     "header_time_monotonic": True, "header_record_max_abs_delta_ns": 0,
                     "frame_pairs": [{"frame_id": "odom", "child_frame_id": "base_footprint", "message_count": 1}]},
            "clock": {"message_count": 1, "record_time_range": time_range(1_000_000_000),
                      "clock_time_range": time_range(1_000_000_000), "record_time_monotonic": True,
                      "clock_time_monotonic": True},
            "frame_graph": {
                "dynamic_edges": [{"parent_frame": "odom", "child_frame": "base_footprint", "message_count": 1,
                                   "first_record_time_ns": 1, "last_record_time_ns": 1}],
                "static_edges": [{"parent_frame": "base_footprint", "child_frame": "base_link", "message_count": 1,
                                  "first_record_time_ns": 1, "last_record_time_ns": 1}],
            },
            "index_verified": True, "chunk_info_verified": True, "topic_conflicts": [], "anomalies": [],
            "limits": asdict(gate.reader_limits_from_policy(self.policy)),
        }
        return {"source_before": identity, "source_after": copy.deepcopy(identity), "bag": bag}

    def ro_mountinfo(self) -> str:
        return f"100 1 0:1 / {self.source_root} ro,nosuid,nodev - tmpfs tmpfs ro\n"

    def build_receipt(self) -> dict[str, object]:
        with mock.patch.object(gate.bag_reader, "inspect_bag", return_value=self.fake_inspection()):
            return gate.inspect_selected(
                self.policy, source_path=self.source, receipt_path=self.audit / "receipt.json",
                mountinfo_text=self.ro_mountinfo(), statvfs_flags=getattr(os, "ST_RDONLY", 1),
                captured_at_utc="2026-08-12T00:00:00Z", snapshot_root=self.source_root,
            )

    def test_valid_receipt_is_deep_closed_and_create_new(self) -> None:
        receipt = self.build_receipt()
        schema = gate.load_schema()
        gate.assert_deep_closed(schema)
        self.assertEqual([], list(Draft202012Validator(schema).iter_errors(receipt)))
        before = gate.snapshot_source_root(self.source_root, 20)
        result = gate.write_receipt_create_new(self.audit / "receipt.json", receipt, maximum_bytes=2_097_152)
        self.assertEqual(result["mode"], "0440")
        self.assertEqual(before, gate.snapshot_source_root(self.source_root, 20))
        with self.assertRaisesRegex(gate.S5A0GateError, "RECEIPT_CREATE_NEW_FAILED"):
            gate.write_receipt_create_new(self.audit / "receipt.json", receipt, maximum_bytes=2_097_152)
        tampered = copy.deepcopy(receipt)
        tampered["claims"]["formal"] = True
        self.assertTrue(list(Draft202012Validator(schema).iter_errors(tampered)))

    def test_reader_rejects_malformed_truncated_and_corrupt_index(self) -> None:
        valid = minimal_bag()
        self.assertTrue(reader.parse_bag_v2(valid)["index_verified"])
        for broken in (b"not-a-bag", valid[:-1], minimal_bag(corrupt_index=True)):
            with self.subTest(size=len(broken)), self.assertRaises(reader.BagV2Error):
                reader.parse_bag_v2(broken)

    def test_reader_rejects_same_topic_definition_conflict(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "definitions conflict"):
            reader.parse_bag_v2(minimal_bag(conflict=True))

    def test_exact_path_and_optional_path_fail_closed(self) -> None:
        with self.assertRaisesRegex(gate.S5A0GateError, "EXACT_PATH_MISMATCH"):
            gate.validate_selected_path(self.policy, str(self.source_root / "latest.bag"))
        with self.assertRaisesRegex(gate.S5A0GateError, "OPTIONAL_PAIR_NOT_AUTHORIZED"):
            gate.validate_selected_path(self.policy, self.policy["selection"]["forbidden_optional_path"])

    def test_symlink_hardlink_active_marker_and_rw_mount_are_rejected(self) -> None:
        link = self.attempt / "link.bag"
        link.symlink_to(self.source)
        with self.assertRaisesRegex(gate.S5A0GateError, "SYMLINK_FORBIDDEN"):
            gate.audit_path_chain(link, 32)
        link.unlink()
        hardlink = self.attempt / "hardlink.bag"
        os.link(self.source, hardlink)
        with self.assertRaisesRegex(gate.S5A0GateError, "SOURCE_NLINK_INVALID"):
            gate.audit_path_chain(self.source, 32)
        hardlink.unlink()
        (self.attempt / ".active").write_text("active", encoding="utf-8")
        with self.assertRaisesRegex(gate.S5A0GateError, "ACTIVE_MARKER_FORBIDDEN"):
            gate.validate_attempt_inventory(self.source, ["capture.bag"])
        (self.attempt / ".active").unlink()
        with self.assertRaisesRegex(gate.S5A0GateError, "READ_ONLY_MOUNT_NOT_ENFORCED"):
            gate.inspect_read_only_mount(
                self.source, self.ro_mountinfo().replace(" ro,", " rw,"), statvfs_flags=0
            )

    def test_missing_extra_wrong_type_and_nonmonotonic_topics_are_rejected(self) -> None:
        bag = self.fake_inspection()["bag"]
        mutations = [
            lambda value: value["topics"].pop(),
            lambda value: value["topics"].append({**value["topics"][0], "topic": "/extra"}),
            lambda value: value["topics"][0].__setitem__("type", "wrong/Type"),
            lambda value: value["topics"][0]["record_time_range"].__setitem__("monotonic", False),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(bag)
            mutate(changed)
            with self.subTest(mutation=mutate), self.assertRaises(gate.S5A0GateError):
                gate.validate_topic_contract(self.policy, changed)

    def test_bwrap_plan_is_read_only_offline_bounded_and_never_executes_bag(self) -> None:
        argv = gate.build_bwrap_argv(self.policy, str(self.audit / "receipt.json"))
        delimiter = argv.index("--")
        self.assertIn("--unshare-net", argv[:delimiter])
        self.assertIn("--ro-bind", argv[:delimiter])
        self.assertEqual(argv.count(str(self.source)), 1)
        self.assertNotIn(str(self.source), argv[delimiter + 1 :])
        self.assertNotIn(self.policy["selection"]["forbidden_optional_path"], argv)
        self.assertTrue(any(item.startswith("--rss=") for item in argv))
        self.assertTrue(any(item.startswith("--as=") for item in argv))

    def test_gate_ast_has_no_execution_network_ros_or_solver_interface(self) -> None:
        tree = ast.parse(GATE_PATH.read_text(encoding="utf-8"))
        forbidden_imports = {"subprocess", "socket", "requests", "urllib", "rospy", "rosbag", "rclpy"}
        roots = set()
        forbidden_calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                roots.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr.startswith(("exec", "spawn")) or node.func.attr in {"system", "popen"}:
                    forbidden_calls.append(node.func.attr)
        self.assertFalse(roots & forbidden_imports)
        self.assertEqual([], forbidden_calls)

    def test_self_check_does_not_probe_a_real_bag(self) -> None:
        with mock.patch.object(gate, "inspect_selected", side_effect=AssertionError("bag touched")), \
             mock.patch.object(gate.os, "lstat", side_effect=AssertionError("source stat")), \
             mock.patch.object(gate.os, "open", side_effect=AssertionError("source open")):
            result = gate.self_check()
        self.assertFalse(result["real_bag_opened"])
        self.assertFalse(result["bwrap_executed"])


if __name__ == "__main__":
    unittest.main()
