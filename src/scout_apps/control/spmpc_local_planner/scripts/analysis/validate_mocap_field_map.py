#!/usr/bin/env python3
"""Fail-closed offline validation for a frozen Cartographer map asset set."""

import argparse
import hashlib
import json
import math
import pathlib
import re
import sys

import yaml


SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
MAP_SUFFIXES = (".pbstream", ".yaml", ".pgm")


def sha256_file(path):
    digest = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_map_stem(value):
    path = pathlib.Path(value).expanduser()
    if path.suffix.lower() in MAP_SUFFIXES:
        path = path.with_suffix("")
    return path.resolve(strict=False)


def asset_paths(stem):
    normalized = normalize_map_stem(stem)
    return {
        "pbstream": normalized.with_suffix(".pbstream"),
        "yaml": normalized.with_suffix(".yaml"),
        "pgm": normalized.with_suffix(".pgm"),
    }


def _next_pnm_token(data, offset):
    length = len(data)
    while offset < length:
        byte = data[offset]
        if byte in b" \t\r\n\v\f":
            offset += 1
            continue
        if byte == ord("#"):
            newline = data.find(b"\n", offset + 1)
            offset = length if newline < 0 else newline + 1
            continue
        break
    if offset >= length:
        raise ValueError("unexpected end of PGM header")
    start = offset
    while offset < length and data[offset] not in b" \t\r\n\v\f#":
        offset += 1
    return data[start:offset].decode("ascii"), offset


def parse_pgm(path):
    data = pathlib.Path(path).read_bytes()
    tokens = []
    offset = 0
    for _ in range(4):
        token, offset = _next_pnm_token(data, offset)
        tokens.append(token)

    magic = tokens[0]
    if magic not in ("P2", "P5"):
        raise ValueError("PGM magic must be P2 or P5, got {!r}".format(magic))
    try:
        width, height, max_value = (int(value) for value in tokens[1:])
    except ValueError as exc:
        raise ValueError("PGM width, height and max value must be integers") from exc
    if width <= 0 or height <= 0:
        raise ValueError("PGM dimensions must be positive")
    if not 1 <= max_value <= 65535:
        raise ValueError("PGM max value must be in [1, 65535]")

    sample_count = width * height
    if magic == "P5":
        if offset >= len(data) or data[offset] not in b" \t\r\n\v\f":
            raise ValueError("PGM binary header is missing its data separator")
        if data[offset] == ord("\r") and offset + 1 < len(data) and data[offset + 1] == ord("\n"):
            payload_offset = offset + 2
        else:
            payload_offset = offset + 1
        expected_bytes = sample_count * (1 if max_value < 256 else 2)
        payload_bytes = len(data) - payload_offset
        if payload_bytes != expected_bytes:
            raise ValueError(
                "PGM payload size mismatch: expected {}, got {}".format(
                    expected_bytes, payload_bytes
                )
            )
    else:
        values = []
        while True:
            try:
                token, offset = _next_pnm_token(data, offset)
            except ValueError:
                break
            try:
                values.append(int(token))
            except ValueError as exc:
                raise ValueError("PGM ASCII payload contains a non-integer") from exc
        if len(values) != sample_count:
            raise ValueError(
                "PGM sample count mismatch: expected {}, got {}".format(
                    sample_count, len(values)
                )
            )
        if values and (min(values) < 0 or max(values) > max_value):
            raise ValueError("PGM ASCII sample is outside the declared range")
        expected_bytes = None
        payload_bytes = None

    return {
        "magic": magic,
        "width": width,
        "height": height,
        "max_value": max_value,
        "sample_count": sample_count,
        "expected_payload_bytes": expected_bytes,
        "payload_bytes": payload_bytes,
    }


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _resolve_yaml_image(yaml_path, image_value):
    image_path = pathlib.Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return image_path.resolve(strict=False)


def validate_map_assets(
    stem,
    expected_resolution=0.02,
    expected_pbstream_sha256="",
    expected_yaml_sha256="",
    expected_pgm_sha256="",
):
    """Return a JSON-serializable report without importing or contacting ROS."""

    stem = normalize_map_stem(stem)
    paths = asset_paths(stem)
    failures = []
    assets = {}

    expected_hashes = {
        "pbstream": expected_pbstream_sha256.strip().lower(),
        "yaml": expected_yaml_sha256.strip().lower(),
        "pgm": expected_pgm_sha256.strip().lower(),
    }
    for kind, expected in expected_hashes.items():
        if expected and not SHA256_RE.fullmatch(expected):
            failures.append("{} expected SHA-256 is not 64 hexadecimal characters".format(kind))

    for kind, path in paths.items():
        entry = {"path": str(path), "exists": path.is_file(), "size_bytes": 0, "sha256": ""}
        if not path.is_file():
            failures.append("missing {} asset: {}".format(kind, path))
        else:
            entry["size_bytes"] = path.stat().st_size
            if entry["size_bytes"] <= 0:
                failures.append("empty {} asset: {}".format(kind, path))
            else:
                entry["sha256"] = sha256_file(path)
                expected = expected_hashes[kind]
                if expected and entry["sha256"] != expected:
                    failures.append(
                        "{} SHA-256 mismatch: expected={}, actual={}".format(
                            kind, expected, entry["sha256"]
                        )
                    )
        assets[kind] = entry

    yaml_summary = {}
    yaml_path = paths["yaml"]
    if yaml_path.is_file() and yaml_path.stat().st_size > 0:
        try:
            payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("map YAML root must be a mapping")

            required_keys = (
                "image",
                "resolution",
                "origin",
                "negate",
                "occupied_thresh",
                "free_thresh",
            )
            missing_keys = [key for key in required_keys if key not in payload]
            if missing_keys:
                failures.append("map YAML missing keys: {}".format(", ".join(missing_keys)))

            image_value = payload.get("image")
            if not isinstance(image_value, str) or not image_value.strip():
                failures.append("map YAML image must be a non-empty path string")
                resolved_image = None
            else:
                resolved_image = _resolve_yaml_image(yaml_path, image_value)
                if resolved_image != paths["pgm"]:
                    failures.append(
                        "map YAML image resolves to {}, expected {}".format(
                            resolved_image, paths["pgm"]
                        )
                    )

            resolution = payload.get("resolution")
            if not _finite_number(resolution) or float(resolution) <= 0.0:
                failures.append("map YAML resolution must be a finite positive number")
            elif expected_resolution is not None and not math.isclose(
                float(resolution), float(expected_resolution), rel_tol=0.0, abs_tol=1e-9
            ):
                failures.append(
                    "map YAML resolution mismatch: expected={}, actual={}".format(
                        expected_resolution, resolution
                    )
                )

            origin = payload.get("origin")
            if (
                not isinstance(origin, (list, tuple))
                or len(origin) != 3
                or not all(_finite_number(value) for value in origin)
            ):
                failures.append("map YAML origin must contain three finite numbers")

            negate = payload.get("negate")
            if negate not in (0, 1, False, True):
                failures.append("map YAML negate must be 0 or 1")

            occupied = payload.get("occupied_thresh")
            free = payload.get("free_thresh")
            if not _finite_number(occupied) or not 0.0 <= float(occupied) <= 1.0:
                failures.append("map YAML occupied_thresh must be in [0, 1]")
            if not _finite_number(free) or not 0.0 <= float(free) <= 1.0:
                failures.append("map YAML free_thresh must be in [0, 1]")
            if _finite_number(occupied) and _finite_number(free) and float(free) >= float(occupied):
                failures.append("map YAML free_thresh must be less than occupied_thresh")

            yaml_summary = {
                "image": image_value,
                "resolved_image": str(resolved_image) if resolved_image else "",
                "resolution": resolution,
                "origin": origin,
                "negate": negate,
                "occupied_thresh": occupied,
                "free_thresh": free,
            }
        except (OSError, UnicodeError, yaml.YAMLError, ValueError) as exc:
            failures.append("could not parse map YAML: {}".format(exc))

    pgm_summary = {}
    pgm_path = paths["pgm"]
    if pgm_path.is_file() and pgm_path.stat().st_size > 0:
        try:
            pgm_summary = parse_pgm(pgm_path)
        except (OSError, UnicodeError, ValueError) as exc:
            failures.append("invalid PGM: {}".format(exc))

    return {
        "schema_version": 1,
        "protocol_id": "SMPCC_mocap_field_map_v1",
        "pass": not failures,
        "map_stem": str(stem),
        "expected": {
            "resolution": expected_resolution,
            "sha256": expected_hashes,
        },
        "assets": assets,
        "yaml": yaml_summary,
        "pgm": pgm_summary,
        "failures": failures,
    }


def _write_json(path, payload):
    output = pathlib.Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_manifest(path, report):
    output = pathlib.Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for kind in ("pbstream", "yaml", "pgm"):
        asset = report["assets"][kind]
        lines.append("{}  {}".format(asset["sha256"], asset["path"]))
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_stem", help="map stem, or any matching .pbstream/.yaml/.pgm path")
    parser.add_argument("--expected-resolution", type=float, default=0.02)
    parser.add_argument("--expected-pbstream-sha256", default="")
    parser.add_argument("--expected-yaml-sha256", default="")
    parser.add_argument("--expected-pgm-sha256", default="")
    parser.add_argument("--report", default="")
    parser.add_argument("--sha256-manifest", default="")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = validate_map_assets(
        args.map_stem,
        expected_resolution=args.expected_resolution,
        expected_pbstream_sha256=args.expected_pbstream_sha256,
        expected_yaml_sha256=args.expected_yaml_sha256,
        expected_pgm_sha256=args.expected_pgm_sha256,
    )
    if args.report:
        _write_json(args.report, report)
    if args.sha256_manifest and report["pass"]:
        _write_manifest(args.sha256_manifest, report)

    if report["pass"]:
        print("[validate_mocap_field_map] PASS: {}".format(report["map_stem"]))
        for kind in ("pbstream", "yaml", "pgm"):
            asset = report["assets"][kind]
            print("  {} sha256={}".format(kind, asset["sha256"]))
        return 0

    print("[validate_mocap_field_map] FAIL: {}".format(report["map_stem"]), file=sys.stderr)
    for failure in report["failures"]:
        print("  - {}".format(failure), file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
