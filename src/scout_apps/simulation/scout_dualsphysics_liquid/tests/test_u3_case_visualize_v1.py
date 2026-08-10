#!/usr/bin/env python3
"""Static/data-only tests for U3 case cross-validation and visualization export."""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = PACKAGE_DIR / "scripts"
TEST_DIR = PACKAGE_DIR / "tests"
for path in (SCRIPT_DIR, TEST_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from test_bi4_reader_v1 import make_bi4  # noqa: E402
from r8_liquid_bi4_reader_v1 import (  # noqa: E402
    Bi4FormatError,
    SecureFile,
    extract_u3_particles,
    parse_jpartdata_bi4,
)
import r8_liquid_u3_case_visualize_v1 as visualizer  # noqa: E402


VALID_XML = b"""<?xml version="1.0"?>
<case app="GenCase test" date="test">
  <casedef><constantsdef><rhopgradient value="2"/></constantsdef></casedef>
  <execution>
    <parameters>
      <parameter key="RhopOutMin" value="700"/>
      <parameter key="RhopOutMax" value="1300"/>
    </parameters>
    <particles np="4" nb="2" nbf="2">
      <_summary><positions>
        <posmin x="-0.001" y="0" z="0"/>
        <posmax x="0.001" y="0" z="0.006"/>
      </positions></_summary>
      <fixed begin="0" count="2"/><fluid begin="2" count="2"/>
    </particles>
    <constants><dp value="0.002"/><rhop0 value="1000"/></constants>
    <motion/>
  </execution>
</case>
"""

VALID_OUT = b"""Distance between points (Dp): 0.002
Particle summary:
  Fixed....: 2  id:(0-1)
  Moving...: 0
  Floating.: 0
  Fluid....: 2  id:(2-3)
Total particles: 4 (bound=2 (fx=2 mv=0 ft=0) fluid=2)
Particle limits:
  X range: -0.001 to 0.001 [m]
  Y range: 0 to 0 [m]
  Z range: 0 to 0.006 [m]
Finished execution (code=0).
"""


def secure(name: str, data: bytes) -> SecureFile:
    return SecureFile(
        path=Path(name),
        data=data,
        sha256=hashlib.sha256(data).hexdigest(),
        size_bytes=len(data),
        mode=0o440,
        uid=1000,
        gid=1000,
    )


class U3CaseVisualizerV1Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = parse_jpartdata_bi4(make_bi4())
        self.particles = extract_u3_particles(self.root)

    def test_three_sources_cross_validate(self) -> None:
        xml = visualizer.parse_case_xml(secure("case.xml", VALID_XML))
        out = visualizer.parse_gencase_out(secure("case.out", VALID_OUT))
        result = visualizer.validate_cross_sources(
            self.root.values, self.particles, xml, out
        )
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(all(result["checks"].values()))

    def test_nonempty_motion_is_rejected(self) -> None:
        changed = VALID_XML.replace(b"<motion/>", b"<motion><obj/></motion>")
        with self.assertRaisesRegex(Bi4FormatError, "motion element is not uniquely empty"):
            visualizer.parse_case_xml(secure("case.xml", changed))

    def test_out_without_success_marker_is_rejected(self) -> None:
        changed = VALID_OUT.replace(b"Finished execution (code=0).", b"Stopped.")
        with self.assertRaisesRegex(Bi4FormatError, "successful completion"):
            visualizer.parse_gencase_out(secure("case.out", changed))

    def test_existing_output_is_refused_before_source_access(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-u3-vis-") as temporary:
            output = Path(temporary) / "existing"
            output.mkdir()
            args = argparse.Namespace(output_dir=str(output))
            with self.assertRaisesRegex(FileExistsError, "refusing to overwrite"):
                visualizer.run(args)

    def test_products_are_hardened_before_manifest_metadata_capture(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-u3-vis-mode-") as temporary:
            output = Path(temporary) / "candidate"
            output.mkdir(mode=0o700)
            for name in ("data", "figures", "reports"):
                (output / name).mkdir(mode=0o700)
            products = [output / "README.md", output / "figures" / "figure.png"]
            for product in products:
                product.write_bytes(b"derived\n")
                os.chmod(product, 0o664)

            visualizer._harden_products(output, products)

            self.assertTrue(all((path.stat().st_mode & 0o777) == 0o640 for path in products))
            self.assertEqual(output.stat().st_mode & 0o777, 0o750)
            self.assertTrue(
                all(
                    (output / name).stat().st_mode & 0o777 == 0o750
                    for name in ("data", "figures", "reports")
                )
            )

    def test_qa_paths_are_package_relative_before_atomic_publication(self) -> None:
        root = Path("/tmp/example.partial.123")
        path = root / "figures" / "figure.png"
        portable = visualizer._portable_figure_info(
            {"path": str(path), "size_bytes": 12}, path, root
        )
        self.assertEqual(portable["path"], "figures/figure.png")
        self.assertEqual(portable["size_bytes"], 12)


if __name__ == "__main__":
    unittest.main()
