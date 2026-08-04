#!/usr/bin/env python3
"""Pure-Python tests for the independent development liquid plant."""

from __future__ import division

import json
import math
from pathlib import Path
import sys
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from scout_liquid_plant.core import (  # noqa: E402
    LiquidPlant,
    OdomSample,
    PlantConfigError,
    PlantParameters,
)


def valid_config():
    return {
        "schema_version": 1,
        "development_only": True,
        "formal": False,
        "fidelity_validation_status": "UNVALIDATED",
        "condition_template_id": "TEST_DEVELOPMENT_TEMPLATE_UNFROZEN",
        "odom_topic": "/odom",
        "height_topic": "/sim_truth/liquid_height",
        "state_topic": "/sim_truth/liquid_state",
        "metadata_topic": "/sim_truth/liquid_metadata",
        "integration_step_sec": 0.002,
        "max_odom_dt_sec": 0.10,
        "rotation_height_gain_s2": 0.00002,
        "max_modal_displacement_m": 0.10,
        "max_modal_velocity_mps": 3.0,
        "height_limit_m": 0.08,
        "container": {
            "container_radius_m": 0.02,
            "static_liquid_height_m": 0.055,
            "liquid_density_kgm3": 1000.0,
            "offset_x_m": 0.01,
            "offset_y_m": -0.005,
        },
        "modes": [
            {
                "mode_id": "m1",
                "natural_frequency_radps": 9.0,
                "damping_ratio": 0.08,
                "input_gain": 1.0,
                "height_gain": 0.7,
                "cubic_stiffness_m2s2": 400.0,
            },
            {
                "mode_id": "m2",
                "natural_frequency_radps": 16.0,
                "damping_ratio": 0.13,
                "input_gain": 0.35,
                "height_gain": 0.3,
                "cubic_stiffness_m2s2": 1200.0,
            },
        ],
    }


class PlantParametersTest(unittest.TestCase):
    def test_strict_development_boundaries_are_enforced(self):
        for key, value in (
            ("formal", True),
            ("development_only", False),
            ("fidelity_validation_status", "PASS"),
            ("odom_topic", "/some_other_odom"),
            ("height_topic", "/other_height"),
        ):
            config = valid_config()
            config[key] = value
            with self.subTest(key=key):
                with self.assertRaises(PlantConfigError):
                    PlantParameters.from_mapping(config)

    def test_multiple_modes_and_unique_ids_are_required(self):
        one_mode = valid_config()
        one_mode["modes"] = one_mode["modes"][:1]
        with self.assertRaises(PlantConfigError):
            PlantParameters.from_mapping(one_mode)

        duplicate = valid_config()
        duplicate["modes"][1]["mode_id"] = "m1"
        with self.assertRaises(PlantConfigError):
            PlantParameters.from_mapping(duplicate)

    def test_metadata_remains_unvalidated_and_nonformal(self):
        metadata = PlantParameters.from_mapping(valid_config()).public_metadata()
        self.assertIs(metadata["development_only"], True)
        self.assertIs(metadata["formal"], False)
        self.assertEqual(metadata["fidelity_validation_status"], "UNVALIDATED")
        self.assertIs(metadata["physical_primary_eligible"], False)
        self.assertEqual(metadata["input"]["topic"], "/odom")

    def test_both_shipped_templates_are_parseable_and_stay_development_only(self):
        for filename in (
            "C1_development_unvalidated.yaml",
            "C2_development_unvalidated.yaml",
        ):
            with self.subTest(filename=filename):
                raw = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
                parameters = PlantParameters.from_mapping(raw)
                self.assertIn("DEVELOPMENT_TEMPLATE_UNFROZEN", parameters.condition_template_id)
                self.assertGreaterEqual(len(parameters.modes), 2)

    def test_c2_development_template_changes_only_requested_95mm_diameter(self):
        """C2 is deliberately a one-variable development transfer candidate.

        This does not make either template formal.  It prevents a later
        hand-edit from silently changing liquid height, offsets, or the
        unvalidated surrogate modes while claiming a diameter-only C2 test.
        """
        c1 = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "C1_development_unvalidated.yaml").read_text()
        )
        c2 = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "C2_development_unvalidated.yaml").read_text()
        )
        self.assertFalse(c1["formal"])
        self.assertFalse(c2["formal"])
        self.assertTrue(c1["development_only"])
        self.assertTrue(c2["development_only"])
        self.assertEqual("UNVALIDATED", c1["fidelity_validation_status"])
        self.assertEqual("UNVALIDATED", c2["fidelity_validation_status"])

        c1_normalized = dict(c1)
        c2_normalized = dict(c2)
        c1_normalized.pop("condition_template_id")
        c2_normalized.pop("condition_template_id")
        c1_container = dict(c1_normalized.pop("container"))
        c2_container = dict(c2_normalized.pop("container"))
        self.assertEqual(c1_normalized, c2_normalized)
        c1_radius = c1_container.pop("container_radius_m")
        c2_radius = c2_container.pop("container_radius_m")
        self.assertEqual(c1_container, c2_container)
        self.assertEqual(0.0185, c1_radius)
        self.assertEqual(0.0475, c2_radius)
        self.assertEqual(0.095, 2.0 * c2_radius)
        self.assertEqual(0.0580, c2_container["static_liquid_height_m"])


class LiquidPlantTest(unittest.TestCase):
    def setUp(self):
        self.parameters = PlantParameters.from_mapping(valid_config())
        self.plant = LiquidPlant(self.parameters)

    @staticmethod
    def sample(stamp, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        return OdomSample(
            stamp_sec=stamp,
            yaw_rad=yaw,
            linear_x_mps=vx,
            linear_y_mps=vy,
            yaw_rate_radps=yaw_rate,
        )

    def test_first_odom_initializes_without_fabricating_acceleration(self):
        result = self.plant.step(self.sample(1.0, vx=0.3))
        self.assertTrue(result.initialized)
        self.assertFalse(result.integrated)
        self.assertEqual(result.reason, "INITIALIZED_NO_DERIVATIVE")
        self.assertEqual(result.ax_body_mps2, 0.0)
        self.assertEqual(result.liquid_height_m, 0.0)

    def test_executed_odom_motion_drives_all_modes(self):
        self.plant.step(self.sample(0.0))
        result = self.plant.step(self.sample(0.02, vx=0.2, yaw_rate=0.4))
        self.assertTrue(result.integrated)
        self.assertGreater(abs(result.ax_body_mps2), 1.0)
        self.assertGreater(result.liquid_height_m, 0.0)
        self.assertGreater(result.modal_height_m, 0.0)
        self.assertEqual(
            len(result.state_values),
            len(self.plant.state_field_names()),
        )
        self.assertIn("mode_m2_qy_m", self.plant.state_field_names())

    def test_world_to_body_acceleration_uses_observed_yaw(self):
        self.plant.step(self.sample(0.0))
        # This is a world-y velocity change after a pi/2 yaw.  Transforming it
        # back into the current body frame must recover positive body x accel.
        result = self.plant.step(self.sample(0.02, yaw=math.pi / 2.0, vx=1.0))
        self.assertAlmostEqual(result.ax_body_mps2, 50.0, places=6)
        self.assertAlmostEqual(result.ay_body_mps2, 0.0, places=6)

    def test_bad_timestamp_is_not_integrated_or_overwritten(self):
        self.plant.step(self.sample(1.0))
        invalid = self.plant.step(self.sample(0.9, vx=1.0))
        self.assertFalse(invalid.integrated)
        self.assertEqual(invalid.reason, "REJECTED_NONMONOTONIC_ODOM_TIME")
        recovered = self.plant.step(self.sample(1.02, vx=0.2))
        self.assertTrue(recovered.integrated)

    def test_large_gap_rebases_input_without_integrating_spurious_impulse(self):
        self.plant.step(self.sample(0.0))
        gap = self.plant.step(self.sample(1.0, vx=2.0))
        self.assertFalse(gap.integrated)
        self.assertEqual(gap.reason, "REJECTED_ODOM_GAP")
        recovered = self.plant.step(self.sample(1.02, vx=2.1))
        self.assertTrue(recovered.integrated)
        self.assertLess(abs(recovered.ax_body_mps2), 10.0)


class PackageContractTest(unittest.TestCase):
    def test_io_schema_and_catkin_install_contract_exist(self):
        schema = json.loads(
            (PACKAGE_ROOT / "schema" / "liquid_plant_io_schema_v1.json").read_text()
        )
        self.assertEqual(schema["status"]["fidelity_validation_status"], "UNVALIDATED")
        self.assertFalse(schema["status"]["formal"])
        self.assertEqual(schema["input"]["topic"], "/odom")
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
        self.assertIn("catkin_python_setup()", cmake)
        self.assertIn("liquid_plant_formal_evidence_intake.py", cmake)
        self.assertTrue((PACKAGE_ROOT / "launch" / "liquid_plant_development.launch").is_file())

    def test_plant_source_has_no_controller_or_shared_model_dependency(self):
        source = "\n".join(
            [
                (PACKAGE_ROOT / "src" / "scout_liquid_plant" / "core.py").read_text(),
                (PACKAGE_ROOT / "scripts" / "liquid_plant_node.py").read_text(),
            ]
        )
        self.assertNotIn("slosh_models", source)
        self.assertNotIn("spmpc_", source)
        self.assertNotIn("scout_local_planner", source)


if __name__ == "__main__":
    unittest.main()
