#!/usr/bin/env python3
import importlib.util
import sys
import types
from pathlib import Path
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "smpcc_sim_controller_gate.py"
SPEC = importlib.util.spec_from_file_location("smpcc_sim_controller_gate", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def snapshot(condition="SIM_Bslosh_R1", container="C1"):
    return {
        "ros_master_uri": "http://127.0.0.1:11328",
        "use_sim_time": True,
        "environment_owner_package": gate.ENVIRONMENT_OWNER_PACKAGE,
        "condition_id": condition,
        "container_condition": container,
        "parameters": gate.required_parameter_paths(condition, container),
    }


class SimControllerGateTest(unittest.TestCase):
    def test_accepts_b0_c1(self):
        receipt = gate.validate_snapshot(snapshot("SIM_B0_R1", "C1"))
        self.assertEqual("SIM_B0_R1", receipt["condition_id"])

    def test_accepts_bslosh_c2(self):
        receipt = gate.validate_snapshot(snapshot("SIM_Bslosh_R1", "C2"))
        self.assertEqual("C2", receipt["container_condition"])

    def test_refuses_non_sim_time(self):
        value = snapshot()
        value["use_sim_time"] = False
        with self.assertRaises(gate.GateError):
            gate.validate_snapshot(value)

    def test_refuses_non_loopback_master(self):
        value = snapshot()
        value["ros_master_uri"] = "http://192.168.1.8:11311"
        with self.assertRaises(gate.GateError):
            gate.validate_snapshot(value)

    def test_refuses_missing_or_non_sim_environment_owner_marker(self):
        value = snapshot()
        value["environment_owner_package"] = "spmpc_local_planner"
        with self.assertRaises(gate.GateError):
            gate.validate_snapshot(value)

    def test_refuses_weight_drift(self):
        value = snapshot()
        value["parameters"]["variants/B_slosh/w_slosh"] = 4.0
        with self.assertRaises(gate.GateError):
            gate.validate_snapshot(value)

    def test_refuses_missing_explicit_ack(self):
        value = snapshot()
        value["parameters"]["sim_adapter/release_ack"] = False
        with self.assertRaises(gate.GateError):
            gate.validate_snapshot(value)

    def test_binds_hash_to_exact_private_node_parameter_with_readback(self):
        class FakeMaster:
            def __init__(self):
                self.parameters = {}

            def getParam(self, path):
                if path not in self.parameters:
                    raise RuntimeError("missing")
                return self.parameters[path]

            def setParam(self, path, value):
                self.parameters[path] = value

        master = FakeMaster()
        fake_rosgraph = types.SimpleNamespace(Master=lambda _caller: master)
        receipt = gate.validate_snapshot(snapshot())
        with mock.patch.dict(sys.modules, {"rosgraph": fake_rosgraph}), \
                mock.patch.object(gate.sys, "argv", ["gate", "__name:=sim_spmpc_local_planner"]):
            gate._bind_gate_hash_to_node(receipt)
        self.assertEqual(
            receipt["gate_hash"],
            master.parameters["/sim_spmpc_local_planner/sim_adapter/gate_hash"],
        )


if __name__ == "__main__":
    unittest.main()
