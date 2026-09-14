"""Numerical tests against the actual generated libraries; no real bags/ROS master."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "analysis"))
import analyze_exact_ocp_cost as cost


class ExactCostTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        cls.bundle = cls.root / "bundle"
        cls.manifest = cost.freeze_bundle(cls.bundle)
        cost.load_bundle(cls.bundle)
        cls.evaluators = {name: cost.CostEvaluator(cls.bundle, cls.manifest, name)
                          for name in cls.manifest["models"]}

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def parameters(self, evaluator):
        p = np.zeros(evaluator.meta["np"])
        for key, value in {"rx1": 1, "e_c_ref": .2, "e_l_ref": .1, "v_ref": .2,
                           "eta_ref": .001, "eta_dot_ref": .03}.items():
            if key in evaluator.index:
                p[evaluator.index[key]] = value
        for index in evaluator.weight_indices:
            p[index] = .3
        return p

    def pair(self, condition="full", cycle=1, stamp=10.):
        evaluator = self.evaluators["spmpc_slosh" if condition == "full" else "spmpc_b0"]
        meta = evaluator.meta
        p = np.tile(self.parameters(evaluator), (61, 1))
        p[1:, evaluator.index["w_du_vs"]] = 0
        common = {"valid": True, "schema_version": 5, "cycle_id": cycle,
                  "control_semantics": "a_cmd_alpha_cmd", "horizon_steps": 60,
                  "slosh_enabled": condition == "full", "dt": 1/30,
                  "solver_input_epoch": stamp - .01,
                  "solve_end_stamp": stamp - .001,
                  "backend": "continuous_mpcc_acados_explicit_actuator",
                  "variant": "B_slosh", "solver_status": "B_slosh_ACADOS_OK"}
        snapshot = dict(common, state_width=meta["nx"], control_width=3,
                        parameter_width=meta["np"], parameter_names=meta["parameter_names"],
                        stage_parameters=p.reshape(-1).tolist())
        horizon = dict(common)
        values = {"x": .32, "y": .015, "yaw": 0, "v": .1, "s": .3, "omega": .1,
                  "v_cmd": .1, "omega_cmd": .1, "a_cmd_memory": .02,
                  "eta_x": .0002, "eta_x_dot": .002, "eta_y": -.0001, "eta_y_dot": -.001}
        for field, value in values.items():
            horizon[field] = [value] * 61
        for key, value in (("a", .15), ("alpha_or_omega", .1), ("v_s", .18)):
            horizon[key] = [value] * 60
        return snapshot, horizon

    def topics(self, condition="full", start=10.):
        audits, snapshots, horizons = [], [], []
        for cycle in (1, 2):
            stamp = start + (cycle - 1) / 30
            snapshot, horizon = self.pair(condition, cycle, stamp)
            snapshots.append(snapshot)
            horizons.append(horizon)
            audits.append({"cycle_id": cycle, "solve_attempted": True, "solve_success": True,
                           "command_was_published": True, "command_accepted": True,
                           "command_publish_stamp": stamp, "published_cmd_v": .2,
                           "published_cmd_omega": 0, "status": "B_slosh_ACADOS_OK",
                           "variant": "B_slosh", "solver_status": "B_slosh_ACADOS_OK",
                           "solve_end_stamp": stamp - .001,
                           "solver_input_epoch": stamp - .01})
        audits.append({"cycle_id": 3, "solve_attempted": False, "solve_success": False,
                       "command_was_published": True, "command_publish_stamp": start + 1,
                       "published_cmd_v": 0, "published_cmd_omega": 0, "status": "GOAL_REACHED"})
        return {"audit": audits, "snapshot": snapshots, "horizon": horizons}

    def test_random_weight_isolation_matches_all_generated_functions(self):
        rng = np.random.RandomState(210)
        for evaluator in self.evaluators.values():
            for k in (0, 17, 60):
                for _ in range(6):
                    x = rng.normal(size=evaluator.meta["nx"]) * .03
                    x[3], x[4] = .05, .3
                    u = np.array([.2, -.1, .12])
                    p = self.parameters(evaluator)
                    p[evaluator.index["ry2"]] = .8  # curved reference and low-speed deficits
                    p[evaluator.weight_indices] = rng.uniform(.1, 1, len(evaluator.weight_indices))
                    original_x, original_p = x.copy(), p.copy()
                    parts, total, error = evaluator.stage(x, u, p, k)
                    self.assertAlmostEqual(sum(parts.values()), total, delta=1e-9 * max(1, abs(total)))
                    self.assertLess(error, 1e-8)
                    np.testing.assert_array_equal(x, original_x)
                    np.testing.assert_array_equal(p, original_p)

    def test_liquid_normalization_and_terminal_scaling(self):
        evaluator = self.evaluators["spmpc_slosh"]
        p = self.parameters(evaluator)
        p[evaluator.weight_indices] = 0
        p[evaluator.index["w_slosh_eta"]] = 1
        x = np.zeros(28); x[24], x[26] = .0002, -.0001
        stage, _, _ = evaluator.stage(x, np.zeros(3), p, 1)
        terminal, _, _ = evaluator.stage(x, np.zeros(0), p, 60)
        raw = (.0002**2 + .0001**2) / .001**2
        self.assertAlmostEqual(stage["J_slosh_eta"], raw / 60 * evaluator.meta["cost_scaling"][1])
        self.assertAlmostEqual(terminal["J_slosh_eta"], raw)
        self.assertEqual(terminal["J_control"], 0)

    def test_anticreep_and_negative_progress_are_preserved(self):
        evaluator = self.evaluators["spmpc_b0"]
        p = self.parameters(evaluator); p[evaluator.weight_indices] = 0
        p[evaluator.index["w_v"]] = 1
        p[evaluator.index["w_progress"]] = .2
        parts, _, _ = evaluator.stage(np.zeros(24), np.array([0, 0, .1]), p, 0)
        expected = ((.2/.8)**2 + 8*((.2/.8)**2 + (.1/.8)**2)) / 60 * evaluator.meta["cost_scaling"][0]
        self.assertAlmostEqual(parts["J_v_with_anticreep"], expected)
        self.assertLess(parts["J_progress"], 0)

    def test_horizon_uses_each_parameter_row_and_counts_terminal_once(self):
        snapshot, horizon = self.pair()
        evaluator = self.evaluators["spmpc_slosh"]
        original = cost.evaluate_cycle(snapshot, horizon, evaluator)
        params = np.array(snapshot["stage_parameters"]).reshape(61, -1)
        params[60, evaluator.index["w_slosh_eta"]] *= .5
        snapshot["stage_parameters"] = params.reshape(-1).tolist()
        changed = cost.evaluate_cycle(snapshot, horizon, evaluator)
        self.assertEqual(original["stage_parts"], changed["stage_parts"])
        self.assertAlmostEqual(changed["terminal_parts"]["J_slosh_eta"],
                               .5*original["terminal_parts"]["J_slosh_eta"])
        self.assertAlmostEqual(changed["total"], changed["stage_total"] + changed["terminal_total"])

    def test_smooth_has_zero_liquid_but_nonzero_other_cost(self):
        snapshot, horizon = self.pair("smooth")
        result = cost.evaluate_cycle(snapshot, horizon, self.evaluators["spmpc_b0"])
        self.assertEqual(result["parts"]["J_slosh_eta"], 0)
        self.assertEqual(result["parts"]["J_slosh_eta_dot"], 0)
        self.assertGreater(result["abs_parts_sum"], 0)

    def test_bad_dimensions_missing_fields_nan_and_layout_rejected(self):
        for key, value in (("state_width", 24), ("parameter_names", []),
                           ("stage_parameters", [float("nan")]*2257), ("dt", .05)):
            s, h = self.pair(); s[key] = value
            with self.assertRaises(ValueError):
                cost.evaluate_cycle(s, h, self.evaluators["spmpc_slosh"])
        s, h = self.pair(); del h["a_cmd_memory"]
        with self.assertRaises(ValueError):
            cost.evaluate_cycle(s, h, self.evaluators["spmpc_slosh"])

    def test_no_solve_terminal_excluded_and_missing_failed_cycles_retained(self):
        topics = self.topics()
        result = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertEqual((result["status"], result["evaluated_cycles"], result["no_solve_cycles_excluded"]),
                         ("PASS", 2, 1))
        topics["audit"][1]["solve_success"] = False
        result = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("solve failed", result["rejected_cycles"][0]["reason"])
        topics = self.topics(); topics["horizon"].pop()
        result = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["evaluated_cycles"], 1)

    def test_duplicates_epochs_and_interventions_are_visible(self):
        topics = self.topics(); topics["snapshot"].append(topics["snapshot"][0])
        self.assertEqual(cost.analyze_topics(topics, self.evaluators, "full")["status"], "FAIL")
        topics = self.topics(); topics["audit"][0]["solver_input_epoch"] += 1
        self.assertEqual(cost.analyze_topics(topics, self.evaluators, "full")["status"], "FAIL")
        topics = self.topics(); topics["audit"][0]["safety_gate_intervened"] = True
        report = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertTrue(report["cycles"][0]["safety_gate_intervened"])

    def test_unpublished_solve_uses_explicit_solve_time_and_is_not_lost(self):
        topics = self.topics()
        topics["audit"][1].update(command_publish_stamp=0, command_was_published=False,
                                  command_accepted=False, command_contract_violation=True)
        report = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertEqual(report["evaluated_cycles"], 2)
        self.assertEqual(report["cycles"][1]["timestamp_source"], "solve_end_stamp")
        self.assertTrue(report["cycles"][1]["command_contract_violation"])
        self.assertFalse(report["cycles"][1]["command_was_published"])
        topics["audit"][1]["solve_end_stamp"] = 0
        report = cost.analyze_topics(topics, self.evaluators, "full")
        self.assertEqual(report["status"], "FAIL")

    def test_wrong_backend_and_solver_status_rejected(self):
        for row_type, key, value in (("snapshot", "backend", "legacy"),
                                     ("horizon", "solver_status", "ACADOS_SOLVE_FAILED_4"),
                                     ("audit", "solver_status", "JERK_CONSTRAINT_VIOLATION")):
            topics = self.topics(); topics[row_type][0][key] = value
            self.assertEqual(cost.analyze_topics(topics, self.evaluators, "full")["status"], "FAIL")

    def test_wrong_recording_chain_rejected_before_bag_read(self):
        bag = self.root / "wrong_chain.bag"
        bag.write_text("not a bag; it must never be opened")
        bag.with_name(bag.stem + "_runtime_smoke_prereg.env").write_text(
            "protocol=SMPCC_C03_INTERNAL_SLOSH_DEV_V2\ncondition=full\nevaluation_chain_sha256=wrong\n")
        out = self.root / "wrong_chain_output"
        with mock.patch.object(cost, "load_inputs", side_effect=AssertionError("must not read bag")):
            self.assertEqual(cost.main(["analyze", "--bag", str(bag), "--bundle", str(self.bundle),
                                        "--output-dir", str(out)]), 1)
        self.assertIn("recorded chain differs", json.loads((out / "report.json").read_text())["error"])

    def test_v3_prereg_checks_actual_ocp_weights_and_jerk(self):
        expected = dict(w_v=.5, w_contour=.7, w_lag=.15, jerk_max=1.)
        for condition in ("full", "smooth"):
            topics = self.topics(condition=condition)
            for s in topics['snapshot']:
                p = np.array(s['stage_parameters']).reshape(61, s['parameter_width'])
                for key in ('w_v', 'w_contour', 'w_lag'):
                    p[:, s['parameter_names'].index(key)] = expected[key]
                s['stage_parameters'] = p.ravel().tolist()
            for states in (topics['snapshot'], topics['horizon']):
                for s in states:
                    s.update(jerk_limit_enable=True, jerk_max=1.)
            self.assertEqual(cost.analyze_topics(topics, self.evaluators, condition, expected)['status'], 'PASS')
            for key in expected:
                wrong = dict(expected, **{key: expected[key]*.5})
                report = cost.analyze_topics(topics, self.evaluators, condition, wrong)
                self.assertEqual(report['status'], 'FAIL', key)
                self.assertEqual(len(report['rejected_cycles']), 2)

    def test_bundle_overwrite_tampering_and_scaling_rejected(self):
        with self.assertRaises(ValueError):
            cost.freeze_bundle(self.bundle)
        path = self.bundle / "manifest.json"
        original = path.read_text()
        try:
            tampered = copy.deepcopy(self.manifest)
            tampered["models"]["spmpc_slosh"]["cost_scaling"][60] = 1/60
            path.write_text(json.dumps(tampered))
            with self.assertRaisesRegex(ValueError, "scaling"):
                cost.load_bundle(self.bundle)
        finally:
            path.write_text(original)

    def test_synthetic_ros_bag_cli_cache_and_plot(self):
        import genpy
        import rosbag
        from spmpc_local_planner.msg import ControlCycleAudit, PreSolveSnapshot, PredictedHorizon
        bag = self.root / "synthetic.bag"
        topics = self.topics(start=time.time()+2)
        for key in ('snapshot', 'horizon'):
            for row in topics[key]:
                row.update(jerk_limit_enable=True, jerk_max=.6)
        types = {"audit": ControlCycleAudit, "snapshot": PreSolveSnapshot, "horizon": PredictedHorizon}
        with rosbag.Bag(str(bag), "w") as stream:
            for topic, key in cost.TOPICS.items():
                for row in topics[key]:
                    msg = types[key]()
                    for name, value in row.items():
                        if isinstance(getattr(msg, name), genpy.Time):
                            value = genpy.Time.from_sec(value)
                        setattr(msg, name, value)
                    stream.write(topic, msg, genpy.Time.from_sec(time.time()))
        prereg = bag.with_name(bag.stem + "_runtime_smoke_prereg.env")
        prereg.write_text("protocol=SMPCC_C03_INTERNAL_SLOSH_DEV_V2\ncondition=full\n"
                           "git_revision=SYNTHETIC_TEST_ONLY\nevaluation_chain_sha256=" +
                           self.manifest["evaluation_chain_sha256"] + "\n")
        output = self.root / "synthetic_output"
        self.assertEqual(cost.main(["analyze", "--bag", str(bag), "--bundle", str(self.bundle),
                                    "--output-dir", str(output)]), 0)
        report = json.loads((output / "report.json").read_text())
        self.assertEqual(report["evaluated_cycles"], 2)
        self.assertFalse(report["cache_hit"])
        self.assertTrue((output / "cycles.csv").is_file())
        self.assertTrue((output / "cost_components.png").is_file())
        with mock.patch("rosbag.Bag", side_effect=AssertionError("cache should prevent reread")):
            cached, hit = cost.load_inputs(bag, Path(report["cache"]))
        self.assertTrue(hit)
        self.assertEqual(len(cached["horizon"]), 2)
        original = (output / "report.json").read_bytes()
        self.assertEqual(cost.main(["analyze", "--bag", str(bag), "--bundle", str(self.bundle),
                                    "--output-dir", str(output)]), 1)
        self.assertEqual((output / "report.json").read_bytes(), original)
        # The same synthetic recording exercises the V3 CLI contract through
        # its cache; no physical bag is touched or re-read.
        prereg.write_text(prereg.read_text().replace('_DEV_V2', '_DEV_V3') +
                          'w_v=0.3\nw_contour=0.3\nw_lag=0.3\njerk_max=0.6\n'
                          'evaluation_primary_monitor=imu\n')
        v3_output = self.root / 'synthetic_v3_output'
        with mock.patch('rosbag.Bag', side_effect=AssertionError('must use cache')):
            self.assertEqual(cost.main(['analyze', '--bag', str(bag), '--bundle', str(self.bundle),
                                        '--output-dir', str(v3_output), '--no-plot']), 0)
        self.assertTrue(json.loads((v3_output / 'report.json').read_text())['cache_hit'])


if __name__ == "__main__":
    unittest.main()
