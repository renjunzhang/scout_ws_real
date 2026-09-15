"""Cost accounting and scaling checks, independent of ROS."""
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import casadi as ca

P = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(P/'scripts/acados'))
from generate_spmpc_acados import load_config, default_parameter_values
from generate_cost_kernel import generate
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols, PIDX, PIDX_SLOSH
from spmpc_acados_cost import cost_components, stage_cost_expr, terminal_cost_expr


class OcpCostContract(unittest.TestCase):
    def test_generated_components_are_reproducible(self):
        with tempfile.TemporaryDirectory() as directory:
            generate(directory)
            for name in ['ocp_cost_generated.c','ocp_cost_generated.h','ocp_cost_contract.h','ocp_parameter_contract.h']:
                self.assertEqual((Path(directory)/name).read_bytes(), (P/'src/core/generated'/name).read_bytes())

    def test_components_equal_objective_with_distinct_terminal_and_curvature(self):
        cfg = load_config()
        rng = np.random.default_rng(619)
        for liquid, export in [(False, export_spmpc_b0_symbols),(True, export_spmpc_slosh_symbols)]:
            s = export(); idx = PIDX_SLOSH if liquid else PIDX
            p = default_parameter_values(cfg, liquid)
            p[idx['ry2']] = .5
            for terminal in [False,True]:
                terms = cost_components(s,cfg,terminal)
                objective = terminal_cost_expr(s,cfg) if terminal else stage_cost_expr(s,cfg)
                f = ca.Function('cost',[s['x'],s['u'],s['p']],[terms,objective])
                for _ in range(10):
                    x = rng.normal(size=s['nx'])*.01; u = rng.normal(size=3)*.1
                    values,total=f(x,u,p)
                    self.assertAlmostEqual(float(np.asarray(values).sum()),float(total),places=12)

    def test_anticreep_zero_removes_only_extra_low_speed_cost(self):
        cfg=load_config(); s=export_spmpc_b0_symbols()
        f=ca.Function('parts',[s['x'],s['u'],s['p']],[cost_components(s,cfg)])
        p=default_parameter_values(cfg,False); x=np.zeros(s['nx']); u=np.array([0,0,p[PIDX['v_ref']]])
        p[PIDX['anticreep_gain']]=8; on=np.asarray(f(x,u,p)).ravel()
        p[PIDX['anticreep_gain']]=0; off=np.asarray(f(x,u,p)).ravel()
        self.assertAlmostEqual(on[5],8*on[3])
        self.assertEqual(off[5],0)
        np.testing.assert_array_equal(np.delete(on,5),np.delete(off,5))

    def test_average_running_and_terminal_each_count_once(self):
        cfg=load_config(); s=export_spmpc_b0_symbols()
        p=default_parameter_values(cfg,False)
        for name in ['w_lag','w_progress','w_v','w_vs','w_a','w_omega','w_alpha','w_du_a','w_du_vs']:
            p[PIDX[name]]=0
        p[PIDX['w_contour']]=1
        x=np.zeros(s['nx']); x[1]=p[PIDX['e_c_ref']]; u=np.zeros(3)
        f=ca.Function('both',[s['x'],s['u'],s['p']], [stage_cost_expr(s,cfg),terminal_cost_expr(s,cfg)])
        stage,terminal=map(float,f(x,u,p))
        self.assertAlmostEqual(stage*cfg['N'],1.)
        self.assertAlmostEqual(terminal,1.)
        for model in ['spmpc_b0','spmpc_slosh']:
            import json
            generated=P/'generated/acados'/model/('acados_ocp_'+model+'.json')
            if generated.exists():
                actual=json.loads(generated.read_text())['solver_options']['cost_scaling']
                np.testing.assert_array_equal(actual,np.ones(cfg['N']+1))

    def test_recovery_charges_true_excess_including_initial_and_terminal_nodes(self):
        cfg=load_config(); s=export_spmpc_slosh_symbols(); idx=PIDX_SLOSH
        p=default_parameter_values(cfg,True); x=np.zeros(s['nx']); u=np.zeros(3)
        p[idx['eta_ref']]=.01; p[idx['eta_target_sq']]=.001**2
        p[idx['slack_linear_weight']]=10; p[idx['slack_quadratic_weight']]=100
        stage=ca.Function('recovery',[s['x'],s['u'],s['p']], [cost_components(s,cfg)])
        terminal=ca.Function('recovery_end',[s['x'],s['u'],s['p']], [cost_components(s,cfg,True)])
        self.assertEqual(float(stage(x,u,p)[10]),0)
        x[s['eta_base']]=.002
        excess=(.002**2-.001**2)/.01**2
        penalty=10*excess+100*excess**2
        self.assertAlmostEqual(float(stage(x,u,p)[10])*cfg['N'],penalty)
        self.assertAlmostEqual(float(terminal(x,u,p)[10]),penalty)
        p[idx['slack_linear_weight']]=p[idx['slack_quadratic_weight']]=0
        self.assertEqual(float(stage(x,u,p)[10]),0)

    def test_true_goal_zero_reference_removes_progress_and_anticreep_pressure(self):
        cfg=load_config(); s=export_spmpc_b0_symbols(); idx=PIDX
        p=default_parameter_values(cfg,False); x=np.zeros(s['nx']); u=np.array([0,0,.2])
        p[idx['stop_goal_s']]=1.; x[4]=1.
        f=ca.Function('stop',[s['x'],s['u'],s['p']],[cost_components(s,cfg)])
        cruise=np.asarray(f(x,u,p)).ravel()
        self.assertLess(cruise[2],0); self.assertGreater(cruise[5],0)
        p[idx['stop_active']]=1
        stop=np.asarray(f(x,u,p)).ravel()
        self.assertEqual(stop[2],0); self.assertEqual(stop[3],0); self.assertEqual(stop[5],0)
        self.assertGreater(stop[4],0)  # Virtual progress is now driven to zero.
        x[8]=.1  # Delayed linear command still has a tail although body speed is zero.
        self.assertGreater(float(f(x,u,p)[11]),0)

if __name__=='__main__': unittest.main()
