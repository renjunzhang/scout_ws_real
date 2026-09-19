"""Post-actuator grid jerk: production dynamics, FIFO boundary, saved plans."""
import copy
import sys
import unittest
from pathlib import Path
import casadi as ca
import numpy as np
sys.path[:0] = [str(Path(__file__).resolve().parents[1]/'acados'), str(Path(__file__).resolve().parents[1])]
from actual_jerk import acceleration_delta_row, actual_acceleration
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols, PIDX
from generate_spmpc_acados import load_config, default_parameter_values
from planning_fixture import make_plan
from planning.validation import validate_plan
from planning.task import load_task

class ActualJerkTest(unittest.TestCase):
    def test_row_matches_full_model_including_fifo_switch(self):
        for exporter in (export_spmpc_b0_symbols,export_spmpc_slosh_symbols):
            sym=exporter(); F=ca.Function('step',[sym['x'],sym['u'],sym['p']],[sym['disc_dyn']])
            p=default_parameter_values(load_config(),with_slosh=sym['nx']==28)
            for tau,gain in ((.07,.85),(.112,1.018),(.25,1.2)):
                p[PIDX['actuator_tau_v']]=tau; p[PIDX['actuator_gain_v']]=gain
                dt=p[PIDX['actuator_dt']]
                for velocity in (.25,.37):
                    for sign in (-1,1):
                        x=np.zeros(sym['nx']); x[3]=velocity;x[8]=.25/gain;x[9]=x[8]+sign*.01
                        next_x=np.asarray(F(x,[.1,0.,0.],p)).ravel()
                        delta=np.diff(actual_acceleration(np.column_stack((x,next_x)),tau,gain))[0]
                        self.assertAlmostEqual(acceleration_delta_row(tau,gain,dt,sym['nx'])@x,delta,places=12)
                        if velocity==.25: self.assertGreater(abs(delta/dt),1.)

    def test_replayed_plan_bound_and_legacy_compatibility(self):
        plan=make_plan(); result=validate_plan(plan)
        peak=result['peak_actual_jerk_m_s3']; self.assertGreater(peak,0)
        for limit,accepted in ((0.,True),(peak*1.1,True),(peak*.5,False)):
            trial=copy.deepcopy(plan)
            trial['motion_limits']['actual_jerk_max']=limit
            trial['task']['motion_limits']['actual_jerk_max']=limit
            if accepted: validate_plan(trial)
            else:
                with self.assertRaisesRegex(ValueError,'actual_jerk'): validate_plan(trial)
        for bad in (-1.,float('nan'),float('inf')):
            task=copy.deepcopy(plan['task']);task['motion_limits']['actual_jerk_max']=bad
            with self.assertRaises(ValueError): load_task(task)

if __name__=='__main__': unittest.main()
