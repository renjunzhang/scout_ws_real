#!/usr/bin/env python3
"""Replay frozen full-state OCP snapshots at anti-creep gains 8 and 0.

No robot I/O. Input: {snapshots: [{name, slosh_enabled, x0, stage_parameters,
height_coeff, jerk_delta_a_max}]}. Parameters must match the generated solver.
Results are offline fixed-snapshot optimizations, not closed-loop performance.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import casadi as ca

P=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(P/'scripts/acados'))
from generate_spmpc_acados import load_config
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols, PIDX, PIDX_SLOSH
from spmpc_acados_cost import cost_components, _reference_terms
from model_contract import COST_VERSION


def audit(request, iterations=30):
    from acados_template import AcadosOcpSolver
    cfg=load_config(); result=[]
    for case in request['snapshots']:
        liquid=case['slosh_enabled']; name='spmpc_slosh' if liquid else 'spmpc_b0'
        sym=(export_spmpc_slosh_symbols if liquid else export_spmpc_b0_symbols)()
        idx=PIDX_SLOSH if liquid else PIDX
        functions=[ca.Function('parts'+str(k),[sym['x'],sym['u'],sym['p']],
                    [cost_components(sym,cfg,terminal=bool(k))]) for k in [0,1]]
        rx,ry,phi=_reference_terms(sym['x'],sym['p'],idx)
        dx,dy=sym['x'][0]-rx,sym['x'][1]-ry
        tracking=ca.Function('tracking',[sym['x'],sym['p']],
            [ca.vertcat(ca.sin(phi)*dx-ca.cos(phi)*dy, -ca.cos(phi)*dx-ca.sin(phi)*dy)])
        for gain in [8.,0.]:
            solver=AcadosOcpSolver(None,json_file=str(P/'generated/acados'/name/('acados_ocp_'+name+'.json')),build=False,generate=False)
            x0=np.array(case['x0'],dtype=float); parameters=np.array(case['stage_parameters'],dtype=float)
            n=cfg['N']
            if x0.shape!=(sym['nx'],) or parameters.shape!=(n+1,sym['np']):
                raise ValueError('Snapshot state/parameter ABI mismatch')
            parameters[:,idx['anticreep_gain']]=gain
            for k in range(n+1):
                solver.set(k,'p',parameters[k]); solver.set(k,'x',x0)
                if k<n:
                    solver.set(k,'u',np.zeros(3))
                    if 'jerk_delta_a_max' in case:
                        bound=case['jerk_delta_a_max']
                        solver.constraints_set(k,'lg',np.array([-bound])); solver.constraints_set(k,'ug',np.array([bound]))
            solver.constraints_set(0,'lbx',x0); solver.constraints_set(0,'ubx',x0)
            times=[]; statuses=[]
            for _ in range(iterations):
                statuses.append(int(solver.solve())); times.append(float(solver.get_stats('time_tot'))*1000)
                if statuses[-1]: break
            xs=np.array([solver.get(k,'x') for k in range(n+1)])
            us=np.array([solver.get(k,'u') for k in range(n)])
            terms=np.sum([np.asarray(functions[0](xs[k],us[k],parameters[k])).ravel() for k in range(n)],axis=0)
            terminal=float(np.asarray(functions[1](xs[n],np.zeros(3),parameters[n])).sum())
            total=float(solver.get_cost()); reconstructed=float(terms.sum()+terminal)
            errors=np.array([np.asarray(tracking(xs[k],parameters[k])).ravel() for k in range(n+1)])
            result.append(dict(name=case['name'],anticreep_gain=gain,statuses=statuses,
                solver_total=total,reconstructed=reconstructed,error=reconstructed-total,
                mean_actual_v=float(xs[:,3].mean()),minimum_future_v=float(xs[5:,3].min()),
                v_at_1s=float(xs[30,3]),first_a_cmd=float(us[0,0]),
                progress=float(xs[-1,4]-xs[0,4]),contour_rms_m=float(np.sqrt(np.mean(errors[:,0]**2))),
                lag_rms_m=float(np.sqrt(np.mean(errors[:,1]**2))),
                peak_modal_height_m=float(np.hypot(xs[:,24],xs[:,26]).max()*case['height_coeff']) if liquid else None,
                anti_creep_cost=float(terms[5]),terminal_cost=terminal,
                solve_median_ms=float(np.median(times)),solve_p95_ms=float(np.percentile(times,95))))
    return dict(cost_version=COST_VERSION,evidence='OFFLINE_FIXED_SNAPSHOT',results=result)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('input',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args(); result=audit(json.loads(args.input.read_text()));args.output.write_text(json.dumps(result,indent=2)+'\n')
