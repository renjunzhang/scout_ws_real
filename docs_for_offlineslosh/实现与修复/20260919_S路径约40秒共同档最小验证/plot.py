#!/usr/bin/env python3
"""Plot the archived single-trial comparison; heights are model output only."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--results',type=Path,default=Path('/data/a/scout_sim_replacement/results/20260919_full_s_slow_202821'));p.add_argument('--output',type=Path,default=Path(__file__).with_name('comparison.png'));a=p.parse_args()
r=json.loads((a.results/'final_report.json').read_text());fp=Path(r['protocol']['fp_as_root'])
fig,axes=plt.subplots(3,1,figsize=(10,8),constrained_layout=True)
for label,root,case,color,key in [('Full slow',a.results,'full_01','#0072B2','Full_slow'),('FP-AS',fp,'fp_as_01','#D55E00','FP_AS_40s')]:
 d=np.load(root/case/'series.npz');m=r['metrics'][key];stop=m['completion_sec'];raw=d['raw'];v=d['velocity'];h=d['height']
 axes[0].plot(raw[:,1],raw[:,2],color=color,label=label)
 axes[1].plot(v[:,0],v[:,1],color=color,label=f'{label}: complete {stop:.3f} s')
 axes[2].plot(h[:,0],h[:,1]*1000,color=color,label=label)
 for ax in axes[1:]:ax.axvline(stop,color=color,ls=':',lw=.8)
axes[0].set(xlabel='x (m)',ylabel='y (m)',title='Different executed paths; one trial per method');axes[0].set_aspect('equal',adjustable='datalim')
axes[1].set(xlabel='Task time (s)',ylabel='Odometry speed (m/s)')
axes[2].set(xlabel='Task time (s)',ylabel='Model modal height (mm)',title='Odometry-driven liquid model; no independent physical liquid measurement')
for ax in axes:
 ax.grid(alpha=.2);ax.legend(fontsize=8)
for ax in axes[1:]:ax.set_xlim(0,max(m['completion_sec'] for m in r['metrics'].values())+5)
fig.savefig(a.output,dpi=160)
