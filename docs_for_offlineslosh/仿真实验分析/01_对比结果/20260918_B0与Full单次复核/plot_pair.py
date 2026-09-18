#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data/a/scout_sim_replacement/results/20260918_b0_full_check_153651')
report=json.loads((ROOT/'comparison.json').read_text())
fig,axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
for trial,color,label in zip(report['trials'],['#2878B5','#D45735'],['B0','Full']):
    data=np.load(ROOT/trial['name']/'series.npz')
    height,velocity,raw=data['height'],data['velocity'],data['raw']
    stop=trial['window_evaluation']['t_stop_candidate_sec']
    axes[0].plot(height[:,0],height[:,1],color=color,label=label,linewidth=1.2)
    axes[1].plot(velocity[:,0],velocity[:,1],color=color,label=label,linewidth=1.2)
    axes[2].plot(raw[:,1],raw[:,2],color=color,label=label,linewidth=1.4)
    for ax in axes[:2]:
        ax.axvline(stop,color=color,linestyle='--',alpha=.65,linewidth=.9)
route=np.load(ROOT/report['trials'][0]['name']/'series.npz')['path']
axes[2].plot(route[:,0],route[:,1],color='#555555',linestyle=':',label='Original route',linewidth=1.)
axes[0].set_title('One frozen B0 / Full simulation pair (development evidence)')
axes[0].set_ylabel('Model liquid height (mm)')
axes[1].set_ylabel('Actual speed (m/s)')
axes[1].set_xlabel('Task time (s); dashed lines = inferred sustained stop')
axes[2].set_xlabel('x (m)');axes[2].set_ylabel('y (m)');axes[2].set_aspect('equal',adjustable='datalim')
for ax in axes[:2]: ax.set_xlim(0,51)
for ax in axes: ax.grid(alpha=.2);ax.legend(loc='best')
fig.savefig(Path(__file__).with_name('comparison.png'),dpi=160)
plt.close(fig)
