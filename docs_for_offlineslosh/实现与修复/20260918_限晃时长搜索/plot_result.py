from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data/a/scout_sim_replacement/results/20260918_liquid_time_resume_161636')
OLD=Path('/data/a/scout_sim_replacement/results/20260918_b0_full_check_153651')
plan=json.loads((ROOT/'time_search/selected_plan.json').read_text())
t=np.array([a['t'] for a in plan['samples']]);x=np.array([a['state'] for a in plan['samples']])
fig,axes=plt.subplots(3,1,figsize=(10,9),constrained_layout=True)
for path,label,color in [(OLD/'b0_01','B0 (previous run)','#3978ad'),(OLD/'full_01','Full (old seed)','#84904a'),(ROOT/'full_selected','Full (new plan)','#ce4a35')]:
 data=np.load(path/'series.npz');height=data['height'];velocity=data['velocity'];raw=data['raw']
 axes[0].plot(height[:,0],height[:,1],color=color,label=label,lw=1.1)
 if path.parent==ROOT:
  axes[1].plot(velocity[:,0],velocity[:,1],color=color,label='New Full actual',lw=1.2)
  axes[2].plot(raw[:,1],raw[:,2],color=color,label='New Full actual',lw=1.2)
axes[0].plot(t,plan['height_coeff']*np.hypot(x[:,24],x[:,26])*1000,'--',color='#553b92',label='Selected plan prediction',lw=1.1)
axes[0].axhline(.9,color='#444444',ls=':',lw=1,label='Transport cap 0.90 mm')
axes[0].set_ylabel('Model liquid height (mm)')
axes[0].set_title('New Full is faster, but exceeds the frozen transport liquid cap')
axes[1].plot(t,x[:,3],'--',color='#553b92',label='Nominal speed versus plan time',lw=1.2)
deviation=np.load(ROOT/'execution_deviation_series.npz')
axes[1].plot(deviation['task_time'],deviation['nominal_speed'],color='#9a76c0',alpha=.7,label='Nominal speed at actual progress',lw=1.)
axes[1].axvline(13.126,color='#555555',ls=':',lw=.8)
axes[1].set_ylabel('Speed (m/s)');axes[1].set_xlabel('Time (s)')
route=np.array(plan['route'])
axes[2].plot(route[:,0],route[:,1],':',color='#666666',label='Original route',lw=1)
axes[2].plot(x[:,0],x[:,1],'--',color='#553b92',label='Selected plan geometry',lw=1.2)
axes[2].set_xlabel('x (m)');axes[2].set_ylabel('y (m)');axes[2].set_aspect('equal',adjustable='datalim')
for ax in axes[:2]:ax.set_xlim(0,46)
for ax in axes:ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
fig.savefig(Path(__file__).with_name('comparison.png'),dpi=150)
