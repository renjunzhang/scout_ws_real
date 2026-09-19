#!/usr/bin/env python3
"""Replot the single straight comparison from its archived series and plans."""
from pathlib import Path
import argparse
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
 parser=argparse.ArgumentParser()
 parser.add_argument('--results',type=Path,default=Path('/data/a/scout_sim_replacement/results/20260919_external_baselines_170619'))
 parser.add_argument('--output',type=Path,default=Path(__file__).with_name('comparison.png'))
 args=parser.parse_args()
 fig,axes=plt.subplots(2,1,figsize=(10.8,6.7),sharex=True,constrained_layout=True)
 for label,name,color in [('B0','b0_01','#666666'),('ZVD','zvd_01','#0072B2'),('Full','full_01','#D55E00')]:
  assessment=json.loads((args.results/name/'assessment.json').read_text())
  series=np.load(args.results/name/'series.npz')
  stop=assessment['windows']['t_stop_candidate_sec']
  for ax,key,column,scale in [(axes[0],'height',1,1000),(axes[1],'velocity',1,1)]:
   data=series[key];mask=(data[:,0]>=0)&(data[:,0]<=stop+5)
   ax.plot(data[mask,0],data[mask,column]*scale,color=color,lw=1.3,label=label)
   ax.axvline(stop,color=color,lw=.8,ls=':',alpha=.65)
 plan=json.loads((args.results/'zvd_plan.json').read_text())
 axes[1].plot([r['t'] for r in plan['samples']],[r['state'][3] for r in plan['samples']],color='#56B4E9',ls='--',lw=1,label='ZVD nominal actual speed')
 axes[0].set_ylabel('Model liquid height (mm)');axes[1].set_ylabel('Odometry speed (m/s)')
 axes[1].set_xlabel('Time from common task epoch (s)')
 axes[0].set_title('5 m straight task: one trial per method, matched actuator simulation')
 for ax in axes:
  ax.grid(alpha=.2);ax.legend(loc='upper right');ax.set_xlim(0,31)
 fig.savefig(args.output,dpi=180)
 print(args.output)


if __name__=='__main__':
 main()
