#!/usr/bin/env python3
"""Display only fault-free runs from the fixed six-run batch; retain provenance."""
from pathlib import Path
import json,hashlib,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
SOURCE=Path('/data/a/scout_sim_replacement/results/20260918_b0_full_repeat3')
source=SOURCE/'comparison.json';data=json.loads(source.read_text())
normal={'INITIALIZED','WAITING_FOR_ODOM','WAITING_FOR_REFERENCE_PATH','B0_ACADOS_OK','B_slosh_ACADOS_OK','TERMINAL_DRAINING','GOAL_REACHED'}
selected=[];excluded=[]
for row in data['reports']:
 abnormal={k:v for k,v in row['status_counts'].items() if k not in normal and v}
 if row['passed'] and not abnormal:selected.append(row)
 else:excluded.append(dict(name=row['name'],passed=row['passed'],abnormal_status_counts=abnormal))
assert sorted(r['name'] for r in selected)==['b0_01','b0_02','b0_03','full_02']
keys=['peak_height_mm','active_height_p95_mm','active_height_rms_mm','goal_sec']
summary={g:dict(n=sum(r['group']==g for r in selected),**{k:float(np.mean([r[k] for r in selected if r['group']==g])) for k in keys}) for g in ['b0','full']}
changes={k:100*(summary['full'][k]/summary['b0'][k]-1) for k in keys}
report=dict(formal=False,scope='Post-hoc fault-free subset of six recorded runs; Full n=1, B0 n=3',
 source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_code_commit=data['git_sha'],
 criteria=dict(require_within_45_seconds=True,allowed_statuses=sorted(normal),exclude_any_other_status=True),
 evaluation=data['evaluation'],selected=[{k:r[k] for k in ['name','group','pair','passed',*keys]} for r in selected],
 excluded=excluded,summary=summary,full_vs_b0_mean_change_percent=changes,
 full_fault_free_fraction='1/3',raw_records_preserved=True)
(OUT/'selected_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
with (OUT/'selected_results.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=['name','group','pair','passed',*keys],lineterminator='\n');writer.writeheader();writer.writerows(report['selected'])
fig,axes=plt.subplots(1,2,figsize=(11,3.9),constrained_layout=True)
for r in sorted(selected,key=lambda x:(x['group'],x['pair'])):
 a=np.load(SOURCE/r['name']/'series.npz');b0=r['group']=='b0';color='#D17B13' if b0 else '#166BB0'
 style={1:'-',2:'--',3:':'}[r['pair']] if b0 else '-'
 label=f"B0 R{r['pair']}" if b0 else 'Full R2 (only retained Full run)'
 for ax,key,column in [(axes[0],'height',1),(axes[1],'velocity',1)]:
  v=a[key];v=v[(v[:,0]>=0)&(v[:,0]<=r['peak_evaluation_end_sec'])]
  ax.plot(v[:,0],v[:,column],linestyle=style,color=color,lw=1.5 if b0 else 2.,label=label,alpha=.85)
axes[0].set(xlabel='Task time [s]',ylabel='Odom-model liquid height [mm]',title='Fault-free subset: B0 n=3, Full n=1')
axes[1].set(xlabel='Task time [s]',ylabel='Actual forward speed [m/s]',title='Different speed profiles and arrival times')
axes[0].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(OUT/'selected_comparison.png',dpi=160)
print(json.dumps(dict(summary=summary,changes_percent=changes,selected=[r['name'] for r in selected]),indent=2))
