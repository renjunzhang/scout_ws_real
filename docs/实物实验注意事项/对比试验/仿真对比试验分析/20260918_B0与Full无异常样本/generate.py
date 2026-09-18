#!/usr/bin/env python3
"""Show three fault-free Full runs, retaining the complete attempt ledger."""
from pathlib import Path
import json,hashlib,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
BASE=Path('/data/a/scout_sim_replacement/results/20260918_b0_full_repeat3')
EXTRA=Path('/data/a/scout_sim_replacement/results/20260918_full_clean3_supplement')
base_file=BASE/'comparison.json';extra_file=EXTRA/'manifest.json'
base=json.loads(base_file.read_text());extra=json.loads(extra_file.read_text())
assert extra.get('completed') and len(extra['selected'])==3
normal={'INITIALIZED','WAITING_FOR_ODOM','WAITING_FOR_REFERENCE_PATH','B0_ACADOS_OK','B_slosh_ACADOS_OK','TERMINAL_DRAINING','GOAL_REACHED'}
rows=[(r,BASE/r['name']) for r in base['reports']]+[(t['review'],EXTRA/t['name']) for t in extra['trials']]
selected=[];excluded=[]
for row,directory in rows:
 abnormal={k:v for k,v in row['status_counts'].items() if k not in normal and v}
 if row['passed'] and not abnormal:selected.append((row,directory))
 else:excluded.append(dict(name=row['name'],directory=str(directory),passed=row['passed'],abnormal_status_counts=abnormal))
assert sum(r['group']=='b0' for r,p in selected)==3
assert sum(r['group']=='full' for r,p in selected)==3
assert {str(p) for r,p in selected if r['group']=='full'}==set(extra['selected'])
keys=['peak_height_mm','active_height_p95_mm','active_height_rms_mm','goal_sec']
summary={g:dict(n=3,**{k:float(np.mean([r[k] for r,p in selected if r['group']==g])) for k in keys}) for g in ['b0','full']}
spread={g:{k:dict(std=float(np.std([r[k] for r,p in selected if r['group']==g],ddof=1)),min=min(r[k] for r,p in selected if r['group']==g),max=max(r[k] for r,p in selected if r['group']==g)) for k in keys} for g in ['b0','full']}
changes={k:100*(summary['full'][k]/summary['b0'][k]-1) for k in keys}
selected_rows=[dict(**{k:r[k] for k in ['name','group','pair','passed',*keys]},directory=str(p)) for r,p in selected]
report=dict(formal=False,scope='Fault-free subset; B0 n=3 and Full n=3. Sequential supplementary sampling stopped at first two additional clean Full runs.',
 sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [base_file,extra_file]},source_code_commit=base['git_sha'],
 criteria=dict(require_within_45_seconds=True,allowed_statuses=sorted(normal),exclude_any_other_status=True,no_height_based_selection=True),
 evaluation=base['evaluation'],selected=selected_rows,excluded=excluded,summary=summary,spread=spread,full_vs_b0_mean_change_percent=changes,
 full_fault_free_fraction=f"3/{extra['total_full_attempts']}",new_full_attempts=len(extra['trials']),raw_records_preserved=True)
(OUT/'selected_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
with (OUT/'selected_results.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=['name','group','pair','passed',*keys,'directory'],lineterminator='\n');writer.writeheader();writer.writerows(selected_rows)
fig,axes=plt.subplots(1,2,figsize=(11,3.9),constrained_layout=True)
count={'b0':0,'full':0}
for r,p in sorted(selected,key=lambda x:(x[0]['group'],x[0]['pair'])):
 a=np.load(p/'series.npz');b0=r['group']=='b0';color='#D17B13' if b0 else '#166BB0';style=['-','--',':'][count[r['group']]];count[r['group']]+=1
 label=('B0' if b0 else 'Full')+f" R{r['pair']}"
 for ax,key,column in [(axes[0],'height',1),(axes[1],'velocity',1)]:
  v=a[key];v=v[(v[:,0]>=0)&(v[:,0]<=r['peak_evaluation_end_sec'])]
  ax.plot(v[:,0],v[:,column],linestyle=style,color=color,lw=1.5,label=label,alpha=.85)
axes[0].set(xlabel='Task time [s]',ylabel='Odom-model liquid height [mm]',title=f"Clean subset: B0 n=3; Full 3/{extra['total_full_attempts']} attempts")
axes[1].set(xlabel='Task time [s]',ylabel='Actual forward speed [m/s]',title='Different speed profiles and arrival times')
axes[0].legend(fontsize=8,ncol=2)
for ax in axes:ax.grid(alpha=.25)
fig.savefig(OUT/'selected_comparison.png',dpi=160)
print(json.dumps(dict(summary=summary,spread=spread,changes_percent=changes,selected=[r['name'] for r,p in selected],total_full_attempts=extra['total_full_attempts']),indent=2))
