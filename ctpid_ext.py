import json, sys
import ff_campaigns as F
specs=[(f'B|CTPID|w={w}|S2|{s}','S2',dict(ctrl_kind='CTPID',omega=float(w),seed=s)) for w in (32,36,40,48,56,64,80,100) for s in F.TUNE]
F.run_jobs(specs,'stage1c')
rows=F.load('stage1a')+F.load('stage1c')
t=F.tuning_table(rows,'CTPID')
for x in sorted(t,key=lambda x:x['w']): print(x['w'],x['rmse'],x['tv'],x['diverged'])
print(F.best(t))
