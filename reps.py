import numpy as np, os, sim2, ff_campaigns as F
from multiprocessing import Pool
sel=F.sel_load(); os.makedirs('reps',exist_ok=True)
def go(a):
    name,scen,seed,kw=a
    o=sim2.run(scenario=scen,seed=seed,**kw)
    keep={k:o[k] for k in ('t','q','qr','x','xd','tau','r','s','sat','Ie') if k in o}
    keep['det']=np.array([o['detected_at'] if o['detected_at'] is not None else np.nan]); keep['diverged']=np.array([o.get('diverged',False)])
    np.savez_compressed(f'reps/{name}.npz',**keep); return name
jobs=[]
for c in ('CT-PID','ISMC-A','ISMC-M','ISMC-B','ISMC-J','FOSMC','FOSMC-J'):
    jobs.append((f'S3_{c}_100','S3',100,F.ck(c,sel)))
for c in ('ISMC-J','FOSMC-J','FOSMC'):
    jobs.append((f'S3_{c}_112','S3',112,F.ck(c,sel)))
jobs.append(('S3_FOSMC-J-cond_112','S3',112,dict(F.ck('FOSMC-J',sel),antiwindup='cond')))
jobs.append(('S3_ISMC-J-cond_112','S3',112,dict(F.ck('ISMC-J',sel),antiwindup='cond')))
for s in range(100,110):
    jobs.append((f'AB_norec_{s}','S3',s,dict(F.ck('FOSMC',sel),reconfigure=False)))
    jobs.append((f'AB_weak_{s}','S3',s,dict(F.ck('FOSMC',sel),kn=4.0,W=(4.0,1.0,1.0))))
with Pool(2) as p:
    for n in p.imap_unordered(go,jobs): print(n,flush=True)
