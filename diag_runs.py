import os
for v in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'): os.environ[v]='1'
import numpy as np, ff_campaigns as F, sim2, json
from multiprocessing import Pool
sel=F.sel_load()
CF={'nom':{}, 'cond':dict(antiwindup='cond'), 'P':dict(rho=(60.,25.,3.))}
SEEDS=[112,129,151,165,186,104,131,160]   # 104/131/160: light/medium payload references
def job(a):
    c,cf,s=a
    kw=F.ck(c,sel); kw.update(CF[cf])
    o=sim2.run(scenario='S3',seed=s,log_delta=True,**kw)
    d={k:o[k] for k in ('t','q','qr','tau','s','sat','Ie','Delta','x','xd')}
    d['div']=bool(o.get('diverged',False)); d['det']=o['detected_at'] or -1
    np.savez_compressed(f'diag/{c}_{cf}_{s}.npz',**d); return a
jobs=[(c,cf,s) for c in ('ISMC-J','FOSMC-J') for cf in CF for s in SEEDS]
with Pool(2) as p:
    for a in p.imap_unordered(job,jobs): pass
print('done')
