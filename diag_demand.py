import os
for v in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'): os.environ[v]='1'
import numpy as np, ff_campaigns as F, sim2, json
from multiprocessing import Pool
sel=F.sel_load()
CF={'nom':{}, 'P':dict(rho=(60.,25.,3.)), 'cond':dict(antiwindup='cond')}
def job(a):
    c,cf,s=a
    kw=F.ck(c,sel); kw.update(CF[cf])
    o=sim2.run(scenario='S3',seed=s,tau_scale=100.0,**kw)
    t=o['t']; d=o['detected_at'] or 4.03
    m=(t>=d)&(t<d+1.0); tau=o['tau'][m]
    return dict(c=c,cf=cf,s=s,peak=[float(np.nanmax(np.abs(tau[:,i]))) for i in range(3)],
                p95=float(np.nanpercentile(np.abs(tau[:,1]),95)),
                tsat=float(np.mean(np.abs(tau[:,1])>=60.0)), div=bool(o.get('diverged',False)))
jobs=[(c,cf,s) for c in ('ISMC-J','FOSMC-J') for cf in ('nom','P') for s in range(100,200)]
with Pool(2) as p: res=p.map(job,jobs,chunksize=4)
json.dump(res,open('diag_demand.json','w')); print('done')
