import os
for v in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'): os.environ[v]='1'
import numpy as np, ff_campaigns as F, sim2, json
from multiprocessing import Pool
from failfn import failed
sel=F.sel_load()
CF={'nom':{}, 'cond':dict(antiwindup='cond'), 'P':dict(rho=(60.,25.,3.))}
SEEDS=[112,119,129,132,142,151,164,165,170,173,174,186]
def job(a):
    c,cf,s=a
    kw=F.ck(c,sel); kw.update(CF[cf])
    r=sim2.summarize(sim2.run(scenario='S3',seed=s,reset_on_reconf=False,**kw),'S3')
    return (c,cf,s,failed(r,'S3'),r['rmse'])
jobs=[(c,cf,s) for c in ('FOSMC-J','ISMC-J') for cf in CF for s in SEEDS]
with Pool(2) as p: res=p.map(job,jobs)
json.dump(res,open('diag_noreset.json','w'))
for c in ('FOSMC-J','ISMC-J'):
    for cf in CF:
        rr=[r for r in res if r[0]==c and r[1]==cf]; print(c,cf,'fail no-reset:',sum(r[3] for r in rr),'/',len(rr))
