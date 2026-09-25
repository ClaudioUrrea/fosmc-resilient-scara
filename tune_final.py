import json, itertools, numpy as np, sim, os
os.makedirs('results', exist_ok=True)
rows=[]
for lam,L2 in itertools.product([0.3,0.5,0.7,1.0],[200.,400.,800.,1600.,3200.]):
    ms=[sim.metrics(sim.run('ISMC' if lam==1 else 'FOSMC',lam,'S2',seed=s,L2=L2)) for s in range(4)]
    rows.append(dict(kind='SMC',lam=lam,L2=L2,rmse=float(np.mean([m['rmse'] for m in ms])),emax=float(np.mean([m['emax'] for m in ms])),tv=float(np.mean([m['tv'].sum() for m in ms]))))
    print(rows[-1],flush=True)
for om in [20.,30.,40.,60.]:
    ms=[sim.metrics(sim.run('CTPID',1.0,'S2',seed=s,omega=om)) for s in range(4)]
    rows.append(dict(kind='CTPID',omega=om,rmse=float(np.mean([m['rmse'] for m in ms])),emax=float(np.mean([m['emax'] for m in ms])),tv=float(np.mean([m['tv'].sum() for m in ms]))))
    print(rows[-1],flush=True)
json.dump(rows,open('results/tuning.json','w'),indent=1)
ref=[r for r in rows if r['kind']=='SMC' and r['lam']==1.0 and r['L2']==400.][0]
budget=1.15*ref['tv']
ok=lambda r: np.isfinite(r['rmse']) and r['tv']<=budget
fo=min([r for r in rows if r['kind']=='SMC' and r['lam']<1 and ok(r)], key=lambda r:r['rmse'])
ib=min([r for r in rows if r['kind']=='SMC' and r['lam']==1 and ok(r)], key=lambda r:r['rmse'])
ct_ok=[r for r in rows if r['kind']=='CTPID' and ok(r)]
ct=min(ct_ok,key=lambda r:r['rmse']) if ct_ok else min([r for r in rows if r['kind']=='CTPID'],key=lambda r:r['tv'])
sel={'budget':budget,'FOSMC':fo,'ISMC-B':ib,'CT-PID':ct}
json.dump(sel,open('results/selection.json','w'),indent=1); print('SELECTION',sel,flush=True)
