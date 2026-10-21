import json,numpy as np
from failfn import failed
def L(n): return [json.loads(l) for l in open(f'results2/camp_{n}.jsonl')]
def payload(seed):
    rng=np.random.default_rng(seed); [rng.uniform(-0.15,0.15) for _ in range(5)]; rng.uniform(-0.3,0.3,3); rng.uniform(-0.3,0.3,3); return rng.uniform(0,3)
PL={s:payload(s) for s in range(100,200)}
A={};Z={};Q={}
for r in L('A'):
    k=r['key'].split('|')
    if k[2]=='S3': A[(k[1],int(k[3]))]=r
for r in L('Z1'):
    k=r['key'].split('|'); Z[(k[1],k[2][3:],int(k[4]))]=r
for r in L('Q')+L('Q2'):
    k=r['key'].split('|')
    if k[3]=='S3': Q[(k[1],int(k[4]))]=r
cfg={}
for c in ('ISMC-J','FOSMC-J'):
    cfg[f'{c} nominal']={s:(A[(c,s)] if s<120 else Z[(c,'none',s)]) for s in range(100,200)}
    cfg[f'{c} cond']={s:Z[(c,'cond',s)] for s in range(100,200)}
    cfg[f'{c} backcalc']={s:Z[(c,'backcalc',s)] for s in range(100,200)}
    cfg[f'{c} P-enl']={s:Q[(c,s)] for s in range(100,200)}
bins=[0,1,1.5,2,2.25,2.5,2.75,3.01]
print('payload bins', bins)
print('n per bin', [sum(bins[i]<=PL[s]<bins[i+1] for s in PL) for i in range(len(bins)-1)])
res={}
for name,d in cfg.items():
    f={s:failed(r,'S3') for s,r in d.items()}
    fp=[PL[s] for s in f if f[s]]
    row=[sum(f[s] for s in f if bins[i]<=PL[s]<bins[i+1]) for i in range(len(bins)-1)]
    res[name]=dict(n=sum(f.values()),min_pl=min(fp) if fp else None,bins=row)
    print(f'{name:18s} fail={sum(f.values()):3d} min payload of failures={min(fp) if fp else float("nan"):.2f} kg  by bin={row}')
print('runs with payload>=2 kg:',sum(PL[s]>=2 for s in PL), ' >=2.5:',sum(PL[s]>=2.5 for s in PL))
json.dump({'payload':PL,'res':res},open('diag_payload.json','w'))
