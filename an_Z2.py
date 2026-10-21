import json, collections, numpy as np
from failfn import failed
def L(n):
    try: return [json.loads(l) for l in open(f'results2/camp_{n}.jsonl')]
    except FileNotFoundError: return []
def cell(rows):
    f=[failed(r,r['key'].split('|')[-2]) for r in rows]; ok=[r['rmse'] for r,x in zip(rows,f) if not x]
    return (np.mean(ok)*1 if ok else float('nan'), sum(f), len(rows), np.median(ok) if ok else float('nan'))
def merged(names):
    g=collections.defaultdict(dict)
    for n in names:
        for r in L(n):
            k=r['key'].split('|'); g['|'.join(k[:-1])][int(k[-1])]=r
    return g
g=merged(['G','H','K','X','Z2'])
print('--- Table 7 cells (mean RMSE mm, fail/n)')
for k in sorted(g):
    if k.startswith(('H|','K|')) or 'ts=0.8' in k and 'b1=0.5' in k:
        m,nf,n,_=cell(list(g[k].values())); print(f'{k:34s} {m:7.3f} ({nf}/{n})')
print('--- Delay (20 seeds)')
gd=merged(['J','X','Z3'])
for k in sorted(gd):
    rows=list(gd[k].values()); m,nf,n,med=cell(rows); print(f'{k:30s} n={n} fail={nf} med_rmse={med:.3f}')
