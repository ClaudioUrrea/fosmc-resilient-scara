import json, collections, numpy as np, sys
from scipy.stats import fisher_exact
from failfn import failed
def L(n):
    try: return [json.loads(l) for l in open(f'results2/camp_{n}.jsonl')]
    except FileNotFoundError: return []
def seed(r): return int(r['key'].split('|')[-1])
Z1 = L('Z1'); A = L('A')
def grp(rows, pref):
    return {seed(r): r for r in rows if r['key'].startswith(pref)}
print('== Z1: anti-windup, S3, 100 seeds')
res = {}
for c in ('ISMC-J', 'FOSMC-J'):
    none = {s: r for s, r in grp(A, f'A|{c}|S3').items() if 100 <= s < 120}
    none.update(grp(Z1, f'Z1|{c}|aw=none'))
    for aw, d in (('none', none), ('cond', grp(Z1, f'Z1|{c}|aw=cond')), ('backcalc', grp(Z1, f'Z1|{c}|aw=backcalc'))):
        n = len(d); nf = sum(failed(r, 'S3') for r in d.values())
        ok = [r['rmse'] for r in d.values() if not failed(r, 'S3')]
        res[(c, aw)] = (n, nf, d)
        print(f'{c:8s} {aw:9s} n={n} fail={nf} rmse_ok={np.mean(ok) if ok else float("nan"):.3f}')
for aw in ('none', 'cond', 'backcalc'):
    a = res[('ISMC-J', aw)]; b = res[('FOSMC-J', aw)]
    if a[0] and b[0]:
        t = [[a[1], a[0]-a[1]], [b[1], b[0]-b[1]]]
        print(aw, 'Fisher ISMC-J vs FOSMC-J p=%.4g' % fisher_exact(t)[1], 'odds', fisher_exact(t)[0])
