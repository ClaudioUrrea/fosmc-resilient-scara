"""Analysis of the Round-2 campaigns (camp_R2.py): reduced-chain margin table, gain schedule on seeds 200-299,
payload configured in the controller model. Writes results2/an_R2.json."""
import json, os, sys
import numpy as np
from scipy.stats import fisher_exact, wilcoxon
from failfn import failed


def load(n):
    rows = []
    p = os.path.join('results2', n + '.jsonl')
    if os.path.exists(p):
        for l in open(p):
            try: rows.append(json.loads(l))
            except Exception: pass
    d = {}
    for r in rows: d[r['key']] = r
    return list(d.values())


out = {}
M = load('camp_R2M')
g = {}
for r in M:
    k = r['key'].split('|'); c, tag, sch, s = k[1], k[2], k[3], int(k[-1])
    g.setdefault((c, tag, sch, 'A' if s < 200 else 'B'), []).append(r)
res = {}
for key in sorted(g):
    rs = g[key]; F = [failed(r, 'S3') for r in rs]; ok = [r for r, f in zip(rs, F) if not f]
    e = dict(n=len(rs), fail=int(sum(F)))
    e['rmse'] = float(np.mean([r['rmse'] for r in ok])) if ok else None
    e['rec'] = float(np.mean([r['rec_1'] for r in ok if r.get('rec_1') is not None])) if ok else None
    e['emax'] = float(np.mean([r['emax'] for r in ok])) if ok else None
    e['tv'] = float(np.mean([r['tv'] for r in ok if r.get('tv')])) if ok else None
    for grp, sel in (('ok', ok), ('fail', [r for r, f in zip(rs, F) if f])):
        for w in ('a', 'b'):
            for i, nm in ((0, 'd'), (1, 'th1')):
                v = [r['Dred_' + w][i] for r in sel if r.get('Dred_' + w) and r['Dred_' + w][i] is not None]
                vi = [r['viol_' + w][i] for r in sel if r.get('viol_' + w) and r['viol_' + w][i] is not None]
                if v:
                    e[f'D_{grp}_{w}_{nm}'] = [float(np.median(v)), float(np.max(v))]
                    e[f'V_{grp}_{w}_{nm}'] = [float(np.median(vi)), float(np.max(vi))] if vi else None
                    e[f'frac_violating_{grp}_{w}_{nm}'] = float(np.mean([x > m for x, m in zip(v, [r['margin'][i] for r in sel if r.get('Dred_' + w) and r['Dred_' + w][i] is not None])]))
    pay_f = [r['payload'] for r, f in zip(rs, F) if f]; e['payload_fail_min'] = float(min(pay_f)) if pay_f else None
    e['margin'] = rs[0].get('margin')
    res['|'.join(key)] = e
out['M'] = res
# paired comparisons on independent seeds
def pair(a, b):
    A = {r['key'].split('|')[-1]: r for r in a}; B = {r['key'].split('|')[-1]: r for r in b}
    common = [s for s in A if s in B and not failed(A[s], 'S3') and not failed(B[s], 'S3')]
    x = np.array([A[s]['rmse'] for s in common]); y = np.array([B[s]['rmse'] for s in common])
    p = wilcoxon(x, y).pvalue if len(common) > 5 else None
    return dict(n=len(common), mean_a=float(x.mean()), mean_b=float(y.mean()), rel=float(x.mean() / y.mean() - 1), p=float(p) if p is not None else None)
cmp = {}
for tag in ('nom', 'P'):
    for half in ('A', 'B'):
        a = g.get(('FOSMC-J', tag, 'sched', half), []); b = g.get(('ISMC-J', tag, 'sched', half), [])
        if a and b: cmp[f'sched|{tag}|{half}'] = pair(a, b)
        fa = sum(failed(r, 'S3') for r in g.get(('ISMC-J', tag, 'none', half), [])); fb = sum(failed(r, 'S3') for r in g.get(('FOSMC-J', tag, 'none', half), []))
        n1 = len(g.get(('ISMC-J', tag, 'none', half), [])); n2 = len(g.get(('FOSMC-J', tag, 'none', half), []))
        if n1 and n2: cmp[f'fisher_none|{tag}|{half}'] = float(fisher_exact([[fa, n1 - fa], [fb, n2 - fb]])[1])
out['cmp'] = cmp
P = load('camp_R2P'); A = load('camp_A') + load('camp_Z1')
pres = {}
for c in ('ISMC-A', 'ISMC-J', 'FOSMC', 'FOSMC-J'):
    rs = [r for r in P if r['key'].split('|')[1] == c]
    if not rs: continue
    F = [failed(r, 'S3') for r in rs]; ok = [r for r, f in zip(rs, F) if not f]
    pres[c] = dict(n=len(rs), fail=int(sum(F)), rmse=float(np.mean([r['rmse'] for r in ok])) if ok else None,
                   payload_fail=[round(r['payload'], 2) for r, f in zip(rs, F) if f])
out['P'] = pres
json.dump(out, open('results2/an_R2.json', 'w'), indent=1)
print(json.dumps(out, indent=1)[:6000])
