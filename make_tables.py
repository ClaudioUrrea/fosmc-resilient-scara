"""Regenerate Tables 2 and 3 of the paper (CSV and Markdown) from results/*.json.

Statistics: mean and standard deviation over the evaluation seeds; runs that diverged
are excluded and counted; p-values: two-sided Wilcoxon signed-rank test, FOSMC vs
ISMC-B, on paired converged runs.
"""
import json, os
import numpy as np
from scipy.stats import wilcoxon

R, OUT = 'results', 'tables'
os.makedirs(OUT, exist_ok=True)
mc = json.load(open(f'{R}/mc.json')); abl = json.load(open(f'{R}/ablation.json'))
CTRL = ['CT-PID', 'ISMC-A', 'ISMC-B', 'FOSMC']
METRICS = [('rmse', 'RMSE (mm)'), ('emax', 'Max. error (mm)'), ('tv', 'TV (N m/s, relative index)')]


def arr(sc, n, k):
    a = np.array([r[k] for r in mc[f'{sc}|{n}']], float)
    a[~np.isfinite(np.array([r['rmse'] for r in mc[f'{sc}|{n}']], float))] = np.nan   # exclude diverged runs
    return a


def pval(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 5 or np.allclose(a[m], b[m]):
        return 'n/a'
    p = wilcoxon(a[m], b[m]).pvalue
    return '<0.001' if p < 1e-3 else '%.3f' % p


rows = [['Scenario', 'Metric'] + CTRL + ['p (FOSMC vs ISMC-B)']]
for sc in ('S1', 'S2', 'S3'):
    for k, lab in METRICS:
        if k == 'emax_post' and sc != 'S3':
            continue
        d = 0 if k == 'tv' else (2 if sc == 'S3' else 3)
        cells = []
        for n in CTRL:
            a = arr(sc, n, k); nd = int(np.isnan(a).sum())
            c = '%.*f +/- %.*f' % (d, np.nanmean(a), d, np.nanstd(a))
            cells.append(c + (' (%d diverged)' % nd if nd else ''))
        rows.append([sc, lab] + cells + [pval(arr(sc, 'FOSMC', k), arr(sc, 'ISMC-B', k))])

fo = arr('S3', 'FOSMC', 'rmse')[:10]; fom = arr('S3', 'FOSMC', 'emax')[:10]
nr = np.array([r['rmse'] for r in abl['no-reconf']], float); nrm = np.array([r['emax'] for r in abl['no-reconf']], float)
wn = np.array([r['rmse'] for r in abl['weak-null']], float)
rows3 = [['Configuration', 'RMSE (mm)', 'Max. error (mm)'],
         ['FOSMC + fault-aware null space + reconfiguration', '%.3f +/- %.3f' % (fo.mean(), fo.std()), '%.3f' % fom.mean()],
         ['Without reconfiguration', '%.1f +/- %.1f' % (np.nanmean(nr), np.nanstd(nr)), '%.1f' % np.nanmean(nrm)],
         ['Weak null-space objective', ('diverged (%d/%d runs)' % (np.isnan(wn).sum(), len(wn))) if np.isnan(wn).any() else '%.1f' % wn.mean(), '-']]


def write(name, rr, title):
    with open(f'{OUT}/{name}.csv', 'w') as fh:
        for r in rr:
            fh.write(','.join('"%s"' % c for c in r) + '\n')
    with open(f'{OUT}/{name}.md', 'w') as fh:
        fh.write('# %s\n\n' % title)
        fh.write('| ' + ' | '.join(rr[0]) + ' |\n|' + '---|' * len(rr[0]) + '\n')
        for r in rr[1:]:
            fh.write('| ' + ' | '.join(r) + ' |\n')


write('table2_monte_carlo', rows, 'Table 2. Monte Carlo results (20 evaluation seeds per controller and scenario)')
write('table3_ablation', rows3, 'Table 3. Ablation in S3 (FOSMC, 10 evaluation seeds)')
print(open(f'{OUT}/table2_monte_carlo.md').read()); print(open(f'{OUT}/table3_ablation.md').read())
