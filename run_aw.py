"""Robustness check: S3 with conditional-integration anti-windup in both SMC variants."""
import json, numpy as np, sim
sel = json.load(open('results/selection.json'))
out = {}
for name, kind, lam, L2 in (('ISMC-A', 'ISMC', 1.0, sel['FOSMC']['L2']), ('ISMC-B', 'ISMC', 1.0, sel['ISMC-B']['L2']), ('FOSMC', 'FOSMC', sel['FOSMC']['lam'], sel['FOSMC']['L2'])):
    rec = []
    for s in range(100, 120):
        o = sim.run(kind, lam, 'S3', seed=s, L2=L2, antiwindup=True)
        m = sim.metrics(o)
        rec.append(dict(seed=s, rmse=m['rmse'], emax=m['emax'], tv=float(m['tv'].sum())))
    out[name] = rec
    print(name, np.nanmean([r['rmse'] for r in rec]), flush=True)
    json.dump(out, open('results/antiwindup.json', 'w'), indent=1)
