"""Round-2 campaigns (Fractal and Fractional, minor revision).
M: reduced-chain margin after isolation (log of the reduced lumped uncertainty) and the post-reconfiguration
   gain schedule on independent seeds: ISMC-J and FOSMC-J, nominal and enlarged P, with and without the
   schedule (20, 800), S3, seeds 100-299 (seeds 200-299 were not used in any earlier analysis).
P: payload configured in the controller model with a +/-20% estimation error, S3, seeds 100-199.
Usage: python camp_R2.py {M|P|all}"""
import os, sys, json, time
for v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
    os.environ[v] = '1'
import numpy as np
import sim2
import ff_campaigns as F

RHO = {'nom': (40., 6., 3.), 'P': (60., 25., 3.)}
BMIN = np.array([1.0, 0.5, 1.0])


def job(spec):
    key, scen, kw = spec
    t0 = time.time()
    o = sim2.run(scenario=scen, **kw)
    r = sim2.summarize(o, scen); r['key'] = key; r['wall'] = time.time() - t0
    if 'Dred' in o and o['detected_at'] is not None:
        t, td, D = o['t'], o['detected_at'], np.abs(o['Dred'])
        marg = BMIN * np.array(kw.get('rho', (40., 6., 3.)))
        f = lambda a: None if not np.any(np.isfinite(a)) else float(np.nanmax(a))
        for name, lo, hi in (('a', td, td + 0.5), ('b', td + 0.5, 10.0), ('all', td, 10.0)):
            m = (t >= lo) & (t < hi)
            r['Dred_' + name] = [f(D[m, i]) for i in range(2)]
            ok = np.isfinite(D[m, :2])
            r['viol_' + name] = [float(np.mean(D[m, i][ok[:, i]] > marg[i])) if ok[:, i].any() else None for i in range(2)]
        r['margin'] = marg[:2].tolist()
    r['payload'] = o['payload']
    return r


def camp_M(sel):
    sp = []
    for c in ('ISMC-J', 'FOSMC-J'):
        for tag in ('nom', 'P'):
            for sch in ('none', 'sched'):
                for s in range(100, 300):
                    extra = dict(rho=RHO[tag], log_red=True)
                    if sch == 'sched': extra['gains_post'] = (20.0, 800.0)
                    sp.append(F.S(f'M|{c}|{tag}|{sch}', 'S3', s, F.ck(c, sel), **extra))
    return sp


def camp_P(sel):
    sp = []
    for c in ('ISMC-A', 'ISMC-J', 'FOSMC', 'FOSMC-J'):
        for s in range(100, 200):
            sp.append(F.S(f'P|{c}|pay20', 'S3', s, F.ck(c, sel), payload_err=0.2))
    return sp


if __name__ == '__main__':
    F.job = job
    sel = F.sel_load(); w = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if w in ('M', 'all'): F.run_jobs(camp_M(sel), 'camp_R2M')
    if w in ('P', 'all'): F.run_jobs(camp_P(sel), 'camp_R2P')
    print('R2 DONE', flush=True)
