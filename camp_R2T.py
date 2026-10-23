"""Round-2 campaign T: tuning with a robustness constraint (Reviewer 1, Suggestion 1).
Stage 1: the most accurate (in order) admissible configurations of each family in the stored tuning grid are re-run on the
tuning seeds (0-3) in S2 and S3 with a 2-ms actuator lag (K1) and with a 2-ms input delay; the most accurate
configuration without any failure is selected (ISMC-K, FOSMC-K).
Stage 2: the selected controllers are evaluated on seeds 100-119 (nominal S1-S3; K1 and 2-ms delay in S2 and S3).
Usage: python camp_R2T.py"""
import os, sys, json
for v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
    os.environ[v] = '1'
import sim2, ff_campaigns as F
from failfn import failed

CONDS = {'K1': dict(act_tc=0.002), 'D2': dict(delay=0.002)}


def job(spec):
    import time
    key, scen, kw = spec
    t0 = time.time(); o = sim2.run(scenario=scen, **kw)
    r = sim2.summarize(o, scen); r['key'] = key; r['wall'] = time.time() - t0
    return r


def candidates():
    d = json.load(open(os.path.join(F.OUT, 'selection2.json')))
    bud = d['selection']['budget']; out = {}
    for fam in ('ISMC', 'FOSMC'):
        c = [x for x in d[fam] if not x['diverged'] and x['tv'] <= bud]
        c.sort(key=lambda x: x['rmse']); out[fam] = c[:30]
    return out


def base(fam, x):
    if fam == 'ISMC': return dict(ctrl_kind='ISMC', lam=1.0, L1=x['L1'], L2=x['L2'])
    return dict(ctrl_kind='FOSMC', lam=x['lam'], L1=x['L1'], L2=x['L2'])


def tag(fam, x):
    return f"{fam}|{x['L1']}|{x.get('lam', 1.0)}|{x['L2']}"


if __name__ == '__main__':
    F.job = job
    cand = candidates(); sel = {}
    for fam, lst in cand.items():
        res = []; best = None
        for x in lst:
            sp = [F.S(f'T1|{tag(fam, x)}|{cn}', sc, s, base(fam, x), **cv)
                  for cn, cv in CONDS.items() for sc in ('S2', 'S3') for s in F.TUNE]
            F.run_jobs(sp, 'camp_R2T1')
            rows = [json.loads(l) for l in open(os.path.join(F.OUT, 'camp_R2T1.jsonl'))]
            rr = [r for r in rows if r['key'].startswith(f'T1|{tag(fam, x)}|')]
            nf = sum(failed(r, r['key'].split('|')[-2]) for r in rr)
            res.append((x['rmse'], nf, x['L1'], x.get('lam', 1.0), x['L2']))
            print('cand', fam, res[-1], flush=True)
            if nf == 0: best = x; break
        sel[fam] = dict(best=best, table=res)
    json.dump(sel, open(os.path.join(F.OUT, 'sel_R2T.json'), 'w'), indent=1)
    print('T1 selection', {k: v['best'] for k, v in sel.items()}, flush=True)
    sp2 = []
    for fam, nm in (('ISMC', 'ISMC-K'), ('FOSMC', 'FOSMC-K')):
        x = sel[fam]['best']
        if x is None: continue
        for sc in ('S1', 'S2', 'S3'):
            for s in F.EV: sp2.append(F.S(f'T2|{nm}|nom', sc, s, base(fam, x)))
        for cn, cv in CONDS.items():
            for sc in ('S2', 'S3'):
                for s in F.EV: sp2.append(F.S(f'T2|{nm}|{cn}', sc, s, base(fam, x), **cv))
    F.run_jobs(sp2, 'camp_R2T2')
    print('T DONE', flush=True)
