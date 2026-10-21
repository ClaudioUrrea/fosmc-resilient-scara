"""Campaigns for the R1 revision (Fractal and Fractional). Usage: python ff_campaigns.py {stage1|A|D|E|F|G|H|I|J|K|all}
Results are appended (resumable) to results2/<name>.jsonl, one summary per run."""
import os, sys, json, time, math
for v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
    os.environ[v] = '1'
from multiprocessing import Pool
import numpy as np
import sim2

OUT = 'results2'; os.makedirs(OUT, exist_ok=True)
BUDGET = 4141.5197589352665          # common TV budget of the submitted version
EV, TUNE = list(range(100, 120)), [0, 1, 2, 3]
WORKERS = int(os.environ.get('FF_WORKERS', '2'))


def kappa(lam, L):
    return L ** (lam - 1.0) / math.gamma(lam)


def job(spec):
    key, scen, kw = spec
    t0 = time.time()
    o = sim2.run(scenario=scen, **kw)
    r = sim2.summarize(o, scen); r['key'] = key; r['wall'] = time.time() - t0
    return r


def run_jobs(specs, name):
    path = os.path.join(OUT, name + '.jsonl'); done = set()
    if os.path.exists(path):
        for ln in open(path):
            try: done.add(json.loads(ln)['key'])
            except Exception: pass
    todo = [s for s in specs if s[0] not in done]
    print(f'[{name}] {len(specs)} jobs, {len(todo)} to run', flush=True)
    t0 = time.time(); n = 0
    with Pool(WORKERS) as p, open(path, 'a') as f:
        for r in p.imap_unordered(job, todo, chunksize=1):
            f.write(json.dumps(r) + '\n'); f.flush(); n += 1
            if n % 50 == 0:
                print(f'[{name}] {n}/{len(todo)} {time.time()-t0:.0f}s', flush=True)
    print(f'[{name}] done {time.time()-t0:.0f}s', flush=True)


def load(name):
    p = os.path.join(OUT, name + '.jsonl')
    return [json.loads(l) for l in open(p)] if os.path.exists(p) else []


def parse(group):
    d = {}
    for tok in group.split('|'):
        if '=' in tok:
            a, b = tok.split('=', 1); d[a] = float(b)
    return d


def tuning_table(rows, kind):
    g = {}
    for r in rows:
        k = r['key']
        if f'|{kind}|' not in k: continue
        grp = k.rsplit('|', 1)[0]; g.setdefault(grp, []).append(r)
    tab = []
    for grp, rs in g.items():
        if len(rs) < len(TUNE): continue
        bad = any(r['diverged'] or r['rmse'] is None for r in rs)
        tab.append(dict(grp=grp, **parse(grp), rmse=None if bad else float(np.mean([r['rmse'] for r in rs])),
                        tv=None if bad else float(np.mean([r['tv'] for r in rs])), diverged=bad))
    return tab


def best(tab):
    ok = [t for t in tab if t['rmse'] is not None and t['tv'] <= BUDGET]
    return min(ok, key=lambda t: t['rmse']) if ok else None


def r3(x):
    return float(f'{x:.3g}')


def stage1():
    specs = []
    for L1 in (10, 15, 20, 30, 40, 60):
        for L2 in (50, 100, 200, 400, 800, 1600, 2400, 3200):
            for s in TUNE:
                specs.append((f'B|ISMC|L1={L1}|L2={L2}|S2|{s}', 'S2', dict(ctrl_kind='ISMC', lam=1.0, L1=float(L1), L2=float(L2), seed=s)))
    for L1 in (10, 20, 40):
        for lam in (0.5, 0.7, 0.9):
            for L2 in (200, 400, 800, 1600, 3200):
                for s in TUNE:
                    specs.append((f'B|FOSMC|L1={L1}|lam={lam}|L2={L2}|S2|{s}', 'S2', dict(ctrl_kind='FOSMC', lam=lam, L1=float(L1), L2=float(L2), seed=s)))
    for w in (20, 22, 24, 26, 28):
        for s in TUNE:
            specs.append((f'B|CTPID|w={w}|S2|{s}', 'S2', dict(ctrl_kind='CTPID', omega=float(w), seed=s)))
    run_jobs(specs, 'stage1a')
    rows = load('stage1a')
    bi, bf = best(tuning_table(rows, 'ISMC')), best(tuning_table(rows, 'FOSMC'))
    ref = []
    for f1 in (0.8, 1.0, 1.25):
        for f2 in (0.8, 1.0, 1.25):
            if f1 == f2 == 1.0: continue
            L1, L2 = r3(bi['L1'] * f1), r3(bi['L2'] * f2)
            for s in TUNE:
                ref.append((f'B|ISMC|L1={L1}|L2={L2}|S2|{s}', 'S2', dict(ctrl_kind='ISMC', lam=1.0, L1=L1, L2=L2, seed=s)))
    for f1 in (0.8, 1.0, 1.25):
        for f2 in (0.8, 1.0, 1.25):
            for dl in (-0.1, 0.0, 0.1):
                if f1 == f2 == 1.0 and dl == 0.0: continue
                lam = round(min(0.95, max(0.3, bf['lam'] + dl)), 2)
                L1, L2 = r3(bf['L1'] * f1), r3(bf['L2'] * f2)
                for s in TUNE:
                    ref.append((f'B|FOSMC|L1={L1}|lam={lam}|L2={L2}|S2|{s}', 'S2', dict(ctrl_kind='FOSMC', lam=lam, L1=L1, L2=L2, seed=s)))
    run_jobs(ref, 'stage1b')
    rows = load('stage1a') + load('stage1b')
    ti, tf, tc = tuning_table(rows, 'ISMC'), tuning_table(rows, 'FOSMC'), tuning_table(rows, 'CTPID')
    sel = {'budget': BUDGET, 'ISMC-J': best(ti), 'FOSMC-J': best(tf), 'CT-PID': best(tc)}
    sel['CT-PID']['omega'] = sel['CT-PID'].pop('w')
    json.dump(dict(selection=sel, ISMC=ti, FOSMC=tf, CTPID=tc), open(os.path.join(OUT, 'selection2.json'), 'w'), indent=1)
    print('SELECTION', json.dumps(sel), flush=True)


def sel_load():
    return json.load(open(os.path.join(OUT, 'selection2.json')))['selection']


def ck(name, sel, Lmem=1.0):
    if name == 'CT-PID': return dict(ctrl_kind='CTPID', omega=float(sel['CT-PID']['omega']))
    if name == 'ISMC-A': return dict(ctrl_kind='ISMC', lam=1.0, L1=20.0, L2=800.0)
    if name == 'ISMC-B': return dict(ctrl_kind='ISMC', lam=1.0, L1=20.0, L2=1600.0)
    if name == 'ISMC-M': return dict(ctrl_kind='ISMC', lam=1.0, L1=20.0, L2=800.0 * kappa(0.7, Lmem))
    if name == 'ISMC-J': return dict(ctrl_kind='ISMC', lam=1.0, L1=sel['ISMC-J']['L1'], L2=sel['ISMC-J']['L2'])
    if name == 'FOSMC': return dict(ctrl_kind='FOSMC', lam=0.7, L1=20.0, L2=800.0, L_mem=Lmem)
    if name == 'FOSMC-J': return dict(ctrl_kind='FOSMC', lam=sel['FOSMC-J']['lam'], L1=sel['FOSMC-J']['L1'], L2=sel['FOSMC-J']['L2'])
    raise KeyError(name)


def S(key, scen, seed, base, **extra):
    kw = dict(base); kw.update(extra); kw['seed'] = seed
    return (f'{key}|{scen}|{seed}', scen, kw)


def camp_A(sel):
    sp = []
    for c in ('CT-PID', 'ISMC-A', 'ISMC-B', 'ISMC-M', 'ISMC-J', 'FOSMC', 'FOSMC-J'):
        for sc in ('S1', 'S2', 'S3'):
            for s in EV:
                sp.append(S(f'A|{c}', sc, s, ck(c, sel), log_delta=(c != 'CT-PID')))
    return sp


def camp_D(sel):
    sp = []
    for L in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0):
        for c in ('FOSMC', 'ISMC-M'):
            for s in EV:
                sp.append(S(f'D|{c}|L={L}', 'S3', s, ck(c, sel, Lmem=L), L_mem=L))
    return sp


def camp_E(sel):
    return [S(f'E|{c}', 'S3', s, ck(c, sel), reset_on_reconf=False) for c in ('FOSMC', 'ISMC-A', 'ISMC-B', 'ISMC-J') for s in EV]


def camp_F(sel):
    return [S(f'F|{c}|aw={aw}', 'S3', s, ck(c, sel), antiwindup=aw)
            for aw in ('cond', 'backcalc') for c in ('ISMC-A', 'ISMC-B', 'ISMC-J', 'ISMC-M', 'FOSMC') for s in EV]


def camp_G(sel):
    sp = []
    for b1 in (0.7, 0.5, 0.35):
        for ts in (0.8, 1.0, 1.2):
            if b1 == 0.5 and ts == 1.0: continue
            for c in ('ISMC-A', 'ISMC-B', 'ISMC-J', 'ISMC-M', 'FOSMC'):
                for s in EV[:10]:
                    sp.append(S(f'G|{c}|b1={b1}|ts={ts}', 'S3', s, ck(c, sel), b1=b1, tau_scale=ts))
    return sp


def camp_H(sel):
    return [S(f'H|{c}', sc, s, ck(c, sel), loe_ramp=2.0, b1=0.5)
            for c in ('CT-PID', 'ISMC-A', 'ISMC-B', 'ISMC-J', 'FOSMC') for sc in ('S2', 'S3') for s in EV]


def camp_I(sel):
    return [S(f'I|FOSMC|rth={sc_}', sc, s, ck('FOSMC', sel), rth_scale=sc_)
            for sc_ in (0.5, 0.75, 1.5, 2.0) for sc in ('S1', 'S2', 'S3') for s in EV]


def camp_J(sel):
    return [S(f'J|{c}|dly={d}', 'S2', s, ck(c, sel), delay=d)
            for d in (0.0, 0.003, 0.005, 0.007, 0.009, 0.011) for c in ('ISMC-A', 'ISMC-B', 'ISMC-J', 'FOSMC') for s in EV[:10]]


def camp_K(sel):
    lv = {'K1': dict(act_tc=0.002), 'K2': dict(act_tc=0.002, enc_res=2 * math.pi / 2 ** 17, unc_scale=1.5),
          'K3': dict(act_tc=0.004, enc_res=2 * math.pi / 2 ** 16, unc_scale=2.0)}
    return [S(f'K|{c}|{k}', sc, s, ck(c, sel), **v)
            for k, v in lv.items() for c in ('ISMC-A', 'ISMC-B', 'ISMC-J', 'FOSMC') for sc in ('S2', 'S3') for s in EV[:10]]


CAMPS = dict(A=camp_A, D=camp_D, E=camp_E, F=camp_F, I=camp_I, G=camp_G, H=camp_H, J=camp_J, K=camp_K)

if __name__ == '__main__':
    st = sys.argv[1]
    if st in ('stage1', 'all'):
        stage1()
    if st == 'all':
        sel = sel_load()
        for c in 'ADEFIGHJK':
            run_jobs(CAMPS[c](sel), 'camp_' + c)
        print('ALL DONE', flush=True)
    elif st in CAMPS:
        run_jobs(CAMPS[st](sel_load()), 'camp_' + st)
