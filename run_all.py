"""Runs every numerical experiment reported in the manuscript.

Controllers (gains from results/selection.json, chosen by tune_final.py on tuning
seeds 0-3 under a common control-activity budget; never on evaluation seeds):
  CT-PID  : computed-torque PID (best omega within the budget)
  ISMC-A  : integer integral sliding mode with the same gains as the FOSMC
  ISMC-B  : best integer integral sliding mode within the budget
  FOSMC   : proposed fractional-order sliding mode
Evaluation seeds: 100-119.
"""
import argparse, json, os, time
import numpy as np
import sim

ap = argparse.ArgumentParser()
ap.add_argument('--l1', type=float, default=None)
ap.add_argument('--l2', type=float, default=None)
ap.add_argument('--quick', action='store_true', help='few seeds (smoke test)')
args = ap.parse_args()
if args.l1:
    sim.NOMINAL['l1'] = args.l1
if args.l2:
    sim.NOMINAL['l2'] = args.l2

SEL = json.load(open('results/selection.json'))
CTRL = {
    'CT-PID': dict(ctrl_kind='CTPID', lam=1.0, omega=SEL['CT-PID']['omega']),
    'ISMC-A': dict(ctrl_kind='ISMC', lam=1.0, L2=SEL['FOSMC']['L2']),   # same gains as the FOSMC
    'ISMC-B': dict(ctrl_kind='ISMC', lam=1.0, L2=SEL['ISMC-B']['L2']),   # best integer within the budget
    'FOSMC': dict(ctrl_kind='FOSMC', lam=SEL['FOSMC']['lam'], L2=SEL['FOSMC']['L2']),
}
print('selected', CTRL, flush=True)
SEEDS = list(range(100, 104)) if args.quick else list(range(100, 120))
os.makedirs('results', exist_ok=True)


def save_json(obj, name):
    with open(os.path.join('results', name), 'w') as f:
        json.dump(obj, f, indent=1)


def export_csv(o, path):
    q, x, xd, fl = o['q'], o['x'], o['xd'], o['flags']
    with open(path, 'w') as f:
        f.write('t,d,theta1,theta2,x,y,xd,yd,loe,locked,reconf\n')
        for k in range(0, len(o['t'])):
            f.write('%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n' % (
                o['t'][k], q[k, 0], q[k, 1], q[k, 2], x[k, 0], x[k, 1], xd[k, 0], xd[k, 1],
                fl[k, 0], fl[k, 1], fl[k, 2]))


t0 = time.time()
# ---------------------------------------------------------------- 1. representative runs
rep = {}
for sc in (() if os.path.exists('results/rep_S3_FOSMC_weaknull.npz') else ('S1', 'S2', 'S3')):
    for name, c in CTRL.items():
        o = sim.run(scenario=sc, seed=100, **c)
        np.savez_compressed(f'results/rep_{sc}_{name}.npz', t=o['t'], q=o['q'], qr=o['qr'], x=o['x'],
                            xd=o['xd'], tau=o['tau'], r=o['r'], flags=o['flags'],
                            det=np.array([-1 if o['detected_at'] is None else o['detected_at']]))
        if sc == 'S3' and name == 'FOSMC':
            export_csv(o, 'results/traj_S3_FOSMC.csv')
        if sc == 'S1' and name == 'FOSMC':
            export_csv(o, 'results/traj_S1_FOSMC.csv')
# ablations on the representative seed
if not os.path.exists('results/rep_S3_FOSMC_weaknull.npz'):
  o = sim.run(scenario='S3', seed=100, reconfigure=False, **CTRL['FOSMC'])
  np.savez_compressed('results/rep_S3_FOSMC_noreconf.npz', t=o['t'], x=o['x'], xd=o['xd'], tau=o['tau'], q=o['q'])
  o = sim.run(scenario='S3', seed=100, kn=4.0, W=(4.0, 1.0, 1.0), **CTRL['FOSMC'])
  np.savez_compressed('results/rep_S3_FOSMC_weaknull.npz', t=o['t'], x=o['x'], xd=o['xd'], tau=o['tau'], q=o['q'])
print('representative runs done %.0f s' % (time.time() - t0), flush=True)

# ---------------------------------------------------------------- 2. Monte Carlo
mc = json.load(open('results/mc.json')) if os.path.exists('results/mc.json') else {}
for sc in ('S1', 'S2', 'S3'):
    for name, c in CTRL.items():
        if f'{sc}|{name}' in mc: continue
        rec = []
        for s in SEEDS:
            o = sim.run(scenario=sc, seed=s, **c)
            m = sim.metrics(o)
            # post-fault window metrics for S3 (lock at 4 s)
            tt = o['t']; w = (tt >= 4.0) & (tt < 5.0)
            ex = np.linalg.norm(o['x'] - o['xd'], axis=1) * 1e3
            rec.append(dict(seed=s, rmse=m['rmse'], emax=m['emax'], tv=float(m['tv'].sum()),
                            energy=float(m['energy'].sum()),
                            emax_post=float(ex[w].max()),
                            rmse_ss=float(np.sqrt(np.mean(ex[tt >= 6.5] ** 2))),
                            det=o['detected_at']))
        mc[f'{sc}|{name}'] = rec
        print(sc, name, 'rmse %.3f +- %.3f' % (np.mean([r['rmse'] for r in rec]), np.std([r['rmse'] for r in rec])),
              '%.0f s' % (time.time() - t0), flush=True)
        save_json(mc, 'mc.json')

# ---------------------------------------------------------------- 3. ablations (S3)
abl = {}
for label, kw in (('no-reconf', dict(reconfigure=False)), ('weak-null', dict(kn=4.0, W=(4.0, 1.0, 1.0)))):
    rec = []
    for s in SEEDS[:10]:
        o = sim.run(scenario='S3', seed=s, **CTRL['FOSMC'], **kw)
        m = sim.metrics(o); rec.append(dict(rmse=m['rmse'], emax=m['emax']))
    abl[label] = rec
    save_json(abl, 'ablation.json')
mem = {}
for L in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0):  # memory length [s]
    rec = []
    for s in SEEDS[:6]:
        o = sim.run(scenario='S2', seed=s, L_mem=L, **CTRL['FOSMC'])
        m = sim.metrics(o); rec.append(dict(rmse=m['rmse'], emax=m['emax'], tv=float(m['tv'].sum())))
    mem[str(L)] = rec
    save_json(mem, 'memory.json')
print('ablations done %.0f s' % (time.time() - t0), flush=True)

# ---------------------------------------------------------------- 4. delay robustness sweep
sweep = []
for lam in (0.3, 0.5, 0.7, 1.0):
    for L2 in (200.0, 400.0, 800.0, 1600.0):
        for dly in (0.0, 0.003, 0.005, 0.007, 0.009, 0.011):
            vals = []
            for s in (0, 1):
                kind = 'ISMC' if lam == 1.0 else 'FOSMC'
                o = sim.run(kind, lam, 'S2', seed=s, L2=L2, delay=dly)
                vals.append(sim.metrics(o)['rmse'])
            sweep.append(dict(lam=lam, L2=L2, delay=dly, rmse=float(np.mean(vals))))
            save_json(sweep, 'delay_sweep.json')
    print('delay sweep lam', lam, '%.0f s' % (time.time() - t0), flush=True)
print('ALL DONE %.0f s' % (time.time() - t0), flush=True)
