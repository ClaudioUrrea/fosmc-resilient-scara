"""Figure: mechanism of the post-lock failures (payload dependence, unconstrained torque demand, limit cycles)."""
import json, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import fontsetup; fontsetup.apply()
from failfn import failed
PL = {int(k): v for k, v in json.load(open('diag_payload.json'))['payload'].items()}
def L(n): return [json.loads(l) for l in open(f'results2/camp_{n}.jsonl')]
A, Z, Q, S = {}, {}, {}, {}
for r in L('A'):
    k = r['key'].split('|')
    if k[2] == 'S3': A[(k[1], int(k[3]))] = r
for r in L('Z1'):
    k = r['key'].split('|'); Z[(k[1], k[2][3:], int(k[4]))] = r
for r in L('Q') + L('Q2'):
    k = r['key'].split('|')
    if k[3] == 'S3': Q[(k[1], int(k[4]))] = r
for r in L('S'):
    k = r['key'].split('|'); S[(k[1], k[3], int(k[5]))] = r
SEEDS = range(100, 300)
R2 = {}
for r in L('R2M'):
    k = r['key'].split('|'); R2[(k[1], k[2], k[3], int(k[5]))] = r
cfg = {'ISMC-J': lambda s: R2[('ISMC-J', 'nom', 'none', s)],
       'FOSMC-J': lambda s: R2[('FOSMC-J', 'nom', 'none', s)],
       'ISMC-J, enlarged $P$': lambda s: R2[('ISMC-J', 'P', 'none', s)],
       'FOSMC-J, enlarged $P$': lambda s: R2[('FOSMC-J', 'P', 'none', s)],
       'FOSMC-J, enlarged $P$ + schedule': lambda s: R2[('FOSMC-J', 'P', 'sched', s)]}
PL = {s: R2[('ISMC-J', 'nom', 'none', s)]['payload'] for s in SEEDS}
sty = {'ISMC-J': dict(color='tab:purple', ls='--', marker='s'), 'FOSMC-J': dict(color='tab:orange', ls='-', marker='o'),
       'ISMC-J, enlarged $P$': dict(color='tab:purple', ls='-.', marker='v'),
       'FOSMC-J, enlarged $P$': dict(color='tab:red', ls='-', marker='D'), 'FOSMC-J, enlarged $P$ + schedule': dict(color='tab:green', ls='-', marker='x')}
bins = np.array([0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0001]); mid = 0.5 * (bins[1:] + bins[:-1])
fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.6))
for name, get in cfg.items():
    fr = []
    for i in range(len(bins) - 1):
        ss = [s for s in SEEDS if bins[i] <= PL[s] < bins[i + 1]]
        fr.append(100 * np.mean([failed(get(s), 'S3') for s in ss]))
    ax[0].plot(mid, fr, lw=1, ms=3, label=name, **sty[name])
ax[0].set_xlabel('Payload (kg)'); ax[0].set_ylabel('Failed runs (%)'); ax[0].set_title('(a) Failures vs. payload (S3, 200 seeds)', fontsize=8)
ax[0].legend(fontsize=5.2, loc='upper left'); ax[0].set_ylim(-5, 105)
D = json.load(open('diag_demand.json'))
for c, cf, lab, st in (('ISMC-J', 'nom', 'ISMC-J', sty['ISMC-J']), ('ISMC-J', 'P', 'ISMC-J, enl. $P$', sty['ISMC-J, enlarged $P$']),
                       ('FOSMC-J', 'P', 'FOSMC-J, enl. $P$', sty['FOSMC-J, enlarged $P$'])):
    rows = [d for d in D if d['c'] == c and d['cf'] == cf]
    x = np.array([PL[d['s']] for d in rows]); y = np.array([d['peak'][1] if not d['div'] else 400 for d in rows])
    ax[1].scatter(x, np.minimum(y, 400), s=5, color=st['color'], marker=st['marker'], label=lab, alpha=0.8)
ax[1].axhline(60, color='k', lw=0.7); ax[1].text(0.05, 64, r'$\tau_{\max,1}$', fontsize=6.5)
ax[1].set_yscale('log'); ax[1].set_ylim(15, 500); ax[1].set_yticks([20, 40, 60, 100, 200, 400]); ax[1].set_yticklabels(['20', '40', '60', '100', '200', '400']); ax[1].minorticks_off(); ax[1].set_xlabel('Payload (kg)'); ax[1].set_ylabel(r'Peak $|\tau_1|$ demand (N m)')
ax[1].set_title('(b) Demand without saturation', fontsize=8); ax[1].legend(fontsize=5.5, loc='lower right')
ax[1].text(0.1, 300, 'diverged (plotted at 400)', fontsize=5.5)
for n, lab, st in (('ISMC-J_cond_129', 'ISMC-J, cond. int.', dict(color='tab:purple', ls='--')), ('FOSMC-J_cond_129', 'FOSMC-J, cond. int.', dict(color='k', ls='-'))):
    z = np.load(f'diag/{n}.npz'); t = z['t']; m = (t >= 7) & (t < 8)
    ax[2].plot(t[m], 1e3 * (z['q'][m, 1] - z['qr'][m, 1]), lw=0.8, label=lab, **st)
ax[2].set_xlabel('Time (s)'); ax[2].set_ylabel(r'$\theta_1$ error (mrad)'); ax[2].set_title('(c) Limit cycles, seed 129 (2.9 kg)', fontsize=8)
ax[2].legend(fontsize=5.5, loc='upper right')
for a in ax: a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig('figs/fig_failmech.pdf'); fig.savefig('figs/fig_failmech.png', dpi=200)
print('ok')
