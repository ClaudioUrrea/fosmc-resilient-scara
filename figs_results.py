"""Result figures from results/*.npz and results/*.json."""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fontsetup
fontsetup.apply()
R = 'results'; F = 'figures'
os.makedirs(F, exist_ok=True)
NAMES = ['CT-PID', 'ISMC-A', 'ISMC-B', 'FOSMC']
LAB = {'CT-PID': 'CT-PID', 'ISMC-A': 'ISMC-A', 'ISMC-B': 'ISMC-B', 'FOSMC': 'FOSMC'}
STY = {'CT-PID': dict(color='0.45', ls=':'), 'ISMC-A': dict(color='tab:blue', ls='--'),
       'ISMC-B': dict(color='tab:green', ls='-.'), 'FOSMC': dict(color='tab:red', ls='-')}


def save(fig, name):
    fig.tight_layout(); fig.savefig(f'{F}/{name}.pdf'); fig.savefig(f'{F}/{name}.png'); plt.close(fig)


_BB = dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.85)


def shade(ax, sc):
    if sc in ('S2', 'S3'):
        ax.axvline(3.0, color='orange', lw=0.8); ax.text(3.03, 0.05, 'LOE', color='darkorange', transform=ax.get_xaxis_transform(), fontsize=7, bbox=_BB)
    if sc == 'S3':
        ax.axvline(4.0, color='r', lw=0.8); ax.text(4.03, 0.05, 'Lock', color='r', transform=ax.get_xaxis_transform(), fontsize=7, bbox=_BB)
    ax.axvline(6.0, color='purple', lw=0.6, ls='--'); ax.text(6.03, 0.05, 'Step disturbance', color='purple', transform=ax.get_xaxis_transform(), fontsize=7, bbox=_BB)


def load(sc, name):
    return np.load(f'{R}/rep_{sc}_{name}.npz')


# ------------------------------------------------------------------ Fig: S3 tracking
def fig_s3():
    fig = plt.figure(figsize=(7.0, 4.6))
    ax1 = fig.add_subplot(2, 2, 1); ax2 = fig.add_subplot(2, 2, 2); ax3 = fig.add_subplot(2, 1, 2)
    d = load('S3', 'FOSMC')
    ax1.plot(d['xd'][:, 0], d['xd'][:, 1], 'g--', lw=1.2, label='Desired')
    ax1.plot(d['x'][:, 0], d['x'][:, 1], 'r-', lw=0.7, label='Executed (FOSMC)')
    ax1.set_ylim(0.26, 0.46)
    ax1.set_xlabel('$x$ (m)'); ax1.set_ylabel('$y$ (m)'); ax1.legend(loc='lower center')
    ax1.set_title('(a) Task space, scenario S3')
    k = d['t'] >= 3.9; kk = d['t'] < 4.6; m = k & kk
    for n in NAMES:
        e = load('S3', n)
        ax2.plot(e['x'][m, 0], e['x'][m, 1] * 1e3, **STY[n], lw=1, label=LAB[n])
    ax2.plot(d['xd'][m, 0], d['xd'][m, 1] * 1e3, 'k', lw=0.6, alpha=0.6, label='Desired')
    ax2.set_xlabel('$x$ (m)'); ax2.set_ylabel('$y$ (mm)'); ax2.set_title('(b) Zoom: 3.9–4.6 s (joint lock)')
    ax2.legend(ncol=2)
    for n in NAMES:
        e = load('S3', n)
        err = np.linalg.norm(e['x'] - e['xd'], axis=1) * 1e3
        ax3.semilogy(e['t'], err + 1e-4, **STY[n], lw=0.9, label=LAB[n])
    shade(ax3, 'S3'); ax3.set_ylim(1e-3, 1e2); ax3.set_xlim(0, 10)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel(r'$\|\mathbf{x}-\mathbf{x}_d\|$ (mm)'); ax3.legend(ncol=4, loc='upper right', framealpha=0.95)
    ax3.set_title('(c) Cartesian tracking error')
    save(fig, 'fig_s3_tracking')


# ------------------------------------------------------------------ Fig: S3 internals FOSMC
def fig_internals():
    d = load('S3', 'FOSMC'); t = d['t']; det = float(d['det'][0])
    fig, ax = plt.subplots(3, 1, figsize=(7.0, 5.4), sharex=True)
    lab = ['$d$ (m)', r'$\theta_1$ (rad)', r'$\theta_2$ (rad)']
    for i, c in enumerate(['tab:orange', 'tab:blue', 'tab:red']):
        ax[0].plot(t, d['q'][:, i], color=c, lw=1, label=lab[i])
        ax[0].plot(t, d['qr'][:, i], color=c, lw=0.8, ls='--')
    ax[0].set_ylabel('Joint position'); ax[0].legend(ncol=3, loc='upper right', framealpha=0.95); ax[0].set_title('(a) Joint positions (solid) and references (dashed), FOSMC')
    tmax = np.array([600, 60, 30])
    for i, c in enumerate(['tab:orange', 'tab:blue', 'tab:red']):
        ax[1].plot(t, d['tau'][:, i] / tmax[i], color=c, lw=0.6, label=[r'$f_d/600$', r'$\tau_1/60$', r'$\tau_2/30$'][i])
    ax[1].set_ylabel('Normalised effort'); ax[1].legend(ncol=3, loc='upper right', framealpha=0.95); ax[1].set_ylim(-1.1, 1.1)
    ax[1].set_title('(b) Actuator commands (normalised by saturation limits)')
    rth = np.array([60, 8, 4])
    for i, c in enumerate(['tab:orange', 'tab:blue', 'tab:red']):
        ax[2].plot(t, np.abs(d['r'][:, i]) / rth[i], color=c, lw=0.8, label=[r'$|r_d|/r_{\mathrm{th},d}$', r'$|r_1|/r_{\mathrm{th},1}$', r'$|r_2|/r_{\mathrm{th},2}$'][i])
    ax[2].axhline(1, color='k', lw=0.6, ls='--'); ax[2].set_yscale('log'); ax[2].set_ylim(1e-3, 1e2)
    if det > 0:
        for a in ax:
            a.axvline(det, color='k', lw=0.6, ls=':')
        ax[2].text(det + 0.08, 3, 'Lock isolated at $t$ = %.3f s' % det, fontsize=7)
    ax[2].set_ylabel('Normalised residual'); ax[2].set_xlabel('Time (s)'); ax[2].legend(ncol=3, loc='upper right', framealpha=0.95)
    ax[2].set_title('(c) Generalised-momentum residuals')
    for a in ax:
        shade(a, 'S3')
    save(fig, 'fig_s3_internals')


# ------------------------------------------------------------------ Fig: ablation
def fig_ablation():
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.6))
    for fn, lab, sty in (('rep_S3_FOSMC.npz', 'FOSMC + fault-aware null space + reconfiguration', dict(color='tab:red')),
                         ('rep_S3_FOSMC_noreconf.npz', 'Without reconfiguration', dict(color='k', ls='--')),
                         ('rep_S3_FOSMC_weaknull.npz', 'Weak null-space objective', dict(color='tab:purple', ls='-.'))):
        d = np.load(f'{R}/{fn}')
        err = np.linalg.norm(d['x'] - d['xd'], axis=1) * 1e3
        err = np.where(np.isfinite(err), err, np.nan)
        ax[0].semilogy(d['t'], err + 1e-4, lw=0.9, label=lab, **sty)
        ax[1].plot(d['t'], np.rad2deg(d['q'][:, 2]), lw=0.9, label=lab, **sty)
    ax[0].set_ylim(1e-2, 1e3); ax[0].set_xlabel('Time (s)'); ax[0].set_ylabel('Task error (mm)'); ax[0].legend(fontsize=6)
    ax[0].axvline(4, color='r', lw=0.6); ax[0].set_title('(a) Cartesian error, S3')
    ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel(r'$\theta_2$ (deg)'); ax[1].axvline(4, color='r', lw=0.6)
    ax[1].set_title(r'(b) Elbow angle before/after lock'); ax[1].set_ylim(60, 140)
    save(fig, 'fig_ablation')


# ------------------------------------------------------------------ Fig: Monte Carlo box plots
def fig_mc():
    from matplotlib.ticker import LogLocator, FuncFormatter
    mc = json.load(open(f'{R}/mc.json'))
    fig, ax = plt.subplots(1, 4, figsize=(7.2, 2.7))
    fmt = FuncFormatter(lambda v, _: ('%g' % v).replace('-', '\u2212'))
    for j, (sc, key, lab, lg) in enumerate((('S1', 'rmse', 'RMSE (mm)', True), ('S2', 'rmse', 'RMSE (mm)', True),
                                            ('S3', 'rmse', 'RMSE (mm)', False), ('S2', 'tv', r'TV (N m s$^{-1}$)', False))):
        raw = [np.array([r[key] for r in mc[f'{sc}|{n}']], float) for n in NAMES]
        data = [d[np.isfinite(d)] for d in raw]
        bp = ax[j].boxplot(data, widths=0.6, patch_artist=True, showfliers=True, flierprops=dict(ms=3))
        for patch, n in zip(bp['boxes'], NAMES):
            patch.set_facecolor(STY[n]['color']); patch.set_alpha(0.45)
        if lg:
            ax[j].set_yscale('log'); ax[j].yaxis.set_major_formatter(fmt); ax[j].yaxis.set_minor_formatter(FuncFormatter(lambda v, _: ''))
        for k, (d, dr) in enumerate(zip(data, raw)):
            nd = int(np.sum(~np.isfinite(dr)))
            if nd:
                ax[j].text(k + 1, np.nanmax(d) * (1.25 if lg else 1.02), '%d diverged' % nd, ha='center', fontsize=6.5, color='tab:red')
        ax[j].set_xticks(range(1, len(NAMES) + 1)); ax[j].set_xticklabels([LAB[n] for n in NAMES], rotation=35, fontsize=6.5)
        ax[j].set_title(f'{sc}: {lab}', fontsize=8)
    save(fig, 'fig_mc')


# ------------------------------------------------------------------ Fig: sensitivity (tuning seeds) + memory
def fig_sens():
    tune = [r for r in json.load(open('results/tuning.json')) if r['kind'] == 'SMC' and np.isfinite(r['rmse'])]
    lams = sorted(set(r['lam'] for r in tune)); L2s = sorted(set(r['L2'] for r in tune))
    fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.5))
    mk = {0.3: 'v', 0.5: 's', 0.7: 'o', 0.85: '^', 1.0: 'D'}
    for lam in lams:
        rr = sorted([r for r in tune if r['lam'] == lam], key=lambda r: r['L2'])
        ax[0].semilogx([r['L2'] for r in rr], [r['rmse'] for r in rr], marker=mk.get(lam, 'o'), ms=3.5, lw=0.9,
                       label=r'$\lambda=%.2g$' % lam + (' (ISMC)' if lam == 1 else ''))
        ax[1].scatter([r['tv'] for r in rr], [r['rmse'] for r in rr], marker=mk.get(lam, 'o'), s=14, label=r'$\lambda=%.2g$' % lam)
    ax[0].set_xlabel(r'$\Lambda_2$'); ax[0].set_ylabel('RMSE (mm)'); ax[0].legend(fontsize=6); ax[0].set_title(r'(a) Gain sensitivity (S2)')
    ax[1].set_xscale('log'); ax[1].set_xlabel(r'TV (N m s$^{-1}$)'); ax[1].set_ylabel('RMSE (mm)'); ax[1].set_title('(b) Accuracy vs. activity')
    ax[1].axvline(json.load(open('results/selection.json'))['budget'], color='k', ls='--', lw=0.6)
    mem = json.load(open(f'{R}/memory.json'))
    Ls = sorted(float(k) for k in mem)
    mu = [np.mean([r['rmse'] for r in mem[str(L) if str(L) in mem else repr(L)]]) for L in Ls]
    sd = [np.std([r['rmse'] for r in mem[str(L) if str(L) in mem else repr(L)]]) for L in Ls]
    ax[2].errorbar(Ls, mu, yerr=sd, marker='o', ms=3.5, capsize=2, color='tab:red')
    ax[2].set_xscale('log'); ax[2].set_xlabel('Memory length $L$ (s)'); ax[2].set_ylabel('RMSE (mm)')
    ax[2].set_title('(c) GL memory length')
    save(fig, 'fig_sensitivity')


# ------------------------------------------------------------------ Fig: delay robustness
def fig_delay():
    sw = json.load(open(f'{R}/delay_sweep.json'))
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.7))
    _sel = json.load(open(f'{R}/selection.json'))
    _fo = (_sel['FOSMC']['lam'], _sel['FOSMC']['L2']); _ib = (1.0, _sel['ISMC-B']['L2'])
    cfgs = [(_ib[0], _ib[1], 'ISMC-B'), (_fo[0], _fo[1], 'FOSMC'), (1.0, _fo[1], 'ISMC-A')]
    cfgs += [c for c in ((0.5, 400.0, None), (0.3, 200.0, None)) if (c[0], c[1]) not in [(x[0], x[1]) for x in cfgs]]
    for lam, L2, nm in cfgs:
        rr = sorted([r for r in sw if r['lam'] == lam and r['L2'] == L2], key=lambda r: r['delay'])
        y = np.array([r['rmse'] for r in rr], float); y[~np.isfinite(y)] = 1e4
        lab = (nm + ': ' if nm else '') + r'$\lambda=%.1f,\ \Lambda_2=%d$' % (lam, L2)
        ax[0].semilogy([1e3 * r['delay'] for r in rr], np.minimum(y, 1e4), marker='o', ms=3, lw=0.9, label=lab)
    ax[0].axhline(2.0, color='k', ls='--', lw=0.6); ax[0].set_ylim(0.005, 1.2e4)
    ax[0].set_xlabel('Input delay (ms)'); ax[0].set_ylabel('RMSE (mm)'); ax[0].legend(fontsize=5.8)
    ax[0].set_title('(a) Degradation with actuation delay (S2)')
    # Pareto: nominal RMSE vs max tolerable delay (RMSE <= 2 mm)
    mk = {0.3: 'v', 0.5: 's', 0.7: 'o', 1.0: 'D'}
    for lam in sorted(set(r['lam'] for r in sw)):
        xs, ys = [], []
        for L2 in sorted(set(r['L2'] for r in sw)):
            rr = sorted([r for r in sw if r['lam'] == lam and r['L2'] == L2], key=lambda r: r['delay'])
            nom = rr[0]['rmse']
            ok = [r['delay'] for r in rr if np.isfinite(r['rmse']) and r['rmse'] <= 2.0]
            xs.append(nom); ys.append(1e3 * max(ok) if ok else 0)
        ax[1].plot(xs, ys, marker=mk[lam], lw=0.8, ms=4, label=r'$\lambda=%.1f$' % lam + (' (integer)' if lam == 1 else ''))
    ax[1].set_xlabel('Nominal RMSE (mm), no delay'); ax[1].set_ylabel('Max. tolerable delay (ms)')
    ax[1].set_title(r'(b) Accuracy–robustness trade-off'); ax[1].legend(fontsize=6)
    save(fig, 'fig_delay')


if __name__ == '__main__':
    import sys
    which = sys.argv[1:] or ['s3', 'int', 'abl', 'mc', 'sens', 'delay']
    if 's3' in which: fig_s3()
    if 'int' in which: fig_internals()
    if 'abl' in which: fig_ablation()
    if 'mc' in which: fig_mc()
    if 'sens' in which: fig_sens()
    if 'delay' in which: fig_delay()
    print('figures ok')
