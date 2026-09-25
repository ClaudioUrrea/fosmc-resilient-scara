"""Theory-related figures: (i) sliding-manifold error dynamics for several
fractional orders, (ii) weights of the GL operator and short-memory error,
(iii) kinematic schematic of the rail-SCARA chain."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from math import gamma
from sim import GLDerivative, NOMINAL, initial_q

import fontsetup
fontsetup.apply()


def sliding_response(lam, L1=20.0, L2=400.0, T=1.0, dt=1e-4, e0=1.0):
    """Solve e_dot + L1 e + L2 I^lam e = 0 (full-memory GL, explicit Euler)."""
    n = int(T / dt)
    e = np.zeros(n); e[0] = e0
    D = None if lam == 1 else GLDerivative(1 - lam, dt, T, n=1)
    I = 0.0
    for k in range(n - 1):
        De = e[k] if D is None else D(np.array([e[k]]))[0]
        I += De * dt
        e[k + 1] = e[k] + dt * (-L1 * e[k] - L2 * I)
    return np.arange(n) * dt, e


def fig_sliding(path):
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.6))
    for lam, ls in zip([0.3, 0.5, 0.7, 1.0], ['-', '--', '-.', ':']):
        t, e = sliding_response(lam)
        ax[0].plot(t, e, ls, label=r'$\lambda=%.1f$' % lam)
        ax[1].semilogy(t, np.abs(e) + 1e-12, ls, label=r'$\lambda=%.1f$' % lam)
    ax[0].set_xlabel('Time (s)'); ax[0].set_ylabel(r'$e(t)/e(0)$'); ax[0].legend(fontsize=7)
    ax[0].set_title('(a) Error on the sliding manifold')
    ax[1].set_xlabel('Time (s)'); ax[1].set_ylabel(r'$|e(t)|/e(0)$'); ax[1].set_ylim(1e-5, 2)
    ax[1].set_title('(b) Logarithmic scale')
    fig.tight_layout(); fig.savefig(path + '.pdf'); fig.savefig(path + '.png'); plt.close(fig)


def fig_gl(path):
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.5))
    for a, ls in zip([0.3, 0.5, 0.7], ['-', '--', '-.']):
        g = GLDerivative(a, 1e-3, 2.0, n=1)
        j = np.arange(1, g.N)
        ax[0].loglog(j, np.abs(g.w[1:]), ls, label=r'$\alpha=%.1f$' % a)
        L = np.logspace(-2, 0.5, 60)
        ax[1].loglog(L, L ** (-a) / abs(gamma(1 - a)), ls, label=r'$\alpha=%.1f$' % a)
    ax[0].set_xlabel('Lag index $j$'); ax[0].set_ylabel(r'$|w_j^{(\alpha)}|$'); ax[0].legend(fontsize=7)
    ax[0].set_title('(a) GL weights')
    ax[1].set_xlabel('Memory length $L$ (s)'); ax[1].set_ylabel(r'$\varepsilon_L/\bar{E}$')
    ax[1].set_title('(b) Short-memory error bound'); ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(path + '.pdf'); fig.savefig(path + '.png'); plt.close(fig)


def fig_schematic(path):
    p = NOMINAL
    q = initial_q(p); q[0] = 0.15; q[1] = 0.35; q[2] = np.pi / 2
    fig, ax = plt.subplots(figsize=(4.6, 3.0))
    ax.grid(False)
    ax.add_patch(plt.Rectangle((-0.9, -0.06), 1.8, 0.06, fc='0.75', ec='k', lw=0.6))
    ax.add_patch(plt.Rectangle((q[0] - 0.08, -0.03), 0.16, 0.05, fc='orange', ec='k', lw=0.6))
    P0 = np.array([q[0], 0.0])
    P1 = P0 + p['l1'] * np.array([np.cos(q[1]), np.sin(q[1])])
    P2 = P1 + p['l2'] * np.array([np.cos(q[1] + q[2]), np.sin(q[1] + q[2])])
    ax.plot(*zip(P0, P1), color='tab:blue', lw=6, solid_capstyle='round')
    ax.plot(*zip(P1, P2), color='tab:cyan', lw=5, solid_capstyle='round')
    for P in (P0, P1):
        ax.plot(*P, 'o', ms=7, mfc='w', mec='k')
    ax.plot(*P2, 'v', ms=7, color='k')
    th = np.linspace(0, 2 * np.pi, 400)
    ax.plot(0.55 * np.sin(th), 0.36 + 0.06 * np.sin(2 * th), 'g--', lw=1)
    ax.annotate('', xy=(q[0] + 0.25, -0.1), xytext=(q[0], -0.1), arrowprops=dict(arrowstyle='->'))
    ax.text(q[0] + 0.32, -0.115, r'$d$ (P)', ha='left')
    ax.text(P0[0] + 0.07, P0[1] + 0.02, r'$\theta_1$')
    ax.text(P1[0] + 0.03, P1[1] - 0.04, r'$\theta_2$')
    ax.text(P2[0] + 0.03, P2[1] + 0.035, r'$\mathbf{x}=(x,y)$', ha='left', bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.9))
    ax.text(-0.52, 0.47, 'Dispensing contour', color='g', fontsize=8)
    ax.text(-0.88, -0.14, 'Linear rail', fontsize=8)
    ax.set_aspect('equal'); ax.set_xlim(-0.95, 0.95); ax.set_ylim(-0.18, 0.52)
    ax.set_xlabel('$x$ (m)'); ax.set_ylabel('$y$ (m)')
    fig.tight_layout(); fig.savefig(path + '.pdf'); fig.savefig(path + '.png'); plt.close(fig)


if __name__ == '__main__':
    import os
    os.makedirs('figures', exist_ok=True)
    fig_sliding('figures/fig_sliding')
    fig_gl('figures/fig_gl')
    fig_schematic('figures/fig_schematic')
    print('done')
