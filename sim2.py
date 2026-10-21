import numpy as np
from math import gamma

NOMINAL = dict(
    m0=30.0,
    m1=6.0, l1=0.400, lc1=0.20, I1=0.080,
    m2=3.0, l2=0.250, lc2=0.13, I2=0.020,
    fv=np.array([40.0, 2.0, 1.0]),
    fc=np.array([15.0, 1.5, 0.8]),
    tau_max=np.array([600.0, 60.0, 30.0]),
    q_min=np.array([-0.90, -np.deg2rad(140), -np.deg2rad(150)]),
    q_max=np.array([0.90, np.deg2rad(140), np.deg2rad(150)]),
)


def with_payload(p, mp):
    p = dict(p)
    m2, lc2, I2, l2 = p['m2'], p['lc2'], p['I2'], p['l2']
    m2n = m2 + mp
    lc2n = (m2 * lc2 + mp * l2) / m2n
    I2n = I2 + m2 * (lc2 - lc2n) ** 2 + mp * (l2 - lc2n) ** 2
    p.update(m2=m2n, lc2=lc2n, I2=I2n)
    return p


def M_mat(q, p):
    s1, s12, c2 = np.sin(q[1]), np.sin(q[1] + q[2]), np.cos(q[2])
    m0, m1, m2, l1, lc1, lc2, I1, I2 = (p[k] for k in ('m0', 'm1', 'm2', 'l1', 'lc1', 'lc2', 'I1', 'I2'))
    a = -(l1 * m2 + lc1 * m1) * s1 - lc2 * m2 * s12
    b = -lc2 * m2 * s12
    m22 = I1 + I2 + lc1 ** 2 * m1 + m2 * (l1 ** 2 + 2 * l1 * lc2 * c2 + lc2 ** 2)
    m23 = I2 + lc2 * m2 * (l1 * c2 + lc2)
    m33 = I2 + lc2 ** 2 * m2
    return np.array([[m0 + m1 + m2, a, b], [a, m22, m23], [b, m23, m33]])


def C_mat(q, v, p):
    c1, c12, s2 = np.cos(q[1]), np.cos(q[1] + q[2]), np.sin(q[2])
    m1, m2, l1, lc1, lc2 = (p[k] for k in ('m1', 'm2', 'l1', 'lc1', 'lc2'))
    h = l1 * lc2 * m2 * s2
    return np.array([
        [0.0, -(l1 * m2 + lc1 * m1) * v[1] * c1 - lc2 * m2 * (v[1] + v[2]) * c12, -lc2 * m2 * (v[1] + v[2]) * c12],
        [0.0, -h * v[2], -h * (v[1] + v[2])],
        [0.0, h * v[1], 0.0]])


def friction(v, fv, fc, eps=0.01):
    return fv * v + fc * np.tanh(v / eps)


def fkine(q, p):
    return np.array([q[0] + p['l1'] * np.cos(q[1]) + p['l2'] * np.cos(q[1] + q[2]),
                     p['l1'] * np.sin(q[1]) + p['l2'] * np.sin(q[1] + q[2])])


def jacobian(q, p):
    s1, s12 = np.sin(q[1]), np.sin(q[1] + q[2])
    c1, c12 = np.cos(q[1]), np.cos(q[1] + q[2])
    return np.array([[1.0, -p['l1'] * s1 - p['l2'] * s12, -p['l2'] * s12],
                     [0.0, p['l1'] * c1 + p['l2'] * c12, p['l2'] * c12]])


TASK = dict(xc=0.0, yc=0.36, A=0.55, B=0.06, T=4.0, t_ramp=1.0)


def _phase(t, task):
    W = 2 * np.pi / task['T']
    tr = task['t_ramp']
    if t < tr:
        s = t / tr
        r = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        rd = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / tr
        R = tr * (2.5 * s ** 4 - 3 * s ** 5 + s ** 6)
        return W * R, W * r, W * rd
    R_end = tr * (2.5 - 3 + 1)
    return W * (R_end + (t - tr)), W, 0.0


def task_ref(t, task=TASK):
    ph, phd, phdd = _phase(t, task)
    A, B = task['A'], task['B']
    x = np.array([task['xc'] + A * np.sin(ph), task['yc'] + B * np.sin(2 * ph)])
    dx_dph = np.array([A * np.cos(ph), 2 * B * np.cos(2 * ph)])
    d2x_dph2 = np.array([-A * np.sin(ph), -4 * B * np.sin(2 * ph)])
    xd = dx_dph * phd
    xdd = d2x_dph2 * phd ** 2 + dx_dph * phdd
    return x, xd, xdd


class RefGenerator:
    def __init__(self, p, q0, dt, W=np.array([1.0, 1.0, 1.0]), Kp=400.0, Kv=40.0,
                 kn=100.0, q2_star=np.pi / 2, task=TASK):
        self.p, self.dt, self.W, self.Kp, self.Kv, self.kn = p, dt, W, Kp, Kv, kn
        self.q2_star, self.task = q2_star, task
        self.q = q0.copy(); self.qd = np.zeros(3); self.qdd = np.zeros(3)
        self.active = np.ones(3, bool)

    def lock(self, i, q_meas):
        self.active[i] = False
        self.q = q_meas.copy(); self.qd[:] = 0.0; self.qdd[:] = 0.0

    def step(self, t):
        x, xd, xdd = task_ref(t, self.task)
        q, qd, p, a = self.q, self.qd, self.p, self.active
        J = jacobian(q, p)
        s1, s12, c1, c12 = np.sin(q[1]), np.sin(q[1] + q[2]), np.cos(q[1]), np.cos(q[1] + q[2])
        w1, w12 = qd[1], qd[1] + qd[2]
        Jdqd = np.array([-p['l1'] * c1 * w1 ** 2 - p['l2'] * c12 * w12 ** 2,
                         -p['l1'] * s1 * w1 ** 2 - p['l2'] * s12 * w12 ** 2])
        e = x - fkine(q, p); ed = xd - J @ qd
        u = xdd + self.Kv * ed + self.Kp * e - Jdqd
        Ja = J[:, a]; Winv = np.diag(1.0 / self.W[a])
        JW = Winv @ Ja.T @ np.linalg.inv(Ja @ Winv @ Ja.T + 1e-6 * np.eye(2))
        qdd_a = JW @ u
        if a.sum() > 2:
            grad = np.zeros(3); grad[2] = (q[2] - self.q2_star)
            z = -self.kn * grad[a] - 2 * np.sqrt(self.kn) * qd[a]
            qdd_a += (np.eye(a.sum()) - JW @ Ja) @ z
        self.qdd = np.zeros(3); self.qdd[a] = qdd_a
        out = (self.q.copy(), self.qd.copy(), self.qdd.copy())
        self.q = self.q + self.qd * self.dt + 0.5 * self.qdd * self.dt ** 2
        self.qd = self.qd + self.qdd * self.dt
        return out


class GLDerivative:
    def __init__(self, alpha, dt, L_sec, n=3):
        self.alpha, self.dt = alpha, dt
        self.N = max(2, int(round(L_sec / dt)))
        w = np.empty(self.N); w[0] = 1.0
        for j in range(1, self.N):
            w[j] = w[j - 1] * (1.0 - (alpha + 1.0) / j)
        self.w = w
        self.buf = np.zeros((self.N, n)); self.k = 0
        self.h = dt ** (-alpha)

    def reset(self):
        self.buf[:] = 0.0; self.k = 0

    def __call__(self, x):
        self.k = (self.k - 1) % self.N
        self.buf[self.k] = x
        idx = (self.k + np.arange(self.N)) % self.N
        return self.h * (self.w @ self.buf[idx])


class SlidingController:
    """Sliding-mode law with (fractional) integral surface. aw in {False/'none', True/'cond', 'backcalc'}."""

    def __init__(self, p_hat, dt, lam=0.6, L1=20.0, L2=100.0, KD=(800., 80., 30.),
                 rho=(40., 6., 3.), phi=0.05, robust=True, L_mem=1.0, name='', antiwindup=False, kb=10.0):
        self.p, self.dt, self.lam = p_hat, dt, lam
        self.L1 = L1 * np.ones(3); self.L2 = L2 * np.ones(3)
        self.KD = np.array(KD) if robust else np.zeros(3)
        self.rho = np.array(rho) if robust else np.zeros(3)
        self.phi, self.robust, self.name = phi, robust, name
        self.D = None if abs(1 - lam) < 1e-12 else GLDerivative(1 - lam, dt, L_mem)
        self.Ie = np.zeros(3)
        self.enabled = np.ones(3, bool)
        self.aw = 'cond' if antiwindup is True else ('none' if antiwindup in (False, None) else antiwindup)
        self.kb = kb
        self.sat = np.zeros(3, bool); self.excess = np.zeros(3)
        self.a_r = np.zeros(3); self.v_r = np.zeros(3)

    def reset_memory(self):
        self.Ie[:] = 0.0
        if self.D is not None:
            self.D.reset()

    def __call__(self, q, qd, q_r, qd_r, qdd_r):
        e, ed = q - q_r, qd - qd_r
        De = e if self.D is None else self.D(e)
        if self.aw == 'cond':
            self.Ie += np.where(self.sat, 0.0, De * self.dt)
        elif self.aw == 'backcalc':
            gain = np.maximum(self.KD * self.L2, 1e-9)
            self.Ie += (De - self.kb * self.excess / gain) * self.dt
        else:
            self.Ie += De * self.dt
        v_r = qd_r - self.L1 * e - self.L2 * self.Ie
        a_r = qdd_r - self.L1 * ed - self.L2 * De
        self.a_r, self.v_r = a_r, v_r
        s = qd - v_r
        M, C = M_mat(q, self.p), C_mat(q, qd, self.p)
        tau_u = M @ a_r + C @ v_r + friction(qd, self.p['fv'], self.p['fc']) \
            - self.KD * s - self.rho * np.clip(s / self.phi, -1, 1)
        self.sat = np.abs(tau_u) >= self.p['tau_max']
        tau = np.clip(tau_u, -self.p['tau_max'], self.p['tau_max'])
        self.excess = tau - tau_u
        tau[~self.enabled] = 0.0
        return tau, s


class CTPID:
    def __init__(self, p_hat, dt, omega=20.0, name='CT-PID'):
        self.p, self.dt, self.name = p_hat, dt, name
        self.Kv, self.Kp, self.Ki = 3 * omega, 3 * omega ** 2, omega ** 3
        self.Ie = np.zeros(3); self.enabled = np.ones(3, bool)

    def reset_memory(self):
        self.Ie[:] = 0.0

    def __call__(self, q, qd, q_r, qd_r, qdd_r):
        e, ed = q - q_r, qd - qd_r
        self.Ie += e * self.dt
        a = qdd_r - self.Kv * ed - self.Kp * e - self.Ki * self.Ie
        tau = M_mat(q, self.p) @ a + C_mat(q, qd, self.p) @ qd + friction(qd, self.p['fv'], self.p['fc'])
        tau = np.clip(tau, -self.p['tau_max'], self.p['tau_max'])
        tau[~self.enabled] = 0.0
        return tau, ed + self.Kv * e


class MomentumObserver:
    def __init__(self, p_hat, dt, KO=50.0):
        self.p, self.dt, self.KO = p_hat, dt, KO
        self.integ = None; self.r = np.zeros(3); self.p0 = None

    def __call__(self, q, qd, tau):
        M, C = M_mat(q, self.p), C_mat(q, qd, self.p)
        pm = M @ qd
        if self.p0 is None:
            self.p0 = pm.copy(); self.integ = np.zeros(3)
        beta = -C.T @ qd
        self.integ += (tau - beta - friction(qd, self.p['fv'], self.p['fc']) + self.r) * self.dt
        self.r = self.KO * (pm - self.integ - self.p0)
        return self.r.copy()


def solve3(A, b):
    (a, b_, c), (d, e, f), (g, h, i) = A.tolist()
    x, y, z = b.tolist()
    co0, co1, co2 = e * i - f * h, f * g - d * i, d * h - e * g
    det = a * co0 + b_ * co1 + c * co2
    r0 = (x * co0 + b_ * (f * z - y * i) + c * (y * h - e * z)) / det
    r1 = (a * (y * i - f * z) + x * co1 + c * (d * z - y * g)) / det
    r2 = (a * (e * z - y * h) + b_ * (y * g - d * z) + x * co2) / det
    return np.array([r0, r1, r2])


class Plant:
    def __init__(self, p_true, q0, dist_fn=None):
        self.p = p_true; self.q = q0.copy(); self.qd = np.zeros(3)
        self.free = np.ones(3, bool); self.eff = np.ones(3); self.bias = np.zeros(3)
        self.dist_fn = dist_fn or (lambda t: np.zeros(3))

    def lock_joint(self, i):
        self.free[i] = False; self.qd[i] = 0.0

    def _acc(self, t, q, qd, tau):
        p, f = self.p, self.free
        M, C = M_mat(q, p), C_mat(q, qd, p)
        rhs = self.eff * tau + self.bias - C @ qd - friction(qd, p['fv'], p['fc']) - self.dist_fn(t)
        if f.all():
            return solve3(M, rhs)
        idx = np.flatnonzero(f)
        a = np.zeros(3)
        if len(idx) == 2:
            i, j = idx
            a11, a12, a22 = M[i, i], M[i, j], M[j, j]
            det = a11 * a22 - a12 * a12
            a[i] = (a22 * rhs[i] - a12 * rhs[j]) / det
            a[j] = (a11 * rhs[j] - a12 * rhs[i]) / det
        else:
            a[idx] = np.linalg.solve(M[np.ix_(idx, idx)], rhs[idx])
        return a

    def step(self, t, tau, dt, sub=2):
        h = dt / sub
        for k in range(sub):
            tk = t + k * h
            q, v = self.q, self.qd
            a1 = self._acc(tk, q, v, tau); v1 = v
            a2 = self._acc(tk + h / 2, q + h / 2 * v1, v + h / 2 * a1, tau); v2 = v + h / 2 * a1
            a3 = self._acc(tk + h / 2, q + h / 2 * v2, v + h / 2 * a2, tau); v3 = v + h / 2 * a2
            a4 = self._acc(tk + h, q + h * v3, v + h * a3, tau); v4 = v + h * a3
            self.q = q + h / 6 * (v1 + 2 * v2 + 2 * v3 + v4)
            self.qd = v + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
            self.qd[~self.free] = 0.0


def initial_q(p, task=TASK):
    x0, _, _ = task_ref(0.0, task)
    q2 = np.pi / 2
    R = np.hypot(p['l1'] + p['l2'] * np.cos(q2), p['l2'] * np.sin(q2))
    beta = np.arctan2(p['l2'] * np.sin(q2), p['l1'] + p['l2'] * np.cos(q2))
    q1 = np.arcsin(x0[1] / R) - beta
    d = x0[0] - R * np.cos(q1 + beta)
    return np.array([d, q1, q2])


def run(ctrl_kind='FOSMC', lam=0.6, scenario='S1', seed=0, Tf=10.0, dt=1e-3,
        reconfigure=True, L_mem=1.0, log=False, p_nom=NOMINAL, L2=100.0, L1=20.0, kn=100.0,
        W=(1.0, 1.0, 1.0), omega=20.0, delay=0.0, antiwindup=False, noise=True, p_true=None,
        fdi='generic', rth_scale=1.0, reset_on_reconf=True, b1=0.5, loe_ramp=0.0, tau_scale=1.0,
        act_tc=0.0, enc_res=0.0, unc_scale=1.0, log_delta=False, kb=10.0, t_lock=4.0, rho=(40., 6., 3.),
        gains_post=None):
    """fdi='legacy' reproduces the submitted version (only joint 3 monitored, only in S3).
    fdi='generic' monitors every free joint in every scenario; each declaration disables the joint.
    gains_post=(L1, L2): optional gain schedule applied at reconfiguration (default None: gains unchanged)."""
    rng = np.random.default_rng(seed)
    pt = dict(p_nom)
    for k in (() if p_true is not None else ('m0', 'm1', 'm2', 'I1', 'I2')):
        pt[k] = p_nom[k] * (1 + rng.uniform(-0.15, 0.15) * unc_scale)
    if p_true is None:
        pt['fv'] = p_nom['fv'] * (1 + rng.uniform(-0.3, 0.3, 3) * unc_scale)
        pt['fc'] = p_nom['fc'] * (1 + rng.uniform(-0.3, 0.3, 3) * unc_scale)
        pt = with_payload(pt, rng.uniform(0.0, 3.0))
    else:
        pt = p_true
    ph = dict(p_nom); ph['tau_max'] = p_nom['tau_max'] * tau_scale

    def dist(t):
        base = np.array([10.0 * np.sin(2 * np.pi * 1.5 * t), 1.0 * np.sin(2 * np.pi * 2.0 * t), 0.5 * np.cos(2 * np.pi * 2.5 * t)])
        step = np.array([25.0, 2.0, 1.0]) if t >= 6.0 else np.zeros(3)
        return base + step

    q0 = initial_q(p_nom)
    plant = Plant(pt, q0, dist)
    ref = RefGenerator(ph, q0, dt, W=np.array(W), kn=kn)
    if ctrl_kind == 'CTPID':
        ctrl = CTPID(ph, dt, omega=omega)
    elif ctrl_kind == 'ISMC':
        ctrl = SlidingController(ph, dt, lam=1.0, L1=L1, L2=L2, robust=True, name='ISMC', antiwindup=antiwindup, kb=kb, rho=rho)
    else:
        ctrl = SlidingController(ph, dt, lam=lam, L1=L1, L2=L2, robust=True, L_mem=L_mem, name=f'FOSMC({lam})', antiwindup=antiwindup, kb=kb, rho=rho)
    obs = MomentumObserver(ph, dt)

    t_loe, lock_joint = 3.0, 2
    r_th = np.array([60.0, 8.0, 4.0]) * rth_scale; v_th = 0.02; persist = int(0.02 / dt)
    cnt = np.zeros(3, int); detected_at = None; alarms = []
    declared = np.zeros(3, bool)

    n = int(round(Tf / dt))
    out = {k: np.zeros((n, m)) for k, m in (('q', 3), ('qr', 3), ('x', 2), ('xd', 2), ('tau', 3), ('r', 3), ('s', 3), ('flags', 3), ('sat', 3), ('Ie', 3))}
    out['t'] = np.arange(n) * dt
    if log_delta:
        out['Delta'] = np.full((n, 3), np.nan)
    sig_q, sig_v = np.array([2e-6, 2e-5, 2e-5]), np.array([1e-3, 2e-3, 2e-3])
    tau = np.zeros(3); tau_act = np.zeros(3)
    nd = int(round(delay / dt)); tau_buf = [np.zeros(3)] * nd
    for k in range(n):
        t = k * dt
        if scenario in ('S2', 'S3') and t >= t_loe:
            if loe_ramp > 0:
                plant.eff[1] = 1.0 - (1.0 - b1) * min(1.0, (t - t_loe) / loe_ramp)
            else:
                plant.eff[1] = b1
            plant.bias[1] = 3.0
        if scenario == 'S3' and t >= t_lock and plant.free[lock_joint]:
            plant.lock_joint(lock_joint)
        if noise:
            qm = plant.q + sig_q * rng.standard_normal(3)
            vm = plant.qd + sig_v * rng.standard_normal(3)
        else:
            qm, vm = plant.q.copy(), plant.qd.copy()
        if enc_res > 0:
            res = np.array([1e-6, enc_res, enc_res])
            qm = res * np.round(qm / res)
        r = obs(qm, vm, tau)
        if fdi == 'legacy':
            if detected_at is None:
                cond = (np.abs(r) > r_th) & (np.abs(vm) < v_th) & (np.abs(ref.qd) > 0.05)
                cnt = np.where(cond, cnt + 1, 0)
                if cnt[lock_joint] >= persist and scenario == 'S3':
                    detected_at = t; alarms.append((t, lock_joint))
                    if reconfigure:
                        ref.lock(lock_joint, qm); ctrl.enabled[lock_joint] = False
                        if reset_on_reconf: ctrl.reset_memory()
        else:
            cond = (np.abs(r) > r_th) & (np.abs(vm) < v_th) & (np.abs(ref.qd) > 0.05) & (~declared)
            cnt = np.where(cond, cnt + 1, 0)
            for j in range(3):
                if cnt[j] >= persist and not declared[j]:
                    declared[j] = True; alarms.append((t, j))
                    if detected_at is None: detected_at = t
                    if reconfigure:
                        ref.lock(j, qm); ctrl.enabled[j] = False
                        if reset_on_reconf: ctrl.reset_memory()
                        if gains_post is not None and hasattr(ctrl, 'L2'):
                            ctrl.L1, ctrl.L2 = float(gains_post[0]), float(gains_post[1])
        q_r, qd_r, qdd_r = ref.step(t)
        tau, s = ctrl(qm, vm, q_r, qd_r, qdd_r)
        out['q'][k] = plant.q; out['qr'][k] = q_r; out['x'][k] = fkine(plant.q, pt)
        out['xd'][k] = task_ref(t)[0]; out['tau'][k] = tau; out['r'][k] = r; out['s'][k] = s
        if hasattr(ctrl, 'sat'):
            out['sat'][k] = ctrl.sat
        if hasattr(ctrl, 'Ie'):
            out['Ie'][k] = ctrl.Ie
        out['flags'][k] = [float(plant.eff[1] < 1), float(not plant.free[lock_joint]), float(detected_at is not None and reconfigure)]
        if log_delta and ctrl_kind != 'CTPID':
            qt, vt = plant.q, plant.qd
            Mh, Ch = M_mat(qt, ph), C_mat(qt, vt, ph); Mt, Ct = M_mat(qt, pt), C_mat(qt, vt, pt)
            Fh, Ft = friction(vt, ph['fv'], ph['fc']), friction(vt, pt['fv'], pt['fc'])
            ar, vr = ctrl.a_r, ctrl.v_r
            D = (plant.eff - 1.0) * (Mh @ ar + Ch @ vr + Fh) + (Mh - Mt) @ ar + (Ch - Ct) @ vr + (Fh - Ft) + plant.bias - dist(t)
            D[~plant.free] = np.nan
            out['Delta'][k] = D
        tau_in = tau
        tau_buf.append(tau.copy()); tau_app = tau_buf.pop(0)
        if act_tc > 0:
            tau_act = tau_act + (dt / (act_tc + dt)) * (tau_app - tau_act); tau_app = tau_act
        plant.step(t, tau_app, dt)
        if (not np.all(np.isfinite(plant.q))) or np.linalg.norm(out['x'][k] - out['xd'][k]) > 1.0:
            for key in ('q', 'qr', 'x', 'tau', 'r', 's'):
                out[key][k:] = np.nan
            out['diverged'] = True
            break
    out['detected_at'] = detected_at
    out['alarms'] = alarms
    out['params_true'] = pt
    return out


def metrics(o, t0=1.0):
    if o.get('diverged', False):
        return dict(rmse=np.nan, emax=np.nan, tv=np.full(3, np.nan), energy=np.full(3, np.nan))
    t = o['t']; m = t >= t0
    ex = np.linalg.norm(o['x'] - o['xd'], axis=1)[m] * 1e3
    tau = o['tau'][m]; dt = t[1] - t[0]
    tv = np.sum(np.abs(np.diff(tau, axis=0)), axis=0) / (t[m][-1] - t[m][0])
    E = np.sum(tau ** 2, axis=0) * dt
    return dict(rmse=np.sqrt(np.mean(ex ** 2)), emax=ex.max(), tv=tv, energy=E)


def summarize(o, scenario, t_lock=4.0):
    """Compact run summary used by all campaigns."""
    f = lambda v: None if (v is None or not np.isfinite(v)) else float(v)
    div = bool(o.get('diverged', False))
    m = metrics(o)
    t = o['t']; res = dict(diverged=div, rmse=f(m['rmse']), emax=f(m['emax']), tv=f(np.sum(m['tv'])))
    al = o['alarms']
    res['alarms'] = [(float(a), int(b)) for a, b in al]
    res['det'] = o['detected_at']
    if scenario == 'S3':
        res['fa'] = any((a < t_lock - 1e-9) or (b != 2) for a, b in al)       # alarm before the lock or on the wrong joint
        res['md'] = not any(b == 2 and a >= t_lock - 1e-9 for a, b in al)
        res['det_delay_ms'] = None if res['md'] else 1e3 * (min(a for a, b in al if b == 2 and a >= t_lock - 1e-9) - t_lock)
    else:
        res['fa'] = len(al) > 0
    if div:
        return res
    ex = np.linalg.norm(o['x'] - o['xd'], axis=1) * 1e3
    w = (t >= 4.0) & (t < 5.0); w2 = (t >= 4.0) & (t < 5.5)
    res['emax_post'] = f(ex[w].max()); res['rmse_ss'] = f(np.sqrt(np.mean(ex[t >= 6.5] ** 2)))
    res['rmse_pre'] = f(np.sqrt(np.mean(ex[(t >= 1.0) & (t < 3.0)] ** 2)))
    res['rmse_loe'] = f(np.sqrt(np.mean(ex[(t >= 3.2) & (t < 4.0)] ** 2)))
    sat = o['sat']
    res['sat_th1_post'] = f(sat[w2, 1].mean()); res['sat_all_post'] = f(sat[w2].any(axis=1).mean()); res['sat_th1_all'] = f(sat[t >= 1.0, 1].mean())
    e = o['q'] - o['qr']
    res['emax_joint_post'] = [f(np.nanmax(np.abs(e[w2, i]))) for i in range(3)]
    res['emax_joint_pre'] = [f(np.nanmax(np.abs(e[(t >= 1) & (t < 3), i]))) for i in range(3)]
    res['Ie_max_post'] = [f(np.nanmax(np.abs(o['Ie'][w2, i]))) for i in range(3)]
    s = np.abs(o['s'])
    res['s_max'] = [f(np.nanmax(s[t >= 1.0, i])) for i in range(3)]; res['s_max_post'] = [f(np.nanmax(s[w2, i])) for i in range(3)]
    if scenario == 'S3':
        for band in (1.0, 0.5):
            idx = np.where((t >= t_lock) & (ex > band))[0]
            res['rec_%g' % band] = None if (len(idx) and t[idx[-1]] > t[-1] - 1.0) else (0.0 if not len(idx) else float(t[idx[-1]] - t_lock))
    if 'Delta' in o:
        D = np.abs(o['Delta'])
        for name, lo, hi in (('pre', 1.0, 3.0), ('loe', 3.0, 4.0), ('post', 4.0, 5.5), ('late', 5.5, 10.0)):
            mm = (t >= lo) & (t < hi)
            res['Dmax_' + name] = [f(np.nanmax(D[mm, i])) if np.any(np.isfinite(D[mm, i])) else None for i in range(3)]
    return res
