function out = fosmc_sim(opts)
%FOSMC_SIM Independent MATLAB implementation of the rail-mounted redundant SCARA
%   benchmark (P-R-R chain): fault-aware CLIK, fractional-order sliding-mode
%   control with short-memory Grunwald-Letnikov operator, generalized-momentum
%   FDI, loss of effectiveness, elbow lock and bumpless reconfiguration.
%
%   out = fosmc_sim(opts)   with fields (defaults in brackets):
%     .lam [0.7]  fractional order (1 -> integer ISMC)
%     .L2 [1600]  integral gain     .L1 [20]
%     .scenario ['S3']  'S1' | 'S2' | 'S3'
%     .Tf [10]  .dt [1e-3]  .Lmem [1.0]  .reconfigure [true]  .antiwindup [false]
%     .noise [false]  .seed [0]  (MATLAB random stream differs from NumPy)
%     .ptrue [struct]  true plant parameters (default: nominal, 1.5-kg payload)
%
%   Runs unchanged in MATLAB (R2016b or later) and GNU Octave (>= 6).
%   Author: C. Urrea.

if nargin < 1, opts = struct(); end
o = defaults(opts, struct('lam', 0.7, 'L2', 1600, 'L1', 20, 'scenario', 'S3', 'Tf', 10, ...
    'dt', 1e-3, 'Lmem', 1.0, 'reconfigure', true, 'antiwindup', false, 'noise', false, ...
    'seed', 0, 'ptrue', []));
pn = nominal_params();
if isempty(o.ptrue)
    pt = pn; pt.fv = 1.2 * pn.fv; pt.fc = 1.2 * pn.fc; pt = with_payload(pt, 1.5);
else
    pt = o.ptrue;
end
dt = o.dt; n = round(o.Tf / dt);

% ---------------- controller settings (identical for ISMC and FOSMC)
KD = [800; 80; 30]; rho = [40; 6; 3]; phi = 0.05;
L1 = o.L1 * ones(3, 1); L2 = o.L2 * ones(3, 1);
alpha = 1 - o.lam; useGL = abs(alpha) > 1e-12;
N = max(2, round(o.Lmem / dt));
w = zeros(N, 1); w(1) = 1;
for j = 2:N, w(j) = w(j-1) * (1 - (alpha + 1) / (j - 1)); end
hgl = dt^(-alpha); buf = zeros(N, 3);
Ie = zeros(3, 1); enabled = true(3, 1); satf = false(3, 1);

% ---------------- CLIK state
q0 = initial_q(pn);
cl.q = q0; cl.qd = zeros(3, 1); cl.active = true(3, 1);
cl.W = [1; 1; 1]; cl.Kp = 400; cl.Kv = 40; cl.kn = 100; cl.q2s = pi / 2;

% ---------------- plant / observer / FDI
pl.q = q0; pl.qd = zeros(3, 1); pl.free = true(3, 1); pl.eff = ones(3, 1); pl.bias = zeros(3, 1);
KO = 50; r = zeros(3, 1); integ = zeros(3, 1); p0 = []; 
r_th = [60; 8; 4]; v_th = 0.02; persist = round(0.02 / dt); cnt = zeros(3, 1); detected = -1;
sig_q = [2e-6; 2e-5; 2e-5]; sig_v = [1e-3; 2e-3; 2e-3];
if o.noise, rng(o.seed); end

out.t = (0:n-1)' * dt; out.q = zeros(n, 3); out.qr = zeros(n, 3); out.x = zeros(n, 2);
out.xd = zeros(n, 2); out.tau = zeros(n, 3); out.r = zeros(n, 3);
tau = zeros(3, 1);
for k = 1:n
    t = (k - 1) * dt;
    if any(strcmp(o.scenario, {'S2', 'S3'})) && t >= 3.0
        pl.eff(2) = 0.5; pl.bias(2) = 3.0;
    end
    if strcmp(o.scenario, 'S3') && t >= 4.0 && pl.free(3)
        pl.free(3) = false; pl.qd(3) = 0;
    end
    if o.noise
        qm = pl.q + sig_q .* randn(3, 1); vm = pl.qd + sig_v .* randn(3, 1);
    else
        qm = pl.q; vm = pl.qd;
    end
    % ---- generalized-momentum observer
    M = M_mat(qm, pn); C = C_mat(qm, vm, pn); pm = M * vm;
    if isempty(p0), p0 = pm; end
    integ = integ + (tau + C' * vm - friction(vm, pn) + r) * dt;
    r = KO * (pm - integ - p0);
    % ---- FDI (lock signature) and reconfiguration
    if detected < 0
        cond = (abs(r) > r_th) & (abs(vm) < v_th) & (abs(cl.qd) > 0.05);
        cnt(cond) = cnt(cond) + 1; cnt(~cond) = 0;
        if cnt(3) >= persist && strcmp(o.scenario, 'S3')
            detected = t;
            if o.reconfigure
                cl.active(3) = false; cl.q = qm; cl.qd = zeros(3, 1);
                enabled(3) = false; Ie(:) = 0; buf(:) = 0;
            end
        end
    end
    % ---- reference (CLIK), returned at time t
    [q_r, qd_r, qdd_r] = clik_accel(cl, t, pn);
    cl.q = cl.q + cl.qd * dt + 0.5 * qdd_r * dt^2;   % exact for piecewise-constant qdd
    cl.qd = cl.qd + qdd_r * dt;
    % ---- FOSMC / ISMC
    e = qm - q_r; ed = vm - qd_r;
    if useGL
        buf = [e'; buf(1:end-1, :)];
        De = hgl * (buf' * w);
    else
        De = e;
    end
    inc = De * dt; if o.antiwindup, inc(satf) = 0; end
    Ie = Ie + inc;
    v_r = qd_r - L1 .* e - L2 .* Ie;
    a_r = qdd_r - L1 .* ed - L2 .* De;
    s = vm - v_r;
    tau = M * a_r + C * v_r + friction(vm, pn) - KD .* s - rho .* min(max(s / phi, -1), 1);
    satf = abs(tau) >= pn.tau_max;
    tau = min(max(tau, -pn.tau_max), pn.tau_max);
    tau(~enabled) = 0;
    % ---- log
    out.q(k, :) = pl.q'; out.qr(k, :) = q_r'; out.x(k, :) = fkine(pl.q, pt)';
    out.xd(k, :) = task_ref(t)'; out.tau(k, :) = tau'; out.r(k, :) = r';
    % ---- plant (RK4, two sub-steps)
    pl = plant_step(pl, t, tau, dt, pt);
end
out.detected = detected;
out.params = pn;   % nominal model actually used (check l1 = 0.400, l2 = 0.250)
ex = sqrt(sum((out.x - out.xd).^2, 2)) * 1e3; m = out.t >= 1;
out.rmse = sqrt(mean(ex(m).^2)); out.emax = max(ex(m));
end

% =====================================================================
function o = defaults(o, d)
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end

function p = nominal_params()
p.m0 = 30; p.m1 = 6; p.l1 = 0.400; p.lc1 = 0.20; p.I1 = 0.080;   % ABB IRB 910SC-3/0.65 arm lengths
p.m2 = 3; p.l2 = 0.250; p.lc2 = 0.13; p.I2 = 0.020;
p.fv = [40; 2; 1]; p.fc = [15; 1.5; 0.8]; p.tau_max = [600; 60; 30];
end

function p = with_payload(p, mp)
m2n = p.m2 + mp; lc2n = (p.m2 * p.lc2 + mp * p.l2) / m2n;
p.I2 = p.I2 + p.m2 * (p.lc2 - lc2n)^2 + mp * (p.l2 - lc2n)^2;
p.m2 = m2n; p.lc2 = lc2n;
end

function M = M_mat(q, p)
s1 = sin(q(2)); s12 = sin(q(2) + q(3)); c2 = cos(q(3));
a = -(p.l1 * p.m2 + p.lc1 * p.m1) * s1 - p.lc2 * p.m2 * s12;
b = -p.lc2 * p.m2 * s12;
m22 = p.I1 + p.I2 + p.lc1^2 * p.m1 + p.m2 * (p.l1^2 + 2 * p.l1 * p.lc2 * c2 + p.lc2^2);
m23 = p.I2 + p.lc2 * p.m2 * (p.l1 * c2 + p.lc2);
m33 = p.I2 + p.lc2^2 * p.m2;
M = [p.m0 + p.m1 + p.m2, a, b; a, m22, m23; b, m23, m33];
end

function C = C_mat(q, v, p)
c1 = cos(q(2)); c12 = cos(q(2) + q(3)); h = p.l1 * p.lc2 * p.m2 * sin(q(3));
C = [0, -(p.l1 * p.m2 + p.lc1 * p.m1) * v(2) * c1 - p.lc2 * p.m2 * (v(2) + v(3)) * c12, -p.lc2 * p.m2 * (v(2) + v(3)) * c12;
     0, -h * v(3), -h * (v(2) + v(3));
     0, h * v(2), 0];
end

function F = friction(v, p)
F = p.fv .* v + p.fc .* tanh(v / 0.01);
end

function x = fkine(q, p)
x = [q(1) + p.l1 * cos(q(2)) + p.l2 * cos(q(2) + q(3)); p.l1 * sin(q(2)) + p.l2 * sin(q(2) + q(3))];
end

function J = jacobian(q, p)
s1 = sin(q(2)); s12 = sin(q(2) + q(3)); c1 = cos(q(2)); c12 = cos(q(2) + q(3));
J = [1, -p.l1 * s1 - p.l2 * s12, -p.l2 * s12; 0, p.l1 * c1 + p.l2 * c12, p.l2 * c12];
end

function [ph, phd, phdd] = phase(t)
T = 4.0; tr = 1.0; W = 2 * pi / T;
if t < tr
    s = t / tr; r = 10 * s^3 - 15 * s^4 + 6 * s^5; rd = (30 * s^2 - 60 * s^3 + 30 * s^4) / tr;
    R = tr * (2.5 * s^4 - 3 * s^5 + s^6);
    ph = W * R; phd = W * r; phdd = W * rd;
else
    ph = W * (tr * 0.5 + (t - tr)); phd = W; phdd = 0;
end
end

function [x, xd, xdd] = task_ref(t)
A = 0.55; B = 0.06; [ph, phd, phdd] = phase(t);
x = [A * sin(ph); 0.36 + B * sin(2 * ph)];
d1 = [A * cos(ph); 2 * B * cos(2 * ph)]; d2 = [-A * sin(ph); -4 * B * sin(2 * ph)];
xd = d1 * phd; xdd = d2 * phd^2 + d1 * phdd;
end

function q = initial_q(p)
x0 = task_ref(0); q2 = pi / 2;
R = hypot(p.l1 + p.l2 * cos(q2), p.l2 * sin(q2));
beta = atan2(p.l2 * sin(q2), p.l1 + p.l2 * cos(q2));
q1 = asin(x0(2) / R) - beta; d = x0(1) - R * cos(q1 + beta);
q = [d; q1; q2];
end

function [q, qd, qdd] = clik_accel(cl, t, p)
[x, xd, xdd] = task_ref(t);
q = cl.q; qd = cl.qd; a = cl.active;
J = jacobian(q, p);
s1 = sin(q(2)); s12 = sin(q(2) + q(3)); c1 = cos(q(2)); c12 = cos(q(2) + q(3));
w1 = qd(2); w12 = qd(2) + qd(3);
Jdqd = [-p.l1 * c1 * w1^2 - p.l2 * c12 * w12^2; -p.l1 * s1 * w1^2 - p.l2 * s12 * w12^2];
u = xdd + cl.Kv * (xd - J * qd) + cl.Kp * (x - fkine(q, p)) - Jdqd;
Ja = J(:, a); Winv = diag(1 ./ cl.W(a));
JW = Winv * Ja' / (Ja * Winv * Ja' + 1e-6 * eye(2));
qdd_a = JW * u;
if sum(a) > 2
    grad = zeros(3, 1); grad(3) = q(3) - cl.q2s;
    z = -cl.kn * grad(a) - 2 * sqrt(cl.kn) * qd(a);
    qdd_a = qdd_a + (eye(sum(a)) - JW * Ja) * z;
end
qdd = zeros(3, 1); qdd(a) = qdd_a;
end

function a = plant_acc(t, q, qd, tau, pl, p)
M = M_mat(q, p); C = C_mat(q, qd, p);
dist = [10 * sin(2 * pi * 1.5 * t); 1.0 * sin(2 * pi * 2.0 * t); 0.5 * cos(2 * pi * 2.5 * t)];
if t >= 6.0, dist = dist + [25; 2; 1]; end
rhs = pl.eff .* tau + pl.bias - C * qd - friction(qd, p) - dist;
f = pl.free; a = zeros(3, 1);
a(f) = M(f, f) \ rhs(f);
end

function pl = plant_step(pl, t, tau, dt, p)
sub = 2; h = dt / sub;
for k = 0:sub-1
    tk = t + k * h; q = pl.q; v = pl.qd;
    a1 = plant_acc(tk, q, v, tau, pl, p); v1 = v;
    a2 = plant_acc(tk + h/2, q + h/2 * v1, v + h/2 * a1, tau, pl, p); v2 = v + h/2 * a1;
    a3 = plant_acc(tk + h/2, q + h/2 * v2, v + h/2 * a2, tau, pl, p); v3 = v + h/2 * a2;
    a4 = plant_acc(tk + h, q + h * v3, v + h * a3, tau, pl, p); v4 = v + h * a3;
    pl.q = q + h/6 * (v1 + 2*v2 + 2*v3 + v4);
    pl.qd = v + h/6 * (a1 + 2*a2 + 2*a3 + a4);
    pl.qd(~pl.free) = 0;
end
end
