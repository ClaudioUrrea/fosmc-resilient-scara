# Experimental validation protocol (to strengthen the manuscript before or during review)

Goal: reproduce scenarios S1–S2 (and, if safe, a software-emulated S3) on a physical
SCARA arm, ideally on a linear axis. If no rail is available, a *non-redundant* test of
the controller (S1–S2 on the 2-DOF arm) already validates the torque-level FOSMC; the
redundancy/reconfiguration results then remain simulation-based and must be stated so.

## 1. Minimum hardware and software
* Torque (current) mode on every axis, 1 kHz loop, encoder read-back, command logging.
* MATLAB R2026 + Simulink Real-Time / Speedgoat, dSPACE, or a PC with EtherCAT master.
* Controller code: port `fosmc_sim.m` (control section only: GL buffer, surface, law,
  saturation, anti-windup flag). The same file already runs offline, so the offline and
  real-time versions share every line of control code.

## 2. Identification (before any comparison)
1. Identify the dynamic parameters (the author's own methods, e.g. Urrea & Pascal,
   2018/2021) and friction (constant-velocity sweeps).
2. Measure the input delay (command-to-current step) and report it: the delay study in
   the paper shows that it determines which (lambda, Lambda2) is admissible.

## 3. Test matrix (5 repetitions each, randomised order)
| Test | Controller | Fault emulation |
|---|---|---|
| E1 | ISMC (lambda = 1) | none |
| E2 | FOSMC (lambda = 0.7) | none |
| E3 | ISMC | 50 % LOE on theta1: multiply the command by 0.5 in software from t = 3 s |
| E4 | FOSMC | same as E3 |
| E5 (optional) | both | payload of 1–3 kg at the tool |
| E6 (optional, rail only) | FOSMC | elbow lock emulated by engaging the brake / position-hold of theta2 at t = 4 s, with FDI + reconfiguration |

Safety: soft joint limits, reduced speed first (period 8 s), then 4 s; emergency stop
on |e| > 5 mm; E6 only with the brake designed for dynamic engagement.

## 4. Data to log (CSV, 1 kHz)
`t, d, theta1, theta2, d_r, theta1_r, theta2_r, u_d, u_1, u_2` → `analyze_experiment.m`
computes the same metrics as the paper (Cartesian RMSE, maximum error, TV).

## 5. Reporting
* Same gains as in simulation first; if retuning is required, apply the same protocol
  (tuning runs separate from evaluation runs, common TV budget) to both controllers.
* Report mean ± SD over repetitions and a paired test (Wilcoxon), as in the paper.
* Add a subsection "Experimental Validation" to Results and remove limitation (i) of the
  Discussion accordingly.
