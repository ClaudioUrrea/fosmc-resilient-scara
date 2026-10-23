# Changelog

# v1.2.0 (revision R2, October 2026; adds 2552 runs to v1.1.1)
- camp_R2.py: campaign M (1600 S3 runs: ISMC-J and FOSMC-J, nominal and enlarged P, with and without the gain schedule (20, 800), seeds 100-299; logs the reduced-chain uncertainty after isolation and its margin) and campaign P (400 S3 runs with the payload included in the controller model, relative error uniform in ±20 %, seeds 100-199).
- camp_R2T.py: tuning with a robustness constraint. Admissible configurations have no failure on tuning seeds 0-3 in S2 and S3 under a 2-ms lag and a 2-ms input delay; selected: ISMC-K (30, 1600) and FOSMC-K (lambda=0.7; 20, 800); evaluation on seeds 100-119 (552 runs in camp_R2T1/T2.jsonl, selection in sel_R2T.json).
- an_R2.py, an_T.py: analysis (Table 7, schedule on independent seeds, payload test, Table 9 with Wilcoxon tests; Holm-Bonferroni adjustment p < 2e-4 for the seven rows).
- sim2.py: new optional arguments payload_err and log_red (default None/False: results identical to v1.1.1). The MATLAB code is unchanged.
- fig_failmech.py: Figure 11a now pools seeds 100-299 (ISMC-J with the nominal P failed in 56 of 59 runs with payload >= 2.25 kg and in 2 of 117 runs below 1.87 kg).
- graphical_abstract/: regenerated for revision R2.
- Main findings: gain schedule on seeds 200-299: ISMC-J 0/100, FOSMC-J 1/100 failures (33 and 30 without it); payload known: 0/400 failures; with the robustness-constrained tuning, FOSMC-K is 4-15 % more accurate than ISMC-K (0/280 failures, p <= 2.7e-5).

# v1.1.1 (revision R1, October 2026; adds 1610 runs to v1.1.0, no change to the simulator)
- camp_Z.py: Z1, anti-windup (none / conditional / back-calculation) for ISMC-J and FOSMC-J in S3 on 100 evaluation seeds (100-199); Z2, completion of the stress-test table (ISMC-M and FOSMC-J in the LOE-ramp and realism K1-K3 rows, FOSMC-J at b1=0.5 with saturation x0.8); Z3, input delay on seeds 110-119 (all five controllers) so that the delay study uses 20 seeds. camp_Z3b.py: FOSMC-J at 0 and 11 ms on seeds 100-109.
- results2/camp_Z1.jsonl, camp_Z2.jsonl, camp_Z3.jsonl, camp_Z3b.jsonl: raw run summaries (resumable, same format as v1.1.0).
- an_Z.py (Table 6: anti-windup, Fisher tests), an_Z2.py (stress-test cells and delay table), failfn.py (failure definition shared by both).
- figs_r1.py: Figure 12 (delay) now uses 20 seeds. All other figures and files are unchanged.
- Main findings: without anti-windup ISMC-J / FOSMC-J fail in 35 / 33 of 100 runs (Fisher p=0.88); with conditional integration 35 / 8 of 100 (p=4.3e-6); with back-calculation 35 / 31 (p=0.65). Tolerable delay with 20 seeds: ISMC-A 7 ms, FOSMC 5 ms, ISMC-B/J and FOSMC-J < 3 ms. FOSMC-J fails in 8-10 of 10 S3 runs under actuator lag K1-K3.
- sim2.py, ff_campaigns.py, matlab/ and all v1.1.0 results are byte-identical to v1.1.0.
- camp_Q.py: Q, all six sliding-mode controllers in S1-S3 (20 seeds) with the robust gain P=diag(60,25,3) that satisfies Assumption 1(b) in the fault-free and LOE windows; camp_Q2.py: ISMC-J and FOSMC-J in S3 on seeds 120-199 with the same gain (100 seeds in total). an_Q.py: Table 8.
- Findings with P=diag(60,25,3): FOSMC-J vs ISMC-J -48% RMSE in S1 and S2 (p=1.9e-6); S3 failures out of 100: ISMC-J 0 (35 with nominal P), FOSMC-J 11 (33 with nominal P); TV about 45% above the nominal budget for all controllers.
- graphical_abstract/: GA_Fractal_and_Fractional.py regenerates Graphical_Abstract.png/.tiff (MDPI, RGB, 250 dpi) with the revised results.
- matlab/verify_report_MATLAB_R2026b.txt: verification in MATLAB R2026b (max |dq| <= 8.9e-16, max |dx| <= 7.9e-13 mm, same isolation sample); Octave 8.4 report kept.
- HARDWARE_TEST_PROTOCOL.md: experimental validation protocol with software-emulated LOE, lock and disturbances (not yet executed).
- Post-lock failure mechanism (Section 3.6, Figure 11): diag_payload.py (failures vs payload), diag_demand.py (400 runs with actuator limits x100: peak theta1 torque demand, divergence without saturation), diag_noreset.py (reset policy check), diag_runs.py (time series, limit cycles; writes diag/*.npz), fig_failmech.py. Results in diagnostics/*.json.
- sim2.py: new optional argument gains_post=(L1, L2), a gain schedule applied at reconfiguration (default None: identical results to v1.1.0, verified on A|FOSMC-J|S3|100). camp_S.py / camp_S2.py: FOSMC-J and ISMC-J with gains_post=(20, 800), nominal and enlarged P, S3, seeds 100-199 (results2/camp_S.jsonl). The MATLAB code does not implement gains_post; the verification cases are unaffected.
- Findings: ISMC-J (nominal P) diverges in 29 of its 35 failing runs even without saturation (margin violation); FOSMC-J never diverges without saturation (saturation failures, predicted by peak demand > ~95-105 N m in 93-94% of runs). With the gain schedule: failures 0/100 (ISMC-J) and 2/100 (FOSMC-J) with nominal P, 0/100 both with enlarged P; FOSMC-J S3 RMSE 8-10% below ISMC-J (p<1e-17).

# v1.1.0 (revision R1, October 2026)
- sim2.py: generic FDI (all joints, all scenarios; `fdi='legacy'` reproduces v1.0.0), conditional and back-calculation anti-windup, LOE ramp, severity/saturation scaling, actuator lag, encoder quantisation, uncertainty scaling, configurable robust gain `rho`, logging of the true lumped uncertainty Delta(t), compact run summaries.
- ff_campaigns.py: joint tuning (stage1a/b; stage1c extends the CT-PID grid) and campaigns A (main Monte Carlo, 7 controllers), D (memory length), E (reset policy), F/X (anti-windup), G (severity x saturation), H (LOE ramp), I (FDI thresholds), J (input delay), K (lag, quantisation, uncertainty), R (enlarged robust gain). Results: results2/*.jsonl (one JSON summary per run, resumable), selection2.json.
- theory_num.py: kappa_L, c_L, positivity of Re K_L(iw), L1 norms of the resolvent g_L. gl_bench.c: per-step cost of the GL operator.
- matlab/fosmc_sim.m: generic FDI and reset option; verify_matlab.m against verify_py.py references (Octave 8.4: max|dq| <= 9.4e-16).
- figs_r1.py, fig_tuning.py, fig_kernel.py: all figures of the revised paper. stats.py, fail.py, an_all.py: all tables and statistics.

## v1.0.0 (2026-09-24)
First public release, accompanying the submission to *Fractal and Fractional*.
