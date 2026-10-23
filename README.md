# fosmc-resilient-scara

**Release v1.2.0** · MIT License · Archived at Figshare: [10.6084/m9.figshare.33987061](https://doi.org/10.6084/m9.figshare.33987061)

Code, raw results and RoboDK kinematic twin for the article

> C. Urrea, *Fractional-Order Sliding-Mode Control of a Rail-Mounted Redundant SCARA Under Actuator Degradation and Elbow Locking: Truncated-Memory Analysis and a Reproducible Simulation Benchmark*, *Fractal and Fractional* (MDPI), 2026 (under revision).

The benchmark is a kinematically redundant rail-mounted SCARA robot (P-R-R, ABB IRB 910SC-3/0.65 arm lengths, 400 mm + 250 mm, on a 2-m linear rail) that follows a dispensing contour while an actuator loses 50 % of its effectiveness and the elbow joint locks (scenarios S1 no fault, S2 loss of effectiveness, S3 = S2 + elbow lock). Sliding-mode controllers with integer (ISMC) and fractional (FOSMC, short-memory Grünwald-Letnikov, exact truncated kernel) integral action are compared with a computed-torque PID under a common control-activity budget, with a fair-tuning protocol (tuning seeds 0-3, evaluation seeds 100-199), fault-aware redundancy resolution and momentum-observer fault detection and isolation.

**What is new in v1.2.0 (revision R2).** Reduced-chain uncertainty and margins after isolation (`camp_R2.py`, campaign M, seeds 100-299, with independent confirmation of the gain schedule on seeds 200-299); payload included in the controller model with a ±20 % error (campaign P); tuning with a robustness constraint (`camp_R2T.py`, ISMC-K and FOSMC-K); Figure 11a extended to seeds 100-299. 2552 additional runs; the simulator is unchanged for existing options. See `CHANGELOG.md`.

**What is new in v1.1.x (revision R1).** Seven controllers and generic fault detection (`sim2.py`); anti-windup, actuator lag, quantisation, uncertainty, LOE ramp and enlarged robust-gain campaigns; analysis of the post-lock failure mechanism (payload, saturation demand, margin violation) and of a gain schedule at reconfiguration; numerical checks of the theory (`theory_num.py`); MATLAB R2026b and Octave cross-check; software-emulated hardware test protocol (`HARDWARE_TEST_PROTOCOL.md`, not yet executed); graphical abstract. See `CHANGELOG.md`.

## Contents

| Path | Content |
|---|---|
| `sim.py` | Plant (P–R–R dynamics, friction, LOE, joint lock), fault-aware CLIK with bumpless reconfiguration, short-memory GL operator, FOSMC/ISMC/CT-PID, momentum observer, scenario runner |
| `derive.py` | Symbolic derivation of M and C and check of the skew-symmetry of Ṁ − 2C |
| `tune_final.py` | Gain selection on tuning seeds 0–3 (scenario S2) under the common activity budget → `results/tuning.json`, `results/selection.json` |
| `run_all.py` | Representative runs, Monte Carlo (seeds 100–119), ablations, memory-length and actuation-delay campaigns |
| `run_aw.py` | Robustness check of scenario S3 with conditional-integration anti-windup |
| `verify_py.py` | Deterministic reference trajectories for the cross-implementation check |
| `matlab/fosmc_sim.m` | Independent MATLAB implementation of the whole closed loop (MATLAB R2016b or later, GNU Octave ≥ 6) |
| `matlab/verify_matlab.m` | MATLAB-vs-Python comparison → `matlab/verify_report.txt` |
| `matlab/palatino_style.m`, `matlab/fix_minus.m`, `matlab/analyze_experiment.m` | Figure style helpers and metrics for experimental logs |
| `figs_theory.py`, `figs_results.py`, `fontsetup.py`, `fonts/` | Figures 1, 3, 4, 6–11 (Palatino-type typeface, true minus sign U+2212) |
| `make_tables.py` | Tables 2 and 3 → `tables/*.csv`, `tables/*.md` |
| `robodk_scene.py` | RoboDK digital twin: builds the cell, self-calibrates the kinematics, replays the trajectories, checks collisions and TCP deviation, saves snapshots |
| `results/` | Raw per-run metrics (`*.json`), representative time series (`rep_*.npz`), replayed trajectories (`traj_*.csv`), verification trajectories (`verify_*.csv`) |
| `figures/` | Figures as published (PDF/PNG), `fig_architecture.tex` (Figure 2, TikZ) and RoboDK snapshots |
| `EXPERIMENTAL_PROTOCOL.md` | Protocol for the experimental validation on a physical SCARA |

### v1.1.x (revision R1): added files

| Path | Content |
|---|---|
| `sim2.py` | Extended simulator: generic FDI, conditional/back-calculation anti-windup, LOE ramp, actuator lag, quantisation, uncertainty scaling, configurable robust gain, gain schedule at reconfiguration (`gains_post`), true-uncertainty logging |
| `ff_campaigns.py`, `failfn.py` | Campaign runner (resumable, one JSON summary per run) and the common failure definition |
| `camp_Z.py`, `camp_Z3b.py`, `camp_Q.py`, `camp_Q2.py`, `camp_S.py`, `camp_S2.py`, `camp_R.py` | Revision campaigns: anti-windup, delay, enlarged robust gain, gain schedule |
| `an_all.py`, `an_Z.py`, `an_Z2.py`, `an_Q.py`, `stats.py`, `fail.py` | Tables and statistics (Wilcoxon, Fisher, failure-inclusive ranking) |
| `diag_payload.py`, `diag_demand.py`, `diag_noreset.py`, `diag_runs.py`, `fig_failmech.py` | Post-lock failure mechanism (Section 3.6, Figure 11) → `diagnostics/`, `diag/` |
| `theory_num.py`, `gl_bench.c` | Numerical checks of Lemma 1, Proposition 3 and Corollary 2; cost per step of the GL operator |
| `figs_r1.py`, `fig_tuning.py`, `fig_kernel.py` | Figures of the revised paper |
| `results2/` | Raw run summaries (`*.jsonl`) and gain selection (`selection2.json`) |
| `graphical_abstract/` | `GA_Fractal_and_Fractional.py` regenerates the graphical abstract (PNG/TIFF) |
| `HARDWARE_TEST_PROTOCOL.md` | Protocol for the experimental validation (not yet executed) |


### v1.2.0 (revision R2): added files

| Path | Content |
|---|---|
| `camp_R2.py` | Campaigns M (reduced-chain uncertainty after isolation and gain schedule, S3, seeds 100-299) and P (payload in the controller model, ±20 %, S3, seeds 100-199) |
| `camp_R2T.py` | Tuning with a robustness constraint (tuning seeds 0-3, no failure under a 2-ms lag and a 2-ms delay) and evaluation of ISMC-K and FOSMC-K (seeds 100-119) |
| `an_R2.py`, `an_T.py` | Analysis of Table 7 (reduced-chain margins), gain schedule on seeds 200-299, payload test, and Table 9 |
| `results2/camp_R2M.jsonl`, `camp_R2P.jsonl`, `camp_R2T1.jsonl`, `camp_R2T2.jsonl`, `an_R2.json`, `an_T.json`, `sel_R2T.json` | Raw run summaries (1600, 400, 272 and 280 runs) and analysis outputs |
| `sim2.py` | New optional arguments `payload_err` and `log_red`; defaults reproduce v1.1.1 |

## Runs deposited

- **v1.2.0 (`results2/camp_R2*.jsonl`):** 2552 run summaries added (reduced-chain margins, gain schedule on independent seeds, payload in the controller model, tuning with a robustness constraint).
- **v1.1.x (`results2/`):** 5060 run summaries (tuning grids, main Monte Carlo with seven controllers, memory length, reset policy, anti-windup, severity × saturation, LOE ramp, FDI thresholds, input delay, lag/quantisation/uncertainty, enlarged robust gain, gain schedule). Evaluation seeds 100-199; tuning seeds 0-3.
- **v1.0.0 (`results/`):** 648 runs of the original submission (240 Monte Carlo + 408 supplementary), kept unchanged for traceability.

Every run is defined by its seed and by the parameters in `sim.py`; the raw metrics of all runs are stored, so the statistics can be recomputed and the gain selection repeated under any other activity budget without re-running the simulations.

## Reproducing the paper

Requirements: Python ≥ 3.10 (`pip install -r requirements.txt`); MATLAB R2026b or GNU Octave for the cross-check; RoboDK ≥ 5.6 for the digital twin (optional). Times refer to one CPU core.

```bash
python derive.py              # symbolic model check (seconds)
python tune_final.py          # gain selection (~6 min)
python run_all.py             # Monte Carlo, ablations, memory and delay campaigns (~30 min)
python run_aw.py              # anti-windup robustness check (~4 min)
# v1.1.x campaigns (resumable; each writes results2/*.jsonl)
python ff_campaigns.py        # stages 1a-c and campaigns A-K, R
python camp_Z.py && python camp_Z3b.py && python camp_Q.py && python camp_Q2.py && python camp_S.py && python camp_S2.py
python an_all.py && python an_Z.py && python an_Z2.py && python an_Q.py   # tables
python diag_payload.py && python diag_demand.py && python diag_noreset.py && python diag_runs.py && python fig_failmech.py
# v1.2.0 campaigns (revision R2)
python camp_R2.py all && python camp_R2T.py && python an_R2.py && python an_T.py
python fig_failmech.py        # Figure 11 (a: seeds 100-299)
python figs_r1.py             # figures of the revised paper
python verify_py.py           # reference trajectories for the MATLAB cross-check
python figs_theory.py         # Figures 1, 3, 4
python figs_results.py        # Figures 6-11
python make_tables.py         # Tables 2 and 3
```

MATLAB cross-implementation check (from the `matlab` folder):

```matlab
verify_matlab          % or, in a terminal: octave-cli verify_matlab.m
```

Expected: maximum joint difference of order 1e-15 and identical lock-isolation instants (see `matlab/verify_report*.txt`).

RoboDK digital twin (Windows, RoboDK open, empty station):

```
python robodk_scene.py --csv results\traj_S3_FOSMC.csv --snap figures --zoom 2.0
```

`--zoom` is a camera-distance factor (> 1 moves the camera away). Expected output: link lengths 400.0/250.0 mm, maximum TCP deviation ≈ 0.001 mm and no frame in collision.

| Paper item | Produced by |
|---|---|
| Figure 1 | `figs_theory.py` → `figures/fig_schematic.*` |
| Figure 2 | `figures/fig_architecture.tex` |
| Figures 3, 4 | `figs_theory.py` → `fig_sliding.*`, `fig_gl.*` |
| Figure 5 | `robodk_scene.py` → `figures/robodk_reconfigured.png` (cropped as `robodk_reconfigured_.png`) |
| Figures 6–11 | `figs_results.py` → `fig_s3_tracking`, `fig_s3_internals`, `fig_mc`, `fig_ablation`, `fig_sensitivity`, `fig_delay` |
| Tables 2, 3 | `make_tables.py` → `tables/` |

## Citation

If you use this code or data, please cite the article and the archived release (see `CITATION.cff`):

> Urrea, C. Fractional-Order Sliding-Mode Control of a Rail-Mounted Redundant SCARA: Code, Raw Results and RoboDK Kinematic Twin (Release v1.1.1) [Dataset]; Figshare, 2026. https://doi.org/10.6084/m9.figshare.33987061

## License

MIT (see `LICENSE`). The RoboDK robot model is not redistributed; it is downloaded from the RoboDK library.
