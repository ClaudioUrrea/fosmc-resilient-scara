# fosmc-resilient-scara

**Release v1.0.0** · MIT License · Archived at Figshare: [10.6084/m9.figshare.33987061](https://doi.org/10.6084/m9.figshare.33987061)

Code, raw results and RoboDK digital twin for the article

> C. Urrea, *Resilient Fractional-Order Sliding-Mode Control of Redundant Robots Under Actuator Faults and Joint Locking: Theory, Short-Memory Implementation, and an Open Digital-Twin Benchmark*, submitted to *Fractal and Fractional* (MDPI), 2026.

The benchmark is a kinematically redundant rail-mounted SCARA robot (ABB IRB 910SC-3/0.65 arm lengths, 400 mm + 250 mm, on a 2-m linear rail) that follows a dispensing contour while an actuator loses 50 % of its effectiveness and the elbow joint locks. Four controllers are compared under a common control-activity budget: computed-torque PID (CT-PID), two integer integral sliding-mode controllers (ISMC-A with the same gains as the proposed controller, ISMC-B best-tuned) and the proposed fractional-order sliding-mode controller (FOSMC, λ = 0.7) with a short-memory Grünwald–Letnikov implementation, fault-aware redundancy resolution and momentum-observer fault detection and isolation.

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

## Runs deposited

- **Evaluation:** 240 Monte Carlo runs = 3 scenarios (S1 no fault, S2 LOE, S3 LOE + elbow lock) × 4 controllers × seeds 100–119 (`results/mc.json`).
- **Supplementary (408 runs):** tuning grid 96 (24 configurations × seeds 0–3), anti-windup 60, ablation 20, memory length 36, actuation delay 192 (96 configurations × seeds 0–1), deterministic cross-implementation cases 4.

Every run is defined by its seed and by the parameters in `sim.py`; the raw metrics of all runs are stored, so the statistics can be recomputed and the gain selection repeated under any other activity budget without re-running the simulations.

## Reproducing the paper

Requirements: Python ≥ 3.10 (`pip install -r requirements.txt`); MATLAB R2026b or GNU Octave for the cross-check; RoboDK ≥ 5.6 for the digital twin (optional). Times refer to one CPU core.

```bash
python derive.py              # symbolic model check (seconds)
python tune_final.py          # gain selection (~6 min)
python run_all.py             # Monte Carlo, ablations, memory and delay campaigns (~30 min)
python run_aw.py              # anti-windup robustness check (~4 min)
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

> Urrea, C. Resilient Fractional-Order Sliding-Mode Control of Redundant Robots: Code, Raw Results and RoboDK Digital Twin (Release v1.0.0) [Dataset]; Figshare, 2026. https://doi.org/10.6084/m9.figshare.33987061

## License

MIT (see `LICENSE`). The RoboDK robot model is not redistributed; it is downloaded from the RoboDK library.
