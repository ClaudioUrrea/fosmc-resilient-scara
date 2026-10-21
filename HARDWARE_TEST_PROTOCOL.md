# Hardware test protocol with software-emulated faults (rail-mounted SCARA)

Purpose: reproduce scenarios S1–S3 of the paper on a physical rail–SCARA without damaging the robot, so that FOSMC and ISMC can be compared under the same faults, disturbances and activity budget. The protocol is provided for future experimental validation; it has not yet been executed.

## 1. Platform requirements
- Rail axis + SCARA arm (or equivalent P–R–R chain) with torque/current-mode access at >= 1 kHz (e.g., an open real-time controller or drive torque interface).
- Joint encoders (>= 17 bit) and drive current measurement; motor torque constants identified beforehand.
- Hardware safety: software torque limits below the drive limits, joint-limit switches, a dead-man switch and an independent emergency stop. Workspace free of obstacles (the RoboDK scene of the release can be used to check the layout).

## 2. Controller implementation
- Port `sim2.py` controller blocks (CLIK with null-space objective, SlidingController with short-memory GL operator, momentum observer and FDI logic) to the real-time target; keep `dt = 1 ms` and `L = 1 s` (N = 1000). The per-step cost of the GL operator is given by `gl_bench.c` (2.8 us on a 2.1-GHz core).
- Use the gains of Table 4 (and P = diag(60,25,3) for the theorem-compliant variant). Reduce the trajectory speed (e.g., 50% of the nominal 4-s lemniscate period) for the first trials.

## 3. Software fault emulation (no hardware modification)
- **Loss of effectiveness (S2):** at t = 3 s multiply the commanded torque of θ1 by b1 = 0.5 after the controller output and add a 3-N·m bias: `tau_applied[1] = 0.5*tau_cmd[1] + 3.0`. The controller is not informed.
- **Elbow lock (S3):** at t = 4 s replace the θ2 command by a stiff position-hold loop (or engage the drive holding brake if available) at the current angle; the sliding-mode controller keeps computing its θ2 command, which is discarded, until the FDI isolates the lock and reconfiguration disables it.
- **Disturbance:** add the multisine and the step of the paper to the applied torque in software.
- **Ramp LOE:** b1 decreasing linearly from 1 to 0.5 between 3 and 5 s.

## 4. Procedure
1. Identify friction and payload; record the nominal parameter set and its uncertainty.
2. For each controller (CT-PID, ISMC-A, ISMC-J, FOSMC, FOSMC-J) and scenario (S1, S2, S3) run at least 10 repetitions in randomised order, with the same disturbance realisation per repetition index.
3. Before each S3 run, check that θ2 stays below θ2max = 98.3° (Remark 1); abort otherwise.
4. Abort criteria: tracking error > 20 mm, torque saturation > 1 s, or any safety trip; count an abort as a failure.

## 5. Measurements and metrics
- Log q, q_dot, commanded and applied torques, residuals r, alarms and FDI state at 1 kHz.
- End-effector error from forward kinematics of the encoders (and from an external tracker, e.g., laser tracker, if available).
- Metrics as in the paper: RMSE, maximum error, recovery time to the 1-mm band, TV, FDI isolation time, false alarms and missed detections, failures.
- Statistics: paired Wilcoxon on common successful repetitions; Fisher exact test for failure counts.

## 6. Expected comparison with simulation
Compare the measured ranking and failure counts with Tables 5, 8 and 9; report discrepancies together with the identified differences (joint elasticity, drive bandwidth, cogging, delay).
