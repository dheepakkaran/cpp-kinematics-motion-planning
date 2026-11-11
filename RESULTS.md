# RESULTS

All numbers below are copied from files written by the code in this repo (full run of 2026-11-09 14:41 EST,
`colcon test`, Release build, Ubuntu 26.04.1, kernel 7.0.0-42-generic, 8 cores, another build session running
in parallel). Simulation only; nothing ran on a real robot.

## Tests
| What | Number | Source |
|---|---|---|
| GoogleTest cases | 10 passed, 0 failed (test_kinematics 5, test_planner 2, test_chomp 2, test_node 1) | `results/gtest_xml/`, `results/colcon_test_result.txt` |

Note: colcon's summary line says `14 tests, 0 errors, 0 failures` because it also counts the 4 ctest
executables (`Testing/.../Test.xml`). The real GoogleTest case count is **10**.

## Kinematics (`results/kinematics.json`)
| What | Number |
|---|---|
| Panda FK vs MuJoCo 3.16.0 `attachment_site`, 1,000 random configs: max position error | 1.04185e-15 m |
| Panda FK vs MuJoCo: max rotation error | 3.49198e-08 rad (MuJoCo's MJCF stores the 135 deg tool quat with 7 digits, `0.3826834 0 0 0.9238795`; the C++ tool uses exact 3*pi/4) |
| Panda Jacobian vs MuJoCo `mj_jacSite`, 1,000 configs: max abs error | 3.77476e-15 |
| Jacobian vs central finite differences (h = 1e-6), 1,000 configs: Panda | 2.46641e-10 |
| Jacobian vs central finite differences, 1,000 configs: palletizer | 4.35813e-10 |
| Palletizer tool tilt from vertical, 1,000 configs: max | 2.58096e-08 rad (floating-point noise of `acos` near 1) |
| IK Panda (6-D pose), 1,000 reachable targets, random seed, up to 10 attempts: success | 991 / 1000 (457 on the first seed) |
| IK Panda: solve time median / p95 | 3.48751 ms / 18.9142 ms |
| IK palletizer (x, y, z, yaw), 1,000 targets: success | 989 / 1000 (541 on the first seed) |
| IK palletizer: solve time median / p95 | 0.106403 ms / 7.18911 ms |
| IK tolerance | position 1e-4 m (norm), rotation 1e-3 rad; max position error of successes: Panda 9.99802e-05 m, palletizer 9.9743e-05 m |

## MATLAB-script check (`results/octave_dryrun_check.txt`, GNU Octave 10.3.0 dry run)
`matlab/check_kinematics.m` re-implements both DH chains from scratch and reads `results/fk_samples.csv`
(100 configs per chain exported by the C++ test).

| Chain | FK pos max err | R max abs err | Jacobian vs C++ | Jacobian vs MATLAB-side finite differences |
|---|---|---|---|---|
| Panda | 2.289e-16 m | 3.331e-16 | 3.331e-16 | 2.529e-10 |
| Palletizer | 4.475e-16 m | 3.331e-16 | 4.441e-16 | 3.965e-10 |

That table is the GNU Octave dry run. The same script then ran in **MATLAB R2026b** (MATLAB Online, 2026-11-09), source `results/matlab_check.txt`:

| Chain | FK pos max err | R max abs err | Jacobian vs C++ | Jacobian vs MATLAB-side finite differences |
|---|---|---|---|---|
| Panda | 2.483e-16 m | 3.331e-16 | 3.331e-16 | 2.230e-10 |
| Palletizer | 3.331e-16 m | 3.331e-16 | 4.441e-16 | 3.965e-10 |

## Planning (`results/planning.json`)
20 Panda queries whose straight joint-space line hits the scene (floor + pillar + shelf). OMPL 2.0.2 RRT-Connect,
then path shortcutting = OMPL `reduceVertices` + `partialShortcutPath` + `reduceVertices`, then a
trapezoidal time parameterization (vmax 1 rad/s, amax 2 rad/s^2).

| What | Number |
|---|---|
| Solved and valid (dense 0.01 rad re-check) | 20 / 20 |
| Planning time median / max | 2.17509 ms / 8.95056 ms |
| Shortcut optimization time median | 0.946288 ms |
| Joint-space path length median, before -> after optimization | 12.8451 rad -> 10.0401 rad |
| Total path length over 20 queries, before -> after | 266.919 rad -> 196.46 rad (-26.4 %, computed from these two numbers) |
| Waypoints median, before -> after | 6.5 -> 4 |
| Trajectory duration median | 8.11847 s |

## Trajectory optimization: CHOMP-style (`results/trajopt.json`)
Same 20 blocked queries. CHOMP-style optimizer (`src/chomp.cpp`): path resampled to N = 50 waypoints, cost =
smoothness (0.5 * sum of squared second differences) + 10 * obstacle cost (sphere-to-box signed distance with a
0.05 m margin), covariant gradient descent preconditioned by the smoothness metric A = K^T K, endpoints fixed,
joint-limit clamping, step halving, stop at relative change < 1e-6 or 300 iterations. Smoothness is computed
for every variant after resampling to the same N = 50 so the numbers compare. Clearance = smallest
sphere-surface-to-box distance along the path (checked every 0.01 rad).

| Variant | Dense-valid | Smoothness median (rad^2) | Length median (rad) | Min clearance median / worst (m) | Time median (ms) |
|---|---|---|---|---|---|
| RRT-Connect raw | 20/20 | 0.12567 | 12.8451 | 0.017838 / 7.06357e-05 | - |
| Shortcut (OMPL simplifier) | 20/20 | 0.0466485 | 10.0401 | 0.0300004 / 7.04591e-05 | 0.976737 |
| CHOMP from raw | 20/20 | 0.0932137 | 11.7047 | 0.0619104 / 0.00358835 | 356.95 |
| Shortcut then CHOMP | 20/20 | 0.0235892 | 9.40506 | 0.0499565 / 0.00358835 | 364.399 (incl. shortcut) |

CHOMP cost went down on 20/20 queries from raw and 20/20 from the shortcut path.

How it compares (honest):
- **CHOMP alone (from the raw RRT path) is worse than shortcutting** on smoothness (0.0932137 vs 0.0466485)
  and length (11.7047 vs 10.0401 rad), and it is about 365x slower (356.95 vs 0.976737 ms median, computed).
  Its one gain is clearance: median 0.0619104 m vs 0.0300004 m, and worst 0.00358835 m vs 7.04591e-05 m.
- **Shortcut then CHOMP is the best path** on smoothness (0.0235892), length (9.40506 rad) and clearance
  (median 0.0499565 m), at about 364 ms per query.
- The CHOMP "worst clearance" of 0.00358835 m is the same for both CHOMP variants. It is most likely the start or
  goal configuration, which is fixed and so cannot be moved away (not checked separately).
- Timings come from a loaded machine (another build session was running) and change from run to run.

## ROS 2 node + fixed-rate loop (`results/rt_loop.json`)
In-process integration test: 3 goal positions sent over `/kinmp/goal`, each IK -> plan -> execute.

| What | Number |
|---|---|
| Goals planned / executed | 3 / 3 |
| Final tool position error (from `/joint_states`), max | 7.90346e-05 m |
| Loop rate | 500 Hz |
| Loop iterations while executing | 5538 |
| Missed deadlines | 219 (4.0 % of 5538, computed) |
| Wake-up lateness mean / p50 / p99 / max | 258.548 / 71.389 / 3065.88 / 8845.97 us |

### All runs of the node test (`results/rt_loop_run1.json` .. `rt_loop_run4.json`, 14:30-14:42)
`results/rt_loop.json` = run 4 (the 14:41 full run; the table above). Same binary, run back to back while another build session used the machine.

| Run | Iterations | Missed deadlines | Wake lateness p99 | max | Final tool pos err max |
|---|---|---|---|---|---|
| 1 | 5802 | 8 | 258.816 us | 3711.14 us | 7.90346e-05 m |
| 2 | 5781 | 28 | 942.117 us | 5111.19 us | 7.90346e-05 m |
| 3 | 4712 | 620 | 8244.41 us | 34521.5 us | 7.90346e-05 m |
| 4 | 5538 | 219 | 3065.88 us | 8845.97 us | 7.90346e-05 m |

Run 3 shows what soft real time means on a loaded normal kernel: 620 missed 2 ms deadlines (missed slots are
skipped, so fewer iterations). The counter caught it, which is the point of the counter; there is no RT
guarantee here (no PREEMPT_RT, no SCHED_FIFO, no CPU isolation). The trajectory still ended on the goal
because the setpoint is sampled from wall-clock time.
