# BUILD-LOG

Built in a 60-minute time box. One Claude Code session on 2026-11-09, wall clock from `date`: **14:19 -> ~14:34 EST**.
Env reused from `projects/ros2-cpp-motion-control/` (its `pixi.toml` + `pixi.lock` + `scripts/env.sh` + Panda MJCF
copied; that project not modified).

## Done
| Step | Clock | Result |
|---|---|---|
| Repo + git, copied pixi env, added conda-forge `eigen` + `ompl`; separate `octave` pixi env | 14:19-14:21 | `pixi install` 19 s: eigen 5.0.1, ompl 2.0.2, libmujoco 3.16.0, rclcpp 28.1.22, gcc 15.3.0. OMPL smoke test (RRT-Connect, 2-D) compiled + solved at 14:20 (minute 1). Octave 10.3.0 env installed in 29 s |
| `kinematics.hpp` (DH chains, FK, Jacobian, DLS IK), Panda + palletizer | 14:21-14:22 | header-only, Eigen only |
| Planner (OMPL RRT-Connect + shortcutting + time param), scene | 14:22-14:23 | |
| GoogleTest: kinematics / planner / node tests | 14:23-14:24 | |
| ROS 2 node + in-process integration test | 14:24 | |
| First `colcon build` | 14:24-14:26 | one compile error: OMPL 2.0 has no `PathSimplifier::shortcutPath` (renamed); switched to `partialShortcutPath`. Build 77 s |
| First `colcon test` | 14:26 | 7/8 cases passed. **Real failure:** Panda IK 929/1000 < the 950 gate with 5 attempts x 300 iters. Also found: IK tolerance was per-axis, so "successes" had position error up to 1.62e-4 m > the 1e-4 m stated tolerance. Fixed: tolerance on the Euclidean norm, defaults 10 attempts x 500 iters -> 991/1000 (Panda), 989/1000 (palletizer) |
| `matlab/check_kinematics.m` + Octave dry run | 14:29-14:30 | two Octave-compat fixes (`run()` changes dir -> absolute paths; Octave scripts can't call functions defined at the end -> FD FK inlined). Result in `results/octave_dryrun_check.txt` |
| Full `colcon test` + logs, 2 extra node-test runs, README / RESULTS / BUILD-LOG, commit | 14:30-14:34 | `Summary: 11 tests, 0 errors, 0 failures` (= 8 GoogleTest cases + 3 ctest entries) |

## Step 2: CHOMP-style trajectory optimization (20-min box: 14:34-~14:45)
| Step | Clock | Result |
|---|---|---|
| `Chain::origin_jacobians`, `PlanResult::raw_path`, `src/chomp.cpp` (covariant gradient descent), `test/test_chomp.cpp` | 14:34-14:35 | |
| Build (header change rebuilt everything) | 14:35-14:40 | 4 min 8 s on the loaded machine |
| First benchmark | 14:40 | **Real failure:** cost went down on only 9 of 20 queries. Cause: the backtracking line search gave up after 8 step halvings. The obstacle gradient, multiplied by A^-1, needs much smaller steps. Fixed to 30 halvings: 20/20 |
| Full `colcon test` | 14:41 | `14 tests, 0 errors, 0 failures` = 10 GoogleTest cases + 4 ctest entries |

Not done: the ROS 2 node still executes the shortcut trajectory (needs a uniform-time parameterization for dense
CHOMP paths). No speed weighting or curvature term in the obstacle cost (CHOMP paper), no Hamiltonian Monte
Carlo restarts.

## Cuts and honest limits
- Cut on purpose: no Docker, no GitHub Actions, no cuRobo, no analytic IK.
- MATLAB: **only dry-run in GNU Octave**; MATLAB itself is not on this machine. `results/matlab_check.txt` is
  missing until Dheepak's step below. The check covers 200 sampled configs (100 per chain), not "every" Jacobian
  the library can produce.
- Executed trajectory = OMPL shortcutting + vertex reduction + trapezoidal time parameterization
  (rest-to-rest per segment, so the robot stops at each waypoint), no jerk limit. The CHOMP-style
  optimizer (step 2) is benchmarked offline only and is not used by the node.
- Collision: spheres along link segments vs boxes; no meshes, no self-collision. Palletizer is planned for
  nothing (IK/FK only); planning + node use the Panda.
- MuJoCo compare is Panda-only (no MJCF for the made-up palletizer); palletizer is checked by finite
  differences, the tool-level property and the Octave script.
- Node: goal is a topic (not a service/action); kinematic execution only (no dynamics); `ros2cli` not in env.
- Loop is soft real time: run 3 of the node test had 620 missed 2 ms deadlines out of 4712 iterations while
  another build session loaded the machine (RESULTS.md). Small reporting bug: the per-goal `"done"` status
  message is published before the last iteration's miss is counted, so per-goal `missed` can be 1 low; the
  total in `rt_loop.json` (`missed_deadlines`) is counted correctly.
- Panda FK vs MuJoCo rotation error 3.5e-8 rad comes from the MJCF's 7-digit tool quaternion, not the library.

## Remaining
1. **Dheepak, 5 min, MATLAB Online** (makes the "against MATLAB" claim true):
   1. Go to https://matlab.mathworks.com and sign in.
   2. In the Files panel, click Upload and pick `matlab/check_kinematics.m` and `results/fk_samples.csv`
      from this repo (put both in the same folder).
   3. In the Command Window type `check_kinematics` and press Enter (it prints 3 lines: engine, panda,
      palletizer; every `*_vs_cpp` error should be ~1e-15 and `*_vs_matlab_fd` ~1e-10).
   4. Right-click `matlab_check.txt` in the Files panel -> Download, save it as `results/matlab_check.txt`.
   5. `git add results/matlab_check.txt && git commit -m "Add MATLAB Online kinematics check"`.
2. Optional: push to GitHub (only on Dheepak's yes), CI, Dockerfile, service/action interface, self-collision,
   SCHED_FIFO / PREEMPT_RT experiment.

## MATLAB Online run (2026-11-09, Dheepak)
`results/matlab_check.txt`: MATLAB R2026b, 100 configs per chain. Jacobian vs C++ max 3.331e-16 (Panda) / 4.441e-16 (palletizer); FK position vs C++ 2.483e-16 / 3.331e-16 m; Jacobian vs MATLAB finite differences 2.230e-10 / 3.965e-10. Ran through `matlab/online-upload/run_kinmp_check.m` (one-file wrapper, function so old workspace variables can't clash; first try failed on a leftover `L` variable from another script, fixed by `clear` + the function wrapper). Output copied from the Command Window.
