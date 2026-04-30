# THA4 — Constrained Optimization-Based Robot Control
**ME384R: Algorithms for Sensor-Based Robotics**
**Prof. Farshid Alambeigi, UT Austin — Spring 2026**
**Due: 04/30/2026 at 3:30 PM**

---

## Overview

This assignment extends THA2's forward kinematics and numerical IK work into
**real-time constrained velocity control** using Quadratic Programming (QP).
The core idea: instead of solving for a target configuration (IK), we compute
joint velocity increments at each timestep that optimally satisfy competing
objectives and hard constraints simultaneously.

The robot is the **KUKA KR120 R2500 Pro (Quantec Nano)** — a 6-DOF serial
manipulator with a spherical wrist. Parameters live in `KR120_params.m` in the
project root.

---

## The Setup

A **cylindrical tool** (100 mm length, 5 mm diameter) is rigidly attached to
the robot's end-effector (EE) frame. The point of interest is the **tool tip**,
located 100 mm along the z-axis of the EE frame:

```
p_tip = p_ee + R_ee * [0; 0; 0.1]   (all in world frame, meters)
```

The goal in every sub-part is to **drive the tool tip toward an arbitrary goal
point `p_goal`** while satisfying a set of constraints that grow from part (a)
to part (c).

---

## Core Mathematical Tools

### Tool Tip Jacobian

The tool tip is a fixed offset from the EE. Its linear velocity in world frame:

```
v_tip = v_ee + ω_ee × r
```

where `r = R_ee * [0; 0; 0.1]` is the offset in world frame. Writing
`v_ee = J_v · dq` and `ω_ee = J_ω · dq` (from the space Jacobian):

```
v_tip = J_v · dq − skew(r) · J_ω · dq
      = (J_v − skew(r) · J_ω) · dq
```

So the **3×6 tool tip Jacobian** is:

```
J_tip = J_v − skew(r) · J_ω
```

where `skew(r)` is the 3×3 skew-symmetric matrix of r:
```
skew([r1; r2; r3]) = [  0  -r3   r2 ]
                     [ r3    0  -r1 ]
                     [-r2   r1    0 ]
```

This is implemented in `tool_tip_fk.m`.

### QP-Based Velocity Control (General Form)

At each timestep, with current joint angles `q`, we solve:

```
minimize    (1/2) dq' H dq + f' dq
subject to  A · dq ≤ b          (inequality constraints)
            lb ≤ dq ≤ ub        (joint limit bounds)
```

The solution `dq` is the optimal joint displacement for this step.
Then: `q ← q + dq`.

The solver used is MATLAB's `quadprog` (Optimization Toolbox).

---

## Part (a) — Approach + Joint Limits + 3 mm Stopping Constraint

**Objective:** Drive the tool tip toward `p_goal`.

```
H = J_tip' · J_tip + λ · I       (λ: Tikhonov regularization for conditioning)
f = −J_tip' · v_d                 (drives tip toward goal)
```

where `v_d = α · (p_goal − p_tip)` is the desired Cartesian step, saturated
to a maximum step size to prevent large jumps.

**Joint limit bounds (simple bounds on dq):**
```
lb = max(q_min − q,  −dq_max · 1)
ub = min(q_max − q,  +dq_max · 1)
```

**3 mm sphere constraint:**
The tip must never leave a 3 mm sphere around `p_goal` once it enters it.
This is a *unilateral* (one-sided) constraint that activates when
`‖p_tip − p_goal‖ ≤ 0.003`:

```
n̂' · J_tip · dq ≤ 0
```

where `n̂ = (p_tip − p_goal) / ‖p_tip − p_goal‖` is the outward radial
unit normal from the goal. This inequality says: once inside the 3 mm
sphere, the tip's radial velocity away from the goal must be ≤ 0.
The tip can spiral closer to `p_goal` but cannot escape back out.

---

## Part (b) — Adds Tool Shaft Direction Minimization

Same as (a), with one additional cost term that penalizes **transverse
angular velocity of the tool** (i.e., tilting or spinning the shaft while
moving).

The tool axis in world frame: `d = R_ee * [0; 0; 1]`

The component of angular velocity *perpendicular* to the tool axis:
```
ω_perp = (I − d·d') · J_ω · dq
```

This is the part that actually changes the direction the tool is pointing.
We add it to H with weight μ:

```
H = J_tip' · J_tip + λ · I + μ · J_ω_perp' · J_ω_perp
```

where `J_ω_perp = (I − d·d') · J_ω`.

**Intuition:** In tasks like drilling or needle insertion, the tool shaft
direction matters — you don't want the drill bit tilting as it advances.
This term makes the robot prefer to translate the tool tip along the
approach axis rather than re-orient while moving.

---

## Part (c) — Bonus: Virtual Wall (+20 pts)

A planar *virtual wall* is defined by a point `p_wall` and outward normal
`n_wall`. It acts as a hard constraint: the tool tip cannot penetrate the
wall from the positive side.

When the tip is within a margin of the wall (or touching it), add:
```
n_wall' · J_tip · dq ≥ 0     (equivalently: −n_wall' · J_tip · dq ≤ 0)
```

The goal point `p_goal` is placed on the far side of (or very near) the
wall so that the wall actively deflects the trajectory.

The wall constraint applies in both (a)-mode and (b)-mode.

---

## Part (d) — Comparison

Plots and metrics for all parts / configurations:
- 3D tool tip trajectory
- Distance to `p_goal` vs. step number
- Joint angles vs. step (verify no limit violations)
- Tool shaft deviation angle vs. step (compare a vs. b)
- Multiple `p_goal` locations and (for c) multiple wall positions

---

## File Structure

All code lives in `THA4/`:

| File | Purpose |
|------|---------|
| `tool_tip_fk.m` | Tool tip position + 3×6 linear Jacobian |
| `QP_step_VF.m` | Single QP step — unified for parts a/b/c |
| `simulate_VF.m` | Control loop: runs N steps, returns trajectory |
| `test_THA4.m` | Master test/comparison script with all plots |
| `THA4_notes.md` | This file — design notes and running log |

---

## Implementation Log

### 2026-04-28 — Initial design
- Confirmed robot: KR120_params (Quantec Nano). KR210_params in repo is legacy.
- Optimization Toolbox installed (MATLAB R2025b). `quadprog` confirmed working.
- Key insight: the 3 mm constraint is a *unilateral* half-space constraint
  (not a two-sided equality). It only activates once the tip enters the 3 mm
  sphere and prevents exit — the tip can still approach p_goal freely.
- For part (b): penalizing `(I − d·d') · J_ω` in the quadratic cost is
  equivalent to adding a secondary null-space task for tool orientation
  stability — connects back to the null-space projection idea from THA2's
  `redundancy_resolution.m`. For a 6-DOF robot this is a *soft* objective
  (a weight term), not a null-space projection, since the robot is square.

### 2026-04-28 — Bug found: Space Jacobian linear velocity convention

When building the tool-tip Jacobian, the first instinct was:

```
J_tip = J_v − skew(r) · J_omega          ← WRONG
```

where `r = R_ee * [0;0;0.1]` is the offset from EE to tip. This produced
errors of ~2.7 m/rad (same order as the arm length) in the finite-difference
Jacobian check.

The fix:

```
J_tip = J_v − skew(p_tip) · J_omega      ← CORRECT
```

The root cause: the **space Jacobian's linear part** `J_v` does NOT give the
linear velocity of the EE origin. It gives the velocity of the *body point
instantaneously coincident with the world origin* — a standard result from
the screw theory formulation in Lynch & Park. The velocity of any body point
at **absolute** position `p` (world frame) is therefore:

```
v_p = v_s + ω × p   →   J_linear(p) = J_v − skew(p) · J_omega
```

Using the *relative offset* `r` instead of `p_tip` only works when the
world origin and the EE origin coincide (home config with p_ee = 0), which
is never the case for a real arm. After the fix, Jacobian errors dropped
from ~2.7 to ~3×10⁻⁹ m/rad across all tested configurations.

**Lesson for presentation:** This is a classic pitfall of the spatial velocity
representation. Body-frame Jacobians (`J_body`) don't have this issue because
the linear part is already expressed relative to the EE frame — but spatial
Jacobians require using absolute position.

### 2026-04-28 — Results from full simulation (initial 6 runs)

Early results before scenario refactor — superseded by the 12-run table below.

### 2026-04-29 — Refactored to scenario list, added Cfg 3 and Cfg 4

`test_THA4.m` now uses a scenario struct array. Adding a new test case is a
one-line addition. Six scenarios run × 2 modes = 12 simulations.

All results: KR120, 100 mm tool, λ=1e-4, μ=0.01 (mode b), step=20 mm/iter,
dq_max=0.05 rad, sphere radius=3 mm.

| Scenario | Description | Mode | Steps | Final dist | Shaft swing | Joints at limit |
|----------|-------------|------|-------|-----------|-------------|-----------------|
| Cfg1 | Baseline, mid-workspace | (a) | 51  | 0.0001 mm | 45.5° | — |
| Cfg1 |                         | (b) | 51  | 0.0017 mm | **5.7°** (−87%) | — |
| Cfg2 | Different goal direction | (a) | 39  | 0.0012 mm | 25.6° | — |
| Cfg2 |                          | (b) | 39  | 0.0012 mm | **1.7°** (−93%) | — |
| Cfg3 | Joint-limit pressure (J2+) | (a) | 191 | 213.9 mm (limit-blocked) | 65.8° | **J2+** |
| Cfg3 |                            | (b) | 181 | 213.8 mm (limit-blocked) | 65.0° (−1%) | **J2+** |
| Cfg4 | Reorientation-heavy        | (a) | 179 | 0.0003 mm | 60.3° | — |
| Cfg4 |                            | (b) | 179 | 0.0098 mm | **5.4°** (−91%) | — |
| Cfg5 | Wall blocks goal           | (a) | 149 | 214.3 mm (wall-blocked) | 46.5° | — |
| Cfg5 |                            | (b) | 149 | 213.6 mm (wall-blocked) | **4.5°** (−90%) | — |
| Cfg6 | Wall as approach limiter   | (a) | 48  | 0.023 mm | 46.6° | — |
| Cfg6 |                            | (b) | 48  | 0.045 mm | **4.1°** (−91%) | — |

**Zero joint limit *violations*** anywhere — Cfg3 reaches its J2+ limit
(intended) and the QP correctly clamps further increase.

### Key observations from the 4 free-approach configs

1. **Mode (b) shaft stabilisation is consistently 87–93% effective** when
   joint limits do *not* dominate (Cfg1, Cfg2, Cfg4). Convergence speed is
   identical to mode (a) in every case.

2. **Cfg3 reveals a limit of part (b):** when the goal is unreachable due to
   a joint limit, the robot converges to the constrained-optimal pose. At
   that pose, the shaft direction is *forced* by geometry — the μ term has
   almost no leverage to stabilise it (65.8° → 65.0°, only 1% reduction).
   This is not a failure of mode (b); it is a fundamental observation that
   the secondary objective only matters when there is slack in the primary
   constraints.

3. **Cfg4 is the best showcase for part (b):** the goal `[-0.5, 1.5, 1.2]`
   sits behind and to the side of the robot, requiring a large J1 swing
   and major wrist reorientation. Mode (a) swings the shaft through 60°.
   Mode (b) keeps it within 5.4° using the same 179-step convergence.

4. **Cfg3 numerically validates the joint-limit constraint** — without the
   bound clamp the QP would push J2 well past +0.611 rad. With the bound
   active, J2 saturates at +0.611 and the remaining motion is distributed
   to other joints. Final tip position [1.47, 0, −1.29] is the closest
   the wrist centre can get to [1.5, 0, −1.5] with J2 ≤ +0.611.

### Bug found and fixed during scenario design

`Cfg3` initially used start `q0 = [0, 0.4, 0.3, 0, 0.5, 0]` (J2 = +0.4),
which was too close to the +0.611 limit and caused the *initial* QP step
to be marginally infeasible (the bounds `lb`/`ub` clipped harshly). Moved
`q0` to `[0, 0, 0.3, 0, 0, 0]` and chose a deeper goal `[1.5, 0, −1.5]`
so the limit activates *during* the trajectory, not at step 0. Better
demonstration.

Key observations:
- The 3 mm sphere constraint activates at step ~50 for Config 1 and holds
  the tip within 3 mm for all subsequent steps.
- Part (b) dramatically reduces shaft swing with essentially no cost in
  convergence speed or final accuracy. For Config 2 (93% reduction), the
  shaft barely moves at all during the approach.
- **Wall 1** (wall between start and goal): the robot is deflected at the
  wall boundary (~Y=0.18) and converges to the closest accessible point,
  ~214 mm from p_goal. This shows the VF controller gracefully finding
  the constrained-optimal position rather than stalling or diverging.
- **Wall 2** (goal accessible, wall nearby): the robot reaches p_goal
  to within 0.023 mm while the wall constrains the final approach angle,
  preventing the tip from swinging past Y=0.35 at any point.

### Notes on virtual wall design (part c)

A common misconception: a planar VF "deflects" the path while the robot
still reaches the goal. For a flat half-space constraint and a **directly
blocked** goal (goal on wrong side), the robot converges to the closest
feasible point ON the wall boundary — it cannot reach the original goal.

For Wall 1 this is intentional: it shows the VF acting as a *hard barrier*
(like a physical surface). The robot demonstrates correct constrained
convergence behaviour, not failure.

For Wall 2, the goal is inside the safe half-space, and the wall acts as
an *approach limiter* — the robot can still reach p_goal, but any attempt
to swing past Y=0.35 during the trajectory is blocked. The path is subtly
different from the unconstrained case.

---

*This file is updated as implementation progresses.*
