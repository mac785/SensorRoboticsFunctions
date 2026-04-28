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

### 2026-04-28 — Results from full simulation

All results with: KR120, 100 mm tool, λ=1e-4, step=20 mm/iter, dq_max=0.05 rad.

| Run | Steps | Final dist | Shaft swing |
|-----|-------|-----------|-------------|
| (a) Config 1 | 51 | 0.0001 mm | 45.5° |
| (b) Config 1 | 51 | 0.0017 mm | **5.7°** (−87.5%) |
| (a) Config 2 | 39 | 0.0012 mm | 25.6° |
| (b) Config 2 | 39 | 0.0012 mm | **1.7°** (−93%) |
| (c-a) Wall 1 | 149 | 214 mm (wall-blocked) | — |
| (c-b) Wall 1 | 149 | 214 mm (wall-blocked) | — |
| (c-a) Wall 2 | 48 | 0.023 mm | — |
| (c-b) Wall 2 | 48 | 0.045 mm | — |

**Zero joint limit violations** in all 6 runs.

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
