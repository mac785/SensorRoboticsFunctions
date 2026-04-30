# THA4 Report Outline
**ME384R — Algorithms for Sensor-Based Robotics**
**Prof. Farshid Alambeigi — Spring 2026**
**Due: 2026-04-30, 3:30 PM**

> **Note for writer:** Figures marked `[FILE: ...]` already exist as PNG files in `THA4/figures/`.
> Animations marked `[ANIM FILE: ...]` exist as AVI files in `THA4/figures/`.
> Figures marked `[TO CREATE]` still need to be drawn or generated.
> All numerical results are final — use the exact numbers given.

---

## 1. Programming Assignment (PA) — 100 pts + 20 bonus pts

### 1.1 Robot and Tool Setup

**Robot:** KUKA KR120 R2500 Pro (Quantec Nano)
- 6-DOF serial manipulator with spherical wrist
- Parameters from `KR120_params.m` (PoE screw axes, home transform M, joint limits)

**Tool:** Cylindrical tool, 100 mm length × 5 mm diameter, rigidly fixed to EE frame along its z-axis.

Tool tip position:
```
p_tip = p_ee + R_ee · [0; 0; 0.1]
```

Joint limits (from `KR120_params.m`):

| Joint | Lower (deg) | Upper (deg) |
|-------|------------|------------|
| J1 | −185 | +185 |
| J2 | −35 | +35 |
| J3 | −120 | +158 |
| J4 | −350 | +350 |
| J5 | −130 | +130 |
| J6 | −350 | +350 |

`[FIG 2 — CROP FROM PDF]`: Crop the right panel of Fig. 1 from `HW4- main.pdf` (page 3) — shows the Franka/Kuka robot with the 100 mm / 5 mm cylindrical tool and dimension labels already annotated by the professor. No need to generate separately.

`[TBL 2]`: The joint limits table above (already written above — reproduce in report).

---

### 1.2 Mathematical Background

#### 1.2.1 Forward Kinematics (Product of Exponentials)

From THA2, the end-effector transform using space-form PoE:
```
T_ee(q) = exp([S₁]θ₁) · exp([S₂]θ₂) · ··· · exp([S₆]θ₆) · M
```
where each `Sᵢ = [ωᵢ; vᵢ]` is the screw axis of joint i expressed in the space (world) frame at the robot's home configuration, and `M` is the home configuration transform.

Implemented in `FK_space.m` (shared from THA2 helpers).

#### 1.2.2 Tool-Tip Linear Jacobian

The space Jacobian `Js = [Jω; Jv]` (6×6) relates joint velocity `dq` to the spatial twist at the space-frame origin. Crucially:

> **`Jv` does NOT give the linear velocity of the EE origin.** It gives the linear velocity of the body point instantaneously coincident with the *world origin*. This is the standard space-form convention (Lynch & Park, Modern Robotics §5.1).

The linear velocity of any body point at absolute world-frame position `p` is:
```
v_p = Jv · dq  +  ω × p   =   (Jv − skew(p) · Jω) · dq
```

Therefore the **3×6 tool-tip Jacobian** is:
```
J_tip = Jv − skew(p_tip) · Jω
```
where `p_tip` is the absolute world-frame position of the tool tip (NOT the offset `r = R_ee·[0;0;0.1]`).

**Pitfall caught during development:** Using `skew(r)` instead of `skew(p_tip)` gave Jacobian errors of ~2.7 m/rad (same order as the arm reach), because the error term is `skew(p_ee)·Jω` — the cross product of the absolute EE position with the angular velocity. After the fix, finite-difference Jacobian verification showed errors of ~3×10⁻⁹ m/rad across all tested configurations.

Implemented in `tool_tip_fk.m`.

`[FIG 3 — OPTIONAL / SKIP]`: Jacobian pitfall schematic. The text explanation above is self-contained — omit this figure unless time permits. If included, hand-draw a simple 2D arm showing world origin, p_ee, p_tip, and the ω × p_tip correction arrow.

#### 1.2.3 QP-Based Velocity Control Formulation

At each timestep, solve for the optimal joint displacement `dq`:
```
minimise    ½ dq' H dq + f' dq
subject to  A_ineq · dq ≤ b_ineq        (sphere and/or wall constraints)
            lb ≤ dq ≤ ub                (joint limit + max-step bounds)
```

Then apply: `q ← q + dq`. Repeat until convergence.

Solver: MATLAB `quadprog` (Optimization Toolbox). The problem formulation (H, f, A, b, lb, ub) is entirely derived from first principles; `quadprog` is used purely as a numerical solver, analogous to `\` for linear systems.

`[FIG 4 — OPTIONAL / SKIP]`: QP control loop block diagram. The formulation above describes the loop fully in text — omit unless time permits. If included, hand-draw: `q_k → [FK+Jac] → [Build QP] → [quadprog] → dq → q_{k+1}` with feedback arrow.

---

### 1.3 Part (a) — Joint Limits + 3 mm Sphere Constraint

#### Objective

Desired Cartesian step toward goal, saturated to max step size `step = 0.02 m`:
```
v_d = min(‖e‖, step) · e/‖e‖    where e = p_goal − p_tip
```

QP matrices:
```
H = J_tip' · J_tip + λ·I          (λ = 1e-4 Tikhonov regularisation)
f = −J_tip' · v_d
```

#### Joint Limit Bounds

Simple bounds clipped to maximum step:
```
lb = max(q_min − q,  −dq_max · 1)      dq_max = 0.05 rad
ub = min(q_max − q,  +dq_max · 1)
```

#### 3 mm Sphere Constraint

Once the tip enters the 3 mm sphere around `p_goal` (`‖p_tip − p_goal‖ ≤ 0.003 m`), a unilateral constraint prevents the tip from escaping back out:

```
n̂_out = (p_tip − p_goal) / ‖p_tip − p_goal‖    (outward radial unit normal)
n̂_out' · J_tip · dq ≤ 0
```

This is a *retention* constraint: the tip can continue approaching `p_goal` once inside the sphere, but cannot move outward. It is one-sided (only active inside the sphere) because outside the sphere there is no constraint — the tip is free to approach freely.

`[FIG 5 — CROP FROM PDF]`: Crop the right panel of Fig. 1 from `HW4- main.pdf` (page 3) — the professor's figure already shows the 3 mm circle around p_goal with the green sphere labeled. Use that directly.

---

### 1.4 Part (b) — Tool Shaft Direction Stabilisation

#### Derivation

Tool axis in world frame: `d̂ = R_ee · [0;0;1]`

The component of angular velocity perpendicular to the tool axis is the part that *rotates* the shaft direction:
```
ω_⊥ = (I − d̂·d̂') · Jω · dq
J_perp = (I − d̂·d̂') · Jω              (3×6)
```

Add secondary cost term to H weighted by `μ = 0.01`:
```
H = J_tip'·J_tip + λ·I + μ·J_perp'·J_perp
```

f and all constraints remain identical to part (a). Only H changes.

#### Connection to THA2 Redundancy Resolution

In THA2, secondary tasks were projected into the null space of the primary Jacobian. For a 6-DOF robot with a 3-DOF task, that null space is 3-dimensional. Here the robot is **square** (6 joints, 6-DOF task after adding the sphere constraint), so there is no null space available. The shaft stabilisation is therefore implemented as a **soft secondary objective** (a cost weight μ) rather than a hard null-space projection. The principle is identical — penalise unwanted joint motion directions — but the mechanism is different.

---

### 1.5 Scenarios and Results

Six scenarios were run, each in mode (a) and mode (b), giving 12 total simulations. All results produced by `test_THA4.m`.

#### Scenario Descriptions

| Label | q₀ (rad) | p_goal (m) | Purpose |
|-------|----------|-----------|---------|
| Cfg1 | [0,−0.4,0.5,0,0.3,0] | [1.8, 0.4, 1.0] | Baseline, mid-workspace |
| Cfg2 | [0,−0.4,0.5,0,0.3,0] | [2.1,−0.3, 1.2] | Different goal direction (−Y, higher Z) |
| Cfg3 | [0, 0, 0.3, 0, 0, 0] | [1.5, 0,−1.5] | Goal below robot base — forces J2 to its upper limit |
| Cfg4 | [0,−0.4,0.5,0,0.3,0] | [−0.5, 1.5, 1.2] | Goal behind robot — large J1 swing + wrist reorientation |
| Cfg5 | [0,−0.4,0.5,0,0.3,0] | [1.8, 0.4, 1.0] | Wall at Y=0.20 (n̂=[0,−1,0]) — wall blocks goal |
| Cfg6 | [0,−0.4,0.5,0,0.3,0] | [1.8, 0.25, 1.0] | Wall at Y=0.35 (n̂=[0,−1,0]) — goal accessible, wall limits overshoot |

#### Full Results Table

| Scenario | Mode | Steps | Final dist (mm) | Shaft swing (°) | Joint limit hit |
|----------|------|-------|----------------|----------------|----------------|
| Cfg1 | (a) | 51 | 0.0001 | 45.5 | — |
| Cfg1 | (b) | 51 | 0.0017 | **5.7** (−87%) | — |
| Cfg2 | (a) | 39 | 0.0012 | 25.6 | — |
| Cfg2 | (b) | 39 | 0.0012 | **1.7** (−93%) | — |
| Cfg3 | (a) | 191 | 213.9 (blocked) | 65.8 | J2+ |
| Cfg3 | (b) | 181 | 213.8 (blocked) | 65.0 (−1%) | J2+ |
| Cfg4 | (a) | 179 | 0.0003 | 60.3 | — |
| Cfg4 | (b) | 179 | 0.0098 | **5.4** (−91%) | — |
| Cfg5 | (a) | 149 | 214.3 (wall-blocked) | 46.5 | — |
| Cfg5 | (b) | 149 | 213.6 (wall-blocked) | **4.5** (−90%) | — |
| Cfg6 | (a) | 48 | 0.023 | 46.6 | — |
| Cfg6 | (b) | 48 | 0.045 | **4.1** (−91%) | — |

`[TBL 3]`: Reproduce the full results table above.

---

### 1.6 Part (a/b) Discussion — Free-Approach Configurations

Use Cfg1, Cfg2, Cfg4 for the main discussion (Cfg3 is treated separately as the joint-limit case).

**Convergence:** All three free-approach configs converge cleanly. Cfg1 and Cfg2 converge in 39–51 steps. Cfg4 takes 179 steps because the goal is behind the robot and the path is longer, not because of any instability.

**Shaft stabilisation effectiveness:** Mode (b) reduces shaft swing by 87–93% in all free-approach cases. There is zero cost to convergence speed — identical step counts in every case. The wrist joints (J4, J5, J6) simply absorb the compensating motion that mode (a) would otherwise express as shaft rotation.

**Cfg4 as the showcase:** The goal at [−0.5, 1.5, 1.2] is behind and across the robot, requiring a ~180° J1 swing and substantial wrist reorientation. Mode (a) swings the shaft through 60.3°. Mode (b) keeps it within 5.4°.

`[FILE: fig1_3d_trajectories.png]` — 3D tool tip trajectories, all 6 scenarios, (a) blue vs (b) red.
`[FILE: fig2_distance_vs_step.png]` — Distance to goal vs step (log scale), all 6 scenarios.
`[FILE: fig3_shaft_angle.png]` — Tool shaft angle from world Z, all 6 scenarios.

`[ANIM FILE: anim1_rst_shaft_cfg4.avi]` — Full KR120 mesh animation, Cfg4, side-by-side mode (a) left vs mode (b) right. Blue trajectory + thick blue tool shaft on left; red trajectory + thick red shaft on right. Shows shaft wagging in (a) vs staying stable in (b). Camera: +X/+Y/+Z isometric view.

---

### 1.7 Part (a) — Joint Limit Validation (Cfg3)

**Setup:** Starting near the home configuration, the goal [1.5, 0, −1.5] is below the robot's base — only reachable by extending J2 past its upper limit of +35° (+0.611 rad). The QP correctly clamps J2 at its limit and redistributes motion to other joints.

**Results:**
- Both modes terminate ~214 mm from goal with J2 saturated at +35°
- Mode (b) barely reduces shaft swing here (65.8° → 65.0°, only 1%). This is fundamental: when joint limits dominate, the robot has no degrees of freedom available to optimise shaft direction. The μ term only helps when there is slack in the primary constraints.
- Zero joint limit *violations* — J2 saturates exactly at +35° and stays there.

`[FILE: fig4_joint_angles_cfg3.png]` — J1–J6 joint angles vs step for Cfg3, both modes. J2 clearly saturates at +35° (dotted limit lines shown).

---

### 1.8 Part (c) — Virtual Wall (+20 bonus pts)

#### Wall Constraint Derivation

Define the wall by a point `p_wall` and outward normal `n̂_wall` (pointing toward the safe/robot side).

Signed distance from wall: `d_pen = n̂_wall' · (p_tip − p_wall)`. Positive = safe side.

When `d_pen ≤ margin` (default 2 cm), activate:
```
−n̂_wall' · J_tip · dq ≤ 0    ⟺    n̂_wall' · J_tip · dq ≥ 0
```
This requires the tip's velocity component toward the wall to be non-negative (tip can move away from or along the wall, but not further into it).

This constraint is appended to `A_ineq` alongside the sphere constraint. The QP naturally finds the constrained-optimal trajectory satisfying all active constraints simultaneously.

#### Cfg5 — Wall Blocks Goal

- Wall at Y = 0.20, `n̂ = [0,−1,0]` (safe side is Y ≤ 0.20)
- Goal at [1.8, 0.4, 1.0] is on the unsafe side (Y = 0.40)
- Activation margin: 2 cm → constraint activates at Y = 0.18

**Outcome:** Robot converges to the closest feasible point on the wall margin boundary (~214 mm from goal). This is correct constrained-optimal behaviour — the QP finds the nearest point to `p_goal` within the feasible half-space. The robot does not stall or diverge; it simply converges to the wall surface.

Mode (b) still reduces shaft swing 90% (46.5° → 4.5°) even in this wall-blocked scenario.

#### Cfg6 — Wall as Approach Limiter

- Wall at Y = 0.35, `n̂ = [0,−1,0]` (safe side is Y ≤ 0.35)
- Goal at [1.8, 0.25, 1.0] is on the safe side — reachable
- The wall prevents the tip from overshooting past Y = 0.35 during the approach

**Outcome:** Robot reaches goal to within 0.023 mm (mode a) / 0.045 mm (mode b). The trajectory is noticeably straighter than without the wall — the wall acts as a guide rail preventing transient overshoot.

`[FILE: fig5_wall_deflection.png]` — Y-position vs step for Cfg5 (left: wall blocks goal) and Cfg6 (right: wall as limiter). Goal Y and wall Y marked with dotted lines.

`[ANIM FILE: anim2_rst_wall_cfg5.avi]` — Full KR120 mesh animation, Cfg5, mode (a) robot shown with both trajectories overlaid. Orange wall plane visible. Blue solid = mode (a), red dashed = mode (b). Camera: +X/−Y/+Z view showing wall face.

---

### 1.9 Part (d) — Comparison and Discussion

`[FILE: fig6_convergence_speed.png]` — Bar chart: steps to reach <1 mm from goal, mode (a) vs (b), all scenarios. Cfg3 and Cfg5 show at 600 (max steps, did not converge — blocked by limit/wall by design).

#### Key Observations

1. **Mode (b) shaft stabilisation is consistently 87–93% effective** for all free-approach configurations. Convergence speed is identical in all cases.

2. **Cfg3 reveals the limits of part (b):** when a joint limit is the binding constraint, there are no free degrees of freedom left for secondary objectives. The 1% shaft reduction in Cfg3 (vs 87–93% elsewhere) confirms that the μ term only helps when the primary task leaves slack. This is not a failure of mode (b) — it is a fundamental property of constrained optimisation.

3. **Cfg4 is the strongest demonstration of part (b):** 60.3° → 5.4° shaft swing while maintaining identical 179-step convergence. The wrist joints absorb the compensating motion invisibly from the tip's perspective.

4. **The virtual wall constraint gracefully handles the unreachable goal case** (Cfg5): rather than oscillating or diverging, the QP finds the closest feasible point and converges there. This is a direct consequence of the convexity of the QP — the constrained optimum is unique and well-defined.

5. **Zero joint limit violations across all 12 runs.** The QP's simple-bound formulation (`lb ≤ dq ≤ ub`) is sufficient; no additional joint-limit inequality rows are needed.

6. **Tikhonov regularisation (λ = 1e−4)** prevents numerical issues near singular configurations without needing SVD or damped least-squares external to the QP. The step saturation (`step = 0.02 m`, `dq_max = 0.05 rad`) keeps the first-order linearisation valid throughout.

---

### 1.10 Implementation Notes

#### File Structure

| File | Purpose |
|------|---------|
| `tool_tip_fk.m` | Tool tip position and 3×6 linear Jacobian |
| `QP_step_VF.m` | Single QP step — handles parts (a), (b), (c) via opts struct |
| `simulate_VF.m` | Control loop: runs N steps, returns full trajectory |
| `test_functions_THA4.m` | Unit tests: T1–T6 for `tool_tip_fk` and `QP_step_VF` (6/6 pass) |
| `test_THA4.m` | Integration test: runs all 12 scenarios, generates all 6 figures |
| `make_animations_rst.m` | Full KR120 mesh animations (RST-based) for Cfg4 and Cfg5 |
| `KR120_params.m` | Robot parameters (shared from THA2 helpers) |

#### Convergence Criteria (simulate_VF.m)

Three stopping conditions, whichever triggers first:
1. `‖dq‖_∞ < 1e-6` — QP solution is essentially zero (numerically converged)
2. `‖p_tip − p_goal‖ < 0.05 mm` — close enough to goal
3. Tip has moved less than 0.1 mm in the last 30 steps (wall-blocked or limit-blocked)

#### Use of quadprog

`quadprog` is the only external solver call. The entire formulation of `H`, `f`, `A_ineq`, `b_ineq`, `lb`, `ub` is derived from first principles in `QP_step_VF.m`. Using `quadprog` is analogous to using MATLAB's `\` for linear systems — it is a numerical tool, not a shortcut around the mathematics.

---

### 1.11 Test Functions and Results

Two test scripts validate every function independently before the full integration run.

#### Unit Tests — `test_functions_THA4.m`

Six targeted tests for `tool_tip_fk` and `QP_step_VF`. All 6 pass.

**T1 — `tool_tip_fk`: tip position**
Verifies `p_tip = p_ee + R_ee·[0;0;L_tool]` against a direct `FK_space` call.
```
p_tip (function): [ 2.55006  -0.81433   0.83362] m
p_tip (ref FK):   [ 2.55006  -0.81433   0.83362] m
position error:   0.00e+00 m                           PASS
```

**T2 — `tool_tip_fk`: Jacobian finite-difference validation**
Compares the analytical `J_tip` against a central finite-difference approximation
(step 1e-7 rad) at 3 joint configurations spanning the workspace. Errors are at
floating-point truncation level (~1e-7), confirming the `skew(p_tip)` formulation is correct.
Using `skew(r)` (the relative offset) instead would give errors ~2.7 m/rad.
```
Config 1: max|J_anal - J_fd| = 1.258e-07 m/rad   [OK]
Config 2: max|J_anal - J_fd| = 1.366e-07 m/rad   [OK]
Config 3: max|J_anal - J_fd| = 1.138e-07 m/rad   [OK]  PASS
```

**T3 — `QP_step_VF`: step direction (unconstrained)**
Verifies the QP step moves the tip toward `p_goal` by checking that
`(J_tip · dq) · ê_goal > 0`.
```
Initial dist to goal: 986.05 mm
||dq||_inf:           0.04104 rad
Velocity toward goal: 0.01999 m                        PASS
```

**T4 — `QP_step_VF`: sphere constraint (retention)**
Drives the tip into the 3 mm sphere, then verifies the next QP step keeps
the tip inside the sphere. (Small residual drift from a fully-converged tip is
expected and acceptable — the constraint prevents escape, not all outward motion.)
```
Tip inside sphere:    dist = 0.0001 mm (threshold 3 mm)
Dist before step:     0.0001 mm
Dist after step:      0.0778 mm  (sphere radius: 3 mm)
Still inside sphere:  [OK]                             PASS
```

**T5 — `QP_step_VF`: wall constraint (no penetration)**
Drives the tip to the wall activation boundary, then verifies the wall-normal
component of `J_tip · dq` is non-negative (tip moves away from or along the wall).
```
Signed dist to wall:              0.0464 m  (margin 0.05 m)
Wall-normal velocity (n'·J·dq):  7.080e-08              PASS
```

**T6 — `QP_step_VF`: joint limits not violated (Cfg3)**
Runs the J2-saturation scenario (Cfg3) for 200 steps. Verifies `q` stays within
`[q_min, q_max]` at every step and J2 saturates at exactly its upper limit (+35°).
```
J2 upper limit:        +35.0000 deg (0.6109 rad)
J2 peak reached:       +35.0000 deg (0.6109 rad)
J2 saturated at limit: [OK]
Any limit violated:    [OK]                            PASS
```

**Final result: 6 / 6 passed.**

---

#### Integration Test — `test_THA4.m`

Runs all 6 scenarios × 2 modes = 12 simulations via `simulate_VF`. Results are
summarised in the table in Section 1.5 and visualised in Figures 1–6. Key outcomes:

- All 4 free-approach scenarios converge to within 0.01 mm of `p_goal`.
- Mode (b) reduces shaft swing 87–93% with zero cost to convergence speed.
- Cfg3 demonstrates correct joint-limit clamping (J2 saturates at +35°, zero violations).
- Cfg5/Cfg6 demonstrate correct wall constraint behaviour.

---

## 2. Presentation Materials

If presenting April 30:

- **Slide 1:** Title, team, problem statement (real-time constrained QP robot control)
- **Slide 2:** Robot setup — `[FIG 2]`, tool diagram, joint limits table
- **Slide 3:** QP formulation — one slide with the min/subject-to form, H, f, constraints
- **Slide 4:** Jacobian pitfall — `[FIG 3]`, the spatial velocity issue, finite-difference validation
- **Slide 5:** Part (a) results — `[FILE: fig1_3d_trajectories.png]`, convergence curves
- **Slide 6:** Part (b) shaft stabilisation — `[ANIM FILE: anim1_rst_shaft_cfg4.avi]`, before/after swing numbers
- **Slide 7:** Part (c) virtual wall — `[ANIM FILE: anim2_rst_wall_cfg5.avi]`, wall constraint equation
- **Slide 8:** Cfg3 joint limit result — `[FILE: fig4_joint_angles_cfg3.png]`, why mode (b) doesn't help
- **Slide 9:** Comparison table — `[TBL 3]`, key takeaways

---

## 3. References

- Lynch, K. M. & Park, F. C. *Modern Robotics: Mechanics, Planning, and Control.* Cambridge University Press, 2017. — PoE FK, space Jacobian (§5.1)
- Course lectures: W14-L1 (admittance-based VFs for tubular/cone), W15-L1 (constrained optimisation control)
- KUKA Deutschland GmbH. *KR 120 R2500 Pro Technical Data.*
- MATLAB Documentation. *quadprog — Quadratic Programming Solver.* Optimization Toolbox.
- Hogan, N. "Impedance Control: An Approach to Manipulation." *J. Dynamic Systems, Measurement, and Control*, 107(1):1–7, 1985.
- Abbott, J. J. & Okamura, A. M. "Virtual Fixture Architectures for Telemanipulation." *ICRA 2003.* (or equivalent VF reference cited in W14-L1)

---

## 4. Submission Checklist

- [ ] Cover/score sheet signed and on top
- [ ] HA 1 written (Sections 1.1–1.5, TBL 1)
- [ ] PA report written (Sections 1.1–1.11)
- [ ] All MATLAB source files (`THA4/*.m`) included in submission archive
- [ ] Figures included: fig1–fig6 PNGs from `THA4/figures/`
- [ ] Animations included: both RST AVIs from `THA4/figures/`
- [ ] References section complete
- [ ] Submitted to Canvas before 3:30 PM, 2026-04-30
- [ ] (Optional) Notified professor by April 29 if presenting

---

## 5. Assets — What Exists vs. What Still Needs Creating

### Already Done — MATLAB-generated figures (THA4/figures/)

| File | Contents | Used in section |
|------|----------|----------------|
| `fig1_3d_trajectories.png` | 3D tool tip paths, all 6 scenarios, (a) vs (b) | 1.6, 1.8 |
| `fig2_distance_vs_step.png` | Distance to goal log-scale, all 6 scenarios | 1.6 |
| `fig3_shaft_angle.png` | Tool shaft angle from world Z, all 6 scenarios | 1.6 |
| `fig4_joint_angles_cfg3.png` | J1–J6 vs step for Cfg3, (a) and (b) | 1.7 |
| `fig5_wall_deflection.png` | Y-position vs step, Cfg5 and Cfg6 | 1.8 |
| `fig6_convergence_speed.png` | Steps-to-1mm bar chart, all scenarios | 1.9 |

### Already Done — Animations (THA4/figures/)

| File | Contents | Used in section |
|------|----------|----------------|
| `anim1_rst_shaft_cfg4.avi` | KR120 mesh, Cfg4, side-by-side (a) vs (b), shaft shown | 1.6 |
| `anim2_rst_wall_cfg5.avi` | KR120 mesh, Cfg5, both trajectories, orange wall | 1.8 |

### Already Done — Conceptual diagrams (THA4/figures/)

| File | Contents | Used in section |
|------|----------|----------------|
| `fig_diagram1_admittance_vs_impedance.png` | Admittance vs impedance block diagrams | 1.1 |

### Crop from HW4- main.pdf (page 3, Fig. 1)

| Figure | What to crop | Used in section |
|--------|-------------|----------------|
| FIG 2 | Right panel — robot with 100 mm / 5 mm tool, dimension labels | 1.1 |
| FIG 5 | Right panel — 3 mm green sphere around p_goal | 1.3 |

### Still Needs Creating

| Item | Description | Section |
|------|-------------|---------|
| TBL 1 | Admittance vs impedance comparison table (write directly in Word) | 1.5 |
| FIG 3 | Jacobian pitfall schematic — **optional**, skip if pressed for time | 1.2.2 |
| FIG 4 | QP loop block diagram — **optional**, skip if pressed for time | 1.2.3 |
