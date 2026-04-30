# THA4 Submission Outline
**ME384R — Algorithms for Sensor-Based Robotics**
**Prof. Farshid Alambeigi — Spring 2026**
**Due: 2026-04-30, 3:30 PM**

> Markers: `[FIG #]` = recommended figure · `[TBL #]` = recommended table · `[ANIM #]` = recommended animation/video

---

## 0. Cover Page (provided by professor)

- Names / EIDs / Emails (both partners if applicable)
- Signed honour statement (signature required)
- Score sheet appended as first sheet
- Reference: HW4-main.pdf page 1

---

## 1. Homework Assignment (HA 1) — 50 pts

### Prompt
> *In W14-L1, we derived admittance-based VFs for Tubular and cone types VFs. Extend these algorithms for Impedance-based VFs. Write appropriate equations and describe how you implement them in real applications.*

### Suggested Section Layout

**1.1 Background: Admittance vs. Impedance Control**
- Brief recap of the admittance control law from W14-L1 (force-in → motion-out): the robot reads applied force `F`, computes a desired velocity `v_d` consistent with the VF, and tracks `v_d`.
- Definition of impedance control (motion-in → force-out): the robot measures motion deviation from the VF, and outputs a wrench `F` that restores it.
- Why the choice matters: admittance fits stiff robots; impedance fits backdrivable / direct-drive robots.
- `[FIG 1]`: side-by-side block diagrams of admittance vs. impedance control loops.

**1.2 Tubular VF — Admittance Form (recap from W14-L1)**
- Parametric tube definition: axis line `L(s) = p₀ + s·t̂`, allowed-motion direction `t̂`, forbidden-motion span `n̂₁, n̂₂`, radius `r`.
- Admittance law writes the constrained velocity as the projection of user input onto the allowed subspace, plus a soft return-to-axis term.

**1.3 Tubular VF — Impedance Form (extension)**
- Define a stiffness `K` and damping `B` in the *forbidden* directions (`n̂₁, n̂₂`).
- Restoring wrench:
  ```
  F_imp = −K · (p_tip − p_axis)_⊥  −  B · (v_tip)_⊥
  ```
  where `(·)_⊥` denotes the component perpendicular to `t̂`.
- Tangent direction `t̂` left compliant (zero stiffness) so the user can slide along the tube freely.
- Show the equivalent for the *outer cylindrical wall* of the tube (one-sided spring activated only when `‖(p−p_axis)_⊥‖ > r`).

**1.4 Conical VF — Impedance Form**
- Parametric cone: apex `p_a`, axis `â`, half-angle `α`.
- Define forbidden region as `angle(p − p_a, â) > α`.
- Impedance law: project `(p − p_a)` onto the cone surface; apply spring/damper between current point and its projection.
- One-sided activation: zero force inside the cone, restoring force as the tip exits.

**1.5 Implementation in Real Applications**
- Sensing: requires high-bandwidth force/torque sensor or torque-controlled joints; impedance form needs accurate `J(q)ᵀ · F` mapping.
- Stability: discuss the contact-stability concern (Hogan's passivity argument); mention sample-rate / virtual-stiffness limits.
- Comparison summary: when to prefer admittance (heavy industrial arm, no torque control) vs. impedance (collaborative/medical arm, contact-rich tasks).
- Application examples: needle insertion (cone VF), tubular endoscopy navigation (tubular VF), surgical drilling (cone VF defining safe approach).

`[TBL 1]`: side-by-side comparison: admittance vs. impedance forms for tubular and cone VFs (rows = quantity sensed, quantity output, stiffness location, suitability).

---

## 2. Programming Assignment (PA) — 100 pts + 20 bonus

### 2.1 Robot and Tool Setup

- Robot: KUKA KR120 R2500 Pro (Quantec Nano), 6-DOF spherical wrist
- Tool: cylindrical, 100 mm length × 5 mm diameter, attached along the EE z-axis
- Tool tip position in world frame: `p_tip = p_ee + R_ee · [0;0;0.1]`

`[FIG 2]`: rendering of the KR120 with the cylindrical tool — annotated dimensions (link lengths a₁, a₂, a₃, d₁, d₆, tool length 100 mm, tool diameter 5 mm). Could be a screenshot of the FK_space visualization with tool drawn on top.

`[TBL 2]`: KR120 link parameters and joint limits (rows = J1..J6, columns = axis, limits in degrees).

### 2.2 Mathematical Background

**2.2.1 Forward Kinematics (PoE form)**
- Brief recap from THA2: `T_ee(q) = exp([S₁]θ₁) · ... · exp([S₆]θ₆) · M`
- All screw axes `S_i` defined in the space frame at the home configuration

**2.2.2 Tool-Tip Linear Jacobian**
- Goal: relate joint velocity `dq` to tool tip Cartesian velocity `v_tip`
- Decompose space Jacobian: `J_s = [J_omega; J_v]`
- Velocity of any body point at world position `p`:
  ```
  v_p = v_s + ω_s × p   →   J_lin(p) = J_v − skew(p) · J_omega
  ```
- **Important pitfall to discuss:** `J_v` is NOT the velocity of the EE origin — it's the velocity of the body point instantaneously at the world origin. Using the offset `r = R_ee · [0;0;0.1]` instead of the absolute `p_tip` gives a spurious error of order `‖p_ee‖`. We caught this with a finite-difference Jacobian check (see [`THA4_notes.md`](THA4_notes.md)).

`[FIG 3]`: schematic showing the spatial-velocity definition — body point at world origin vs. body point at the EE — with the cross-product `ω × p_tip` term illustrated.

**2.2.3 Constrained Optimisation Formulation (W15-L1)**
- The general velocity-level QP control loop:
  ```
  min_dq    ½ dq' H dq + f' dq
  s.t.      A_ineq · dq ≤ b_ineq
            lb ≤ dq ≤ ub
  ```
- Update rule: `q ← q + dq`, repeat until convergence

`[FIG 4]`: block diagram of the QP control loop (current state → QP setup → quadprog → joint update → FK).

### 2.3 Part (a) — Joint Limits + 3 mm Sphere

**2.3.1 Objective derivation**
- Desired Cartesian step: `v_d = α · (p_goal − p_tip)`, saturated to a max step length
- Quadratic cost: `½‖J_tip · dq − v_d‖² + ½λ‖dq‖²` → expand to get H, f
  - `H = J_tip' J_tip + λ I`
  - `f = −J_tip' v_d`

**2.3.2 Joint limit constraints**
- Simple bounds on `dq`: `lb = max(q_min − q, −dq_max)`, `ub = min(q_max − q, dq_max)`

**2.3.3 The 3 mm sphere constraint**
- Unilateral half-space: activates when `‖p_tip − p_goal‖ ≤ 3 mm`
- Outward radial unit vector: `n̂_out = (p_tip − p_goal) / ‖p_tip − p_goal‖`
- Constraint: `n̂_out' · J_tip · dq ≤ 0` — once inside the sphere, no outward radial velocity allowed
- Discuss: why unilateral and why outward — it is a *retention* constraint, not a *forbidden region*

`[FIG 5]`: 2D cartoon of the sphere constraint — tip approaching p_goal, the 3 mm sphere drawn around p_goal, with the outward normal `n̂_out` and the half-plane of allowed dq directions shaded.

**2.3.4 Test functions**
- `tool_tip_fk.m` — verify against finite-difference Jacobian
- `QP_step_VF.m` (mode a)
- `simulate_VF.m` — runs to convergence

**2.3.5 Results — Configuration 1**
- Start: `q₀ = [0, −0.4, 0.5, 0, 0.3, 0]`, tip at `[2.690, 0, 0.860]`
- Goal: `[1.8, 0.4, 1.0]`, start distance 986 mm
- Converged in **51 steps**, final dist **0.0001 mm**, no joint limit violations
- 3 mm constraint activated at step ~50, held tip inside sphere thereafter

`[FIG 6]`: 3D trajectory for part (a) Config 1 — already produced as Fig 1 in `test_THA4.m`.
`[FIG 7]`: distance-to-goal vs. step (log scale) for part (a) — already in Fig 2 of `test_THA4.m`.

**2.3.6 Results — Configuration 2 (different goal)**
- Goal: `[2.1, −0.3, 1.2]`, start distance 745 mm
- Converged in 39 steps, final dist 0.001 mm

`[TBL 3]`: results summary for parts (a) and (b) across both configurations (already drafted in `THA4_notes.md`).

### 2.4 Part (b) — Tool Shaft Direction Stabilisation

**2.4.1 Derivation**
- Tool axis in world frame: `d̂ = R_ee · [0;0;1]`
- Angular velocity component perpendicular to `d̂` is the part that *changes* the shaft direction
- Define `J_perp = (I − d̂ · d̂') · J_omega`
- Add cost term: `½μ‖J_perp · dq‖²` → adds `μ · J_perp' · J_perp` to H

**2.4.2 Connection to redundancy resolution (THA2)**
- For 6-DOF robot the null space is empty, so we cannot project a *secondary task* into the null space (as in THA2 `redundancy_resolution.m`). Instead we use a **soft secondary objective** weighted by μ — same principle, different mechanism.

**2.4.3 Results**
- Config 1: shaft swing reduced **45.5° → 5.7° (87.5% reduction)**, identical 51-step convergence
- Config 2: shaft swing reduced **25.6° → 1.7° (93% reduction)**

`[FIG 8]`: tool shaft angle vs. world Z over the trajectory — comparing (a) vs. (b), both configurations. Already produced as Fig 4 in `test_THA4.m`.

`[ANIM 1]`: animated robot pose during approach for Config 1, side-by-side (a) vs (b). Visually shows the tool wagging in (a) and staying steady in (b). **High value for the presentation bonus.**

### 2.5 Part (c) — Virtual Wall (+20 bonus)

**2.5.1 Planar wall definition**
- Wall: point `p_wall`, outward normal `n̂_wall` (points toward the safe side / robot)
- Half-space membership: `n̂_wall' (p − p_wall) ≥ 0` ⇒ tip is on the safe side
- Linear constraint: when `n̂_wall' (p_tip − p_wall) ≤ margin`, add `−n̂_wall' · J_tip · dq ≤ 0` to the QP

**2.5.2 Two scenarios chosen**
- **Wall 1** (blocking wall): goal `[1.8, 0.4, 1.0]`, wall at Y = 0.20 with `n̂ = [0,−1,0]`. Goal is on the unsafe side; robot converges to closest-feasible point near Y = 0.18. Demonstrates how the VF redirects the trajectory and gracefully terminates when the goal is unreachable.
- **Wall 2** (approach-limiting wall): goal `[1.8, 0.25, 1.0]`, wall at Y = 0.35. Goal is reachable; the wall constrains overshoot during approach. Demonstrates the wall acting as a soft-stop.

`[FIG 9]`: 3D trajectory for Wall 1 (Config 1 a/b/c-a/c-b overlaid with wall plane and 3 mm sphere). Already produced as Fig 1 in `test_THA4.m`.

`[FIG 10]`: Y-position vs. step for Wall 1 — clearly shows the deflection at Y = 0.18. Already produced as Fig 5.

`[FIG 11]`: 3D trajectory + Y-trajectory for Wall 2 (Fig 6 in `test_THA4.m`).

`[ANIM 2]`: animation of the robot approaching p_goal through Wall 1 — shows the tip getting deflected at the wall and converging along the wall surface. **Excellent for presentation.**

### 2.6 Part (d) — Comparison and Discussion

`[TBL 4]`: master comparison table: rows = run (a Cfg1, b Cfg1, a Cfg2, b Cfg2, c-a Wall1, c-b Wall1, c-a Wall2, c-b Wall2), columns = steps, final dist, shaft swing (deg), max per-step shaft change, max joint deviation from limit, # constraints active at convergence.

**2.6.1 Convergence comparison**
- Free-approach (a/b) converges in 39–51 steps regardless of configuration
- Wall-blocked (Wall 1) terminates around step 149 due to no-progress detection — the robot has reached the constrained optimum and is making sub-mm oscillations against the wall margin
- Joint limits never violated (exit flag from quadprog = 1 in all 800+ QP calls)

`[FIG 12]`: joint angles vs. step for all 6 joints, Config 1, all 4 modes (a/b/c-a/c-b) — already produced as Fig 3 in `test_THA4.m`.

**2.6.2 Tool shaft preservation**
- Quantitative effect of μ: small μ (0.01) gives ~90% reduction in swing with negligible cost in convergence speed
- For Config 2 the effect is even more pronounced because the natural path requires more tool reorientation

**2.6.3 Wall behaviour**
- Hard-blocking wall converges to the projection of p_goal onto the safe half-space minus the activation margin
- Soft-stop wall (Wall 2) achieves goal to within 0.05 mm while preventing transient overshoot

**2.6.4 Sensitivity / robustness comments**
- Tikhonov regulariser λ = 1e−4 prevents singularities (no SVD needed inside the QP)
- Step saturation `step = 0.02 m` and `dq_max = 0.05 rad` keep the linearisation valid
- Wall margin choice: too small ⇒ chattering at the boundary; too large ⇒ premature stopping. We use 2 cm as a stable middle ground.

`[FIG 13]` (optional): velocity / step-size profile `‖dq‖` vs. step for each run — demonstrates smooth convergence in (a/b) and the sudden drop when the wall activates in (c).

### 2.7 Implementation Notes

**2.7.1 File organisation**
- `tool_tip_fk.m` — tool tip FK + linear Jacobian
- `QP_step_VF.m` — single QP step (parts a/b/c via opts struct)
- `simulate_VF.m` — control loop with three convergence checks
- `test_THA4.m` — reproduces all results and figures

**2.7.2 Use of the Optimization Toolbox**
- `quadprog` is the only external solver call; the *formulation* of H, f, A, b, lb, ub is entirely ours
- Justification: this is a numerical solver analogous to `\` for linear systems; rewriting it would be an entire optimisation course in itself

**2.7.3 Reproducibility**
- All results come from running `test_THA4.m` from the project root with the THA4 folder added to the path

---

## 3. Presentation Materials (for +20 bonus, optional)

If presenting on April 30:

- 8–10 slides
- Slide 1: title / team / problem statement
- Slide 2: setup figure (`[FIG 2]`)
- Slide 3: QP formulation (one-slide derivation)
- Slide 4: part (a) result + 3 mm sphere figure
- Slide 5: part (b) result with shaft swing comparison + animation
- Slide 6: part (c) result with wall animation
- Slide 7: comparison table
- Slide 8: lessons learned (Jacobian pitfall + virtual fixture geometry choices)
- Slide 9: extension ideas / questions

---

## 4. References

- Lynch, K. M., & Park, F. C. *Modern Robotics: Mechanics, Planning, and Control.* Cambridge University Press, 2017. (PoE FK and Jacobian formulations)
- Course lectures: W14-L1 (admittance-based VFs), W15-L1 (constrained-optimisation control)
- KUKA Deutschland GmbH, *KR 120 R2500 Pro Technical Data*
- MATLAB Optimization Toolbox documentation: `quadprog`
- (HA1) Hogan, N. *Impedance Control: An Approach to Manipulation.* J. Dynamic Systems, Measurement, and Control, 1985.
- (HA1) Abbott, J. J. & Okamura, A. M. *Virtual Fixture Architectures for Telemanipulation.* ICRA 2003 (or similar VF reference cited in lecture).

---

## 5. Submission Checklist

- [ ] Cover/score sheet signed and on top
- [ ] HA 1 derivation with figures
- [ ] PA report (Sections 2.1 – 2.6)
- [ ] All MATLAB source files (`THA4/*.m`) included in the submission archive
- [ ] All required figures present
- [ ] References section
- [ ] Submitted to Canvas before 3:30 PM, 2026-04-30
- [ ] (Optional) Notify professor by April 29 if presenting

---

## Figures / Tables / Animations — Production Plan

| ID | Type | Source | Effort |
|----|------|--------|--------|
| FIG 1 | Diagram | TikZ / hand-drawn / draw.io | low |
| FIG 2 | Robot rendering | Modify `FK_space` visualization, draw tool cylinder | low |
| FIG 3 | Schematic | hand-drawn / draw.io | low |
| FIG 4 | Block diagram | draw.io | low |
| FIG 5 | Cartoon | hand-drawn | very low |
| FIG 6 | 3D plot | already in `test_THA4.m` Fig 1 | none |
| FIG 7 | Distance plot | already in `test_THA4.m` Fig 2 | none |
| FIG 8 | Shaft angle plot | already in `test_THA4.m` Fig 4 | none |
| FIG 9 | 3D plot Wall 1 | already in `test_THA4.m` Fig 1 | none |
| FIG 10 | Y-trajectory plot | already in `test_THA4.m` Fig 5 | none |
| FIG 11 | 3D + Y plot Wall 2 | already in `test_THA4.m` Fig 6 | none |
| FIG 12 | Joint angle plot | already in `test_THA4.m` Fig 3 | none |
| FIG 13 | Velocity profile plot | new — small addition to test script | low |
| ANIM 1 | (a) vs (b) shaft animation | new — needs robot-pose animator | medium |
| ANIM 2 | Wall 1 deflection animation | new — same animator | medium |
| TBL 1 | Adm vs Imp summary | hand-write | low |
| TBL 2 | KR120 specs | extract from `KR120_params.m` | very low |
| TBL 3 | (a/b) results | already drafted in notes | none |
| TBL 4 | Master comparison | new — extend the existing summary | low |

**Recommendation:** the highest-leverage new artefacts are `[ANIM 1]` and `[ANIM 2]` — they're worth the most for a presentation and give a strong visual sense of the QP control's behaviour. We already have `robot_animation.m` and `ik_animation.m` in the repo from THA2 — those can probably be adapted.

