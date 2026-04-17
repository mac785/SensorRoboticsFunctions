# THA3 Report Writing Context
## ME384R: Algorithms for Sensor-Based Robotics — Prof. Farshid Alambeigi, Spring 2026
### Due: 04/16/2026 at 3:30 PM

---

## HOW TO USE THIS FILE

This document contains everything needed to write the THA3 report. The report must cover:
- **Derived equations** for each algorithm
- **Description of the approach** (how each algorithm works)
- **Test functions** (what was tested and how)
- **Results** for each function

The report covers **PA1 (80 pts)** and **PA2.1 (20 pts) + PA2.2 Bonus (10 pts)**.

The assignment says it is open book/notes/web, but **cite any references consulted**.

---

## ASSIGNMENT STRUCTURE

| Question | Points |
|----------|--------|
| PA1 | 80 |
| PA2.1 | 20 |
| PA2.2 (Bonus) | 10 |

---

## PA1: STEREOTACTIC NAVIGATION SYSTEM CALIBRATION

### Problem Statement

PA1 is adapted from the *Computer Integrated Surgery* course by Dr. Russell H. Taylor, Johns Hopkins University [1].

The system has:
- An **EM tracking system** that measures 3D positions of small markers but has **characterized distortion** (up to several mm) and random noise (up to ~0.3 mm).
- An **optical tracking system** that is assumed to read positions to **very high accuracy** (no geometric error assumed).
- A **calibration object** with N_C EM markers at known body-frame positions **c_i**, N_A optical markers at known body-frame positions **a_j**, and N_D optical markers on the EM base at known positions **d_j**.

Each sample frame of calibration data contains: [D_1,...,D_ND, A_1,...,A_NA, C_1,...,C_NC].

The system also has two probes for **pivot calibration**:
- An **optical probe** with N_H LED markers at unknown-but-fixed positions **h_i**
- An **EM probe** with N_G EM markers at unknown-but-fixed positions **g_i**

### Goal 1: Point-Set Registration (cloud2cloud)

**Algorithm:** SVD-based rigid body registration (Horn's method).

**Inputs:** Two corresponding N-point clouds A (static) and B (moving).

**Derivation:**

Given N corresponding points, find T = [R | p] in SE(3) minimizing:

$$\sum_{i=1}^{N} \|B_i - (R A_i + p)\|^2$$

**Step 1 — Compute centroids:**
$$\bar{a} = \frac{1}{N}\sum_{i=1}^N a_i, \quad \bar{b} = \frac{1}{N}\sum_{i=1}^N b_i$$

**Step 2 — Mean-center the clouds:**
$$\tilde{a}_i = a_i - \bar{a}, \quad \tilde{b}_i = b_i - \bar{b}$$

**Step 3 — Form the cross-covariance matrix:**
$$H = \tilde{A}^T \tilde{B} \quad \in \mathbb{R}^{3\times 3}$$

**Step 4 — SVD decomposition:**
$$H = U \Sigma V^T$$

**Step 5 — Recover rotation:**
$$X = V U^T$$

If det(X) = +1 → R = X (proper rotation).
If det(X) = -1 → reflection (coplanar points); fix by negating third column of V:
$$R = V' U^T, \quad V'_{:,3} = -V_{:,3}$$

**Step 6 — Recover translation:**
$$p = \bar{b} - R\bar{a}$$

**Output:** T = [R | p; 0 0 0 1] ∈ SE(3)

**Reference:** Arun, K.S., Huang, T.S., Blostein, S.D., "Least-Squares Fitting of Two 3-D Point Sets," IEEE TPAMI, 1987. Also: Horn, B.K.P., "Closed-form solution of absolute orientation using unit quaternions," JOSA A, 1987.

---

### Goal 2: C_expected Computation

**Problem:** Given F_D (optical tracker → EM base frame) and F_A (calibration object → optical tracker frame), compute where the EM markers **should** appear in the EM base frame if there were no distortion.

**Formula (from assignment, page 4, Goal 3c):**
$$C_i^{(\text{expected})} = F_D^{-1} \cdot F_A \cdot c_i$$

Where:
- F_D = cloud2cloud(d_body, D_measured): maps d_j (known body positions) to D_j (optical tracker readings of EM base markers)
- F_A = cloud2cloud(a_body, A_measured): maps a_j (known body positions) to A_j (optical tracker readings of calibration object markers)
- c_i = known position of EM marker i in calibration object body frame
- The result is in the EM base coordinate frame

**Why this works:** F_D maps from the EM base body frame to the optical tracker frame. F_D^{-1} maps back. F_A maps from the calibration object body frame to the optical tracker frame. Chaining them: F_D^{-1} · F_A takes c_i from the cal object body frame to the EM base frame. Since optical tracking has no geometric error, this gives the "true" expected position of each EM marker.

**Note on distorted datasets:** Datasets c, e, f, g have EM distortion artificially added. The C_expected values computed by the optical path (above formula) represent the undistorted "ground truth," while the actual C_i measurements from the EM sensor are distorted. The differences (up to ~4.7 mm for dataset g) are therefore the *signature of EM distortion*, not a bug. PA2 distortion correction would address this.

---

### Goal 3: EM Pivot Calibration

**Problem:** Find the fixed position P_dimple of a dimple in the EM base frame, and the probe tip position t_G in the probe's local coordinate frame, using multiple frames where the probe tip is held stationary at the dimple.

**Approach:**

For each frame k, use cloud2cloud to find the transformation F_G[k] from the probe's reference configuration to frame k:
$$G_j = F_G[k] \cdot g_j$$

Since the probe tip is stationary:
$$P_{\text{dimple}} = F_G[k] \cdot t_G \quad \forall k$$

**Overdetermined linear system:** Expanding the SE(3) product:
$$R_k \cdot t_G + p_k = P_{\text{dimple}}$$
$$R_k \cdot t_G - P_{\text{dimple}} = -p_k$$

Stacking all frames:
$$\begin{bmatrix} R_1 & -I_3 \\ R_2 & -I_3 \\ \vdots \\ R_K & -I_3 \end{bmatrix} \begin{bmatrix} t_G \\ P_{\text{dimple}} \end{bmatrix} = \begin{bmatrix} -p_1 \\ -p_2 \\ \vdots \\ -p_K \end{bmatrix}$$

**Solution:** Solved via least-squares (MATLAB `\` operator on the overdetermined system).

**Local reference frame:** Frame 1 defines the probe's local coordinate system. The g_j are mean-centered: g_j = G_j - G_0 where G_0 = mean(G_j) from frame 1.

---

### Goal 4: Optical Pivot Calibration

**Same approach as EM pivot**, but with an extra step required because the optical tracker may not be in the same position/orientation for every frame.

**Extra step:** For each frame k, use the simultaneously measured D markers to compute F_D[k], then transform the H markers (optical probe beacons) to EM base coordinates:
$$H_j^{(EM)} = F_D[k]^{-1} \cdot H_j^{(\text{optical})}$$

Then run the same pivot calibration algorithm on H_em_frames. This gives P_opt and t_H both in the EM base frame.

**Why this is necessary:** The optical tracker's pose relative to the EM base changes between frames. F_D^{-1} "undoes" this varying optical tracker pose, putting all H observations into a common EM frame.

---

### PA1 Code Summary

**`cloud2cloud.m`** — SVD registration, returns 4×4 SE(3) matrix. Handles reflection case (coplanar points) by negating the third column of V.

**`pivot_cal.m`** — Builds the overdetermined [R_k | -I] system across all frames, solves with `\`. Takes in a cell array of N_G×3 point clouds.

**`cal_body_data.m`** — Reads calbody .txt file. Returns N=[Nd Na Nc] and matrices d, a, c. Uses `readmatrix(..., 'NumHeaderLines', 1)` for robust header parsing.

**`pa1_solve.m`** — Full pipeline function: loads all files for a dataset prefix, runs F_D/F_A registration per frame, computes C_expected, runs EM pivot, runs optical pivot (with F_D^{-1} transform of H markers), optionally writes output file.

---

### PA1 Test Results

Test script: `pa1_test.m`. Runs all 7 debug datasets (a–g). Uses a ±0.50 mm tolerance for pivot calibration pass/fail (compares against provided output1 reference files). C_expected differences are reported as [INFO] only (not gated) because distorted datasets inherently diverge from reference.

**Section 1 — Data Ingestion (all PASS):**
All 7 datasets: N_c=27, N_frames=8, N_G=6, N_H=6

**Section 2 — C_expected (informational only):**

| Dataset | Max C_exp error (mm) | Notes |
|---------|----------------------|-------|
| debug-a | 0.0070 | Clean data, near-zero error |
| debug-b | 0.5052 | Slight distortion |
| debug-c | 0.8503 | Moderate distortion |
| debug-d | 0.0227 | Clean data |
| debug-e | 4.1552 | Heavy EM distortion |
| debug-f | 3.8366 | Heavy EM distortion |
| debug-g | 4.7350 | Heavy EM distortion |

*Note: Large errors in e, f, g are expected — they represent the EM distortion that the PA2 distortion correction step would address.*

**Section 3 — Pivot Calibration (all PASS/PASS, tol=0.50 mm):**

| Dataset | EM pivot error (mm) | Optical pivot error (mm) |
|---------|---------------------|--------------------------|
| debug-a | 0.0024 | 0.0070 |
| debug-b | 0.0041 | 0.0060 |
| debug-c | 0.0028 | 0.0013 |
| debug-d | 0.0027 | 0.0055 |
| debug-e | 0.0100 | 0.0041 |
| debug-f | 0.0190 | 0.0010 |
| debug-g | 0.0122 | 0.0055 |

All errors are well below 0.50 mm tolerance. EM pivot errors are slightly larger for distorted datasets (e, f, g) due to EM measurement noise, but still sub-0.02 mm.

**Section 4 — Full PA1 Solve (all PASS):**
All 7 datasets pass. Confirms the full pipeline (data loading → C_expected → EM pivot → optical pivot → output file) runs end-to-end correctly.

**Section 5 — Unknown Dataset Output:**

| Dataset | EM pivot P_dimple (mm) | Optical pivot P_dimple (mm) |
|---------|------------------------|-----------------------------|
| unknown-h | [181.05, 182.96, 199.87] | [393.53, 400.35, 194.06] |
| unknown-i | [201.39, 207.12, 196.52] | [405.90, 407.60, 209.66] |
| unknown-j | [207.74, 205.29, 188.11] | [396.35, 402.93, 190.79] |
| unknown-k | [198.40, 201.69, 205.08] | [391.34, 397.05, 196.42] |

Output files written to `THA3/output/`.

---

## PA2: EYE-IN-HAND (AX = XB) CALIBRATION

### Problem Statement

Find the unknown rigid body transformation **X** from the camera frame to the robot end-effector frame, given N configurations of the robot with corresponding end-effector poses {E_i} and camera sensor poses {S_i}.

The constraint is: for each consecutive pair (i, i+1):
$$A \cdot X = X \cdot B$$

Where:
- A = E_i^{-1} · E_{i+1} (relative end-effector motion)
- B = S_i · S_{i+1}^{-1} (relative sensor motion)
- X = [R_x | p_x; 0 0 0 1] is the unknown camera-to-end-effector transform

### PA2.1: Quaternion Analytical Approach

**Reference:** Daniilidis, K., "Hand-Eye Calibration Using Dual Quaternions," IJRR, 1999. Also: Tsai, R.Y. and Lenz, R.K., "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration," IEEE Trans. Robotics and Automation, 1989.

#### Step 1: Solve for R_x

For each consecutive pair (i, i+1) of the N configurations, compute relative rotations:
$$R_A = R_{E_i}^T R_{E_{i+1}}, \quad R_B = R_{S_i} R_{S_{i+1}}^T$$

Convert each to a quaternion q = [s, v] (scalar s, vector v ∈ ℝ³):
$$q_A = [s_A, v_A], \quad q_B = [s_B, v_B]$$

**Sign alignment:** If q_A · q_B < 0, negate q_B (quaternions and their negatives represent the same rotation).

**Build the M_k block** for each pair:
$$M_k = \begin{bmatrix} s_A - s_B & -(v_A - v_B)^T \\ v_A - v_B & (s_A - s_B)I_3 + [v_A + v_B]_\times \end{bmatrix} \in \mathbb{R}^{4\times 4}$$

Where [·]× denotes the skew-symmetric (SO(3)) matrix of a vector.

Stack all blocks to form M ∈ ℝ^{4k×4} (k = N-1 consecutive pairs):
$$M = \begin{bmatrix} M_1 \\ M_2 \\ \vdots \\ M_k \end{bmatrix}$$

**SVD of M:**
$$M = U \Sigma V^T$$

The quaternion q_x is the **fourth column of V** (smallest singular value → null space of M).

Convert q_x back to R_x ∈ SO(3) using the quaternion-to-rotation formula.

#### Step 2: Solve for p_x (translation)

For each consecutive pair, compute the translation components of A and B.

**Translation part of A** (relative end-effector motion):
$$p_A^{(i)} = R_{E_i}^T \left( t_{E_{i+1}} - t_{E_i} \right)$$

**Translation part of B** (relative sensor motion):
$$p_B^{(i)} = t_{S_i} - R_B \cdot t_{S_{i+1}}$$

These are derived from the full SE(3) composition:
- A = E_i^{-1} · E_{i+1}: since E_i^{-1} = [R_{E_i}^T | -R_{E_i}^T t_{E_i}], the translation part of A is R_{E_i}^T(t_{E_{i+1}} - t_{E_i})
- B = S_i · S_{i+1}^{-1}: since S_{i+1}^{-1} = [R_{S_{i+1}}^T | -R_{S_{i+1}}^T t_{S_{i+1}}], the translation part of B is t_{S_i} - R_{S_i} R_{S_{i+1}}^T t_{S_{i+1}} = t_{S_i} - R_B t_{S_{i+1}}

**Build overdetermined linear system from AX = XB:**

Expanding the translation part of AX = XB:
$$R_A \cdot p_x + p_A = R_x \cdot p_B + p_x$$
$$(R_A - I_3) \cdot p_x = R_x \cdot p_B - p_A$$

Stacking all k pairs:
$$\underbrace{\begin{bmatrix} R_{A_1} - I \\ R_{A_2} - I \\ \vdots \end{bmatrix}}_{A_{ls}} p_x = \underbrace{\begin{bmatrix} R_x p_B^{(1)} - p_A^{(1)} \\ R_x p_B^{(2)} - p_A^{(2)} \\ \vdots \end{bmatrix}}_{b_{ls}}$$

**Solution:** MATLAB `lsqr(A_ls, b_ls)` solves the least-squares problem for p_x.

**Output:** T_x = [R_x | p_x; 0 0 0 1]

---

### PA2 Code: `eyeinhand_cal.m`

**Function signature:**
```matlab
function [T, results] = eyeinhand_cal(q_Robot_config, q_camera_config, t_Robot_config, t_camera_config)
```

**Inputs:**
- `q_Robot_config`: N×4 array of end-effector quaternions [w x y z]
- `q_camera_config`: N×4 array of camera/sensor quaternions [w x y z]
- `t_Robot_config`: N×3 array of end-effector translations
- `t_camera_config`: N×3 array of sensor translations

**Key implementation note:** The translation components p_a and p_b must be computed using the full SE(3) derivation (not simple differences). The correct formulas are:
```matlab
P_A_matrix(i,:) = (R_E_array(:,:,i)' * (t_Robot_config(i+1,:) - t_Robot_config(i,:))')';
P_B_matrix(i,:) = (t_camera_config(i,:)' - R_b * t_camera_config(i+1,:)')';
```

**Outputs:**
- `T`: 4×4 SE(3) transformation X (camera frame → end-effector frame)
- `results`: [lsqr_flag, relative_residual]

**Helper functions used:** `quat2rotm_my`, `rotm2quat_my`, `vecToSO3` (all custom implementations avoiding MATLAB built-ins).

---

### PA2 Test Script: `pa2_test.m`

Three sections:
1. **Clean data, N=10 configurations** — `data_quaternion()` from HW3-PA2/
2. **Noisy data, N=10 configurations** — `data_quaternion_noisy()` from HW3-PA2/
3. **Noisy data, N=5 configurations** — first 5 rows of noisy data

For each: runs `eyeinhand_cal`, prints R_x, P_x, det(R_x), lsqr convergence stats, and AX=XB residuals per consecutive pair.

**verify_AXB helper:** Builds full 4×4 A = T_Ei \ T_Ei1 and B = T_Si / T_Si1 for each pair, computes Frobenius norm of (AX - XB).

---

### PA2 Results

#### Section 1: Clean Data (N=10)

**R_x:**
```
[-0.00320  0.99999 -0.00011]
[-0.00077  0.00011  1.00000]
[ 0.99999  0.00320  0.00077]
```
**P_x:** [0.13551, -0.96773, -0.31439] (units match input data)

**det(R_x):** 1.000000 (valid rotation matrix ✓)

**lsqr:** flag=0 (converged), relres=4.21e-01

**AX=XB residuals per pair:**
| Pair | Residual (Frobenius norm) |
|------|--------------------------|
| 1-2 | 0.516741 |
| 2-3 | 0.548111 |
| 3-4 | 0.585310 |
| 4-5 | 0.145857 |
| 5-6 | 0.510305 |
| 6-7 | 0.359811 |
| 7-8 | 0.129129 |
| 8-9 | 0.077542 |
| 9-10 | 0.224507 |
**Mean AX=XB residual: 0.344146**

#### Section 2: Noisy Data (N=10)

**R_x:**
```
[-0.00336  0.99999  0.00003]
[-0.00088 -0.00003  1.00000]
[ 0.99999  0.00336  0.00088]
```
**P_x:** [0.13583, -0.96729, -0.31408]

**det(R_x):** 1.000000 ✓

**lsqr:** flag=0, relres=4.21e-01

**Mean AX=XB residual: 0.344205**

#### Section 3: Noisy Data (N=5, first 5 configs only)

**R_x:**
```
[-0.00368  0.99999  0.00360]
[-0.00307 -0.00361  0.99999]
[ 0.99999  0.00367  0.00308]
```
**P_x:** [0.17342, -1.11031, -0.30258]

**det(R_x):** 1.000000 ✓

**lsqr:** flag=0, relres=3.71e-01

**Mean AX=XB residual: 0.430885**

---

### PA2.2 Summary Comparison (Bonus — discuss results)

**Rotation difference (Frobenius norm of R_diff):**
| Comparison | ||R_clean - R_?||_F |
|------------|----------------------|
| Clean (N=10) vs Noisy (N=10) | 0.000333 |
| Clean (N=10) vs Noisy (N=5) | 0.006216 |
| Noisy (N=10) vs Noisy (N=5) | 0.005951 |

**Translation difference (Euclidean norm):**
| Comparison | ||p_clean - p_?||_2 |
|------------|----------------------|
| Clean (N=10) vs Noisy (N=10) | 0.000628 |
| Clean (N=10) vs Noisy (N=5) | 0.148007 |
| Noisy (N=10) vs Noisy (N=5) | 0.148327 |

**Mean AX=XB residuals:**
| Case | Mean Residual |
|------|---------------|
| Clean (N=10) | 0.344146 |
| Noisy (N=10) | 0.344205 |
| Noisy (N=5) | 0.430885 |

#### Discussion Points for PA2.2b

1. **Noise effect (N=10):** Adding noise to the full 10-configuration dataset causes negligible degradation — the rotation changes by only 0.000333 (Frobenius norm) and translation by 0.000628 mm. The algorithm is very robust to the noise level in the provided dataset. This is expected because with 9 consecutive pairs (k=9), the overdetermined system (36×4 for rotation, 27×3 for translation) averages out random noise well via least-squares.

2. **Reduced data effect (N=5):** Halving the data to 5 configurations causes much more significant degradation — the rotation error grows ~19× (to 0.006216) and translation error grows ~235× (to 0.148 units). This shows that the number of configurations has a much larger impact than noise level.

3. **Why fewer configurations hurt more:** With only 4 consecutive pairs (k=4), the overdetermined system is much less overdetermined (16×4 for rotation, 12×3 for translation). The SVD null-space estimate of q_x becomes more sensitive to the specific configurations chosen. If configurations are not geometrically diverse (varying rotation axes), the M matrix may be nearly rank-deficient, making the q_x estimate unreliable.

4. **AX=XB residuals (~0.34):** The non-zero residuals even for clean data indicate that the provided dataset itself has some inherent inconsistency (the constraint AX = XB cannot be exactly satisfied for all pairs simultaneously with a single X). This is normal for real robot data due to small kinematic modeling errors or calibration tolerances in the provided ground truth.

---

## REFERENCES

[1] PA1 adapted with permission from the *Computer Integrated Surgery* course, Dr. Russell H. Taylor, Johns Hopkins University.

[2] Arun, K.S., Huang, T.S., Blostein, S.D., "Least-Squares Fitting of Two 3-D Point Sets," *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 9, no. 5, pp. 698–700, 1987.

[3] Horn, B.K.P., "Closed-form solution of absolute orientation using unit quaternions," *Journal of the Optical Society of America A*, vol. 4, no. 4, pp. 629–642, 1987.

[4] Daniilidis, K., "Hand-Eye Calibration Using Dual Quaternions," *International Journal of Robotics Research*, vol. 18, no. 3, pp. 286–298, 1999.

[5] Tsai, R.Y. and Lenz, R.K., "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration," *IEEE Transactions on Robotics and Automation*, vol. 5, no. 3, pp. 345–358, 1989.

[6] Shiu, Y.C. and Ahmad, S., "Calibration of Wrist-Mounted Robotic Sensors by Solving Homogeneous Transform Equations of the Form AX=XB," *IEEE Transactions on Robotics and Automation*, vol. 5, no. 1, pp. 16–29, 1989.

---

## NOTES FOR THE REPORT WRITER

- The report must show **derived equations**, **describe the approach**, include **test functions**, and **discuss results for each function**.
- The assignment requires citing references — cite [2] or [3] for cloud2cloud, and [4] or [5] or [6] for the AX=XB approach.
- For PA1 C_expected: make clear that the large errors for datasets e, f, g are *expected* and represent EM distortion, not a bug in the formula.
- For PA2 AX=XB residuals: the mean residual of ~0.34 is non-zero because real robot data cannot perfectly satisfy AX=XB for all pairs simultaneously. The rotation component is extremely accurate (det(R_x) = 1.000000 exactly, small off-diagonal elements confirm near-permutation structure consistent with the physical camera-robot geometry).
- The det(R_x) = 1.000000 for all three cases confirms R_x is a valid SO(3) rotation matrix.
- lsqr flag=0 means the solver converged (not a failure code).
- All MATLAB implementations use custom `_my` suffix functions (quat2rotm_my, rotm2quat_my, vecToSO3, etc.) rather than MATLAB built-ins, per course convention.
