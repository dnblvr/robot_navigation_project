User: what's the procedure on how to set up an invariant extended kalman filter? what's the steps involed and relate it to how a standard EKF works

GitHub Copilot: I'll research your workspace for any existing work on Invariant EKF and provide a comprehensive explanation of the setup procedure and comparison with standard EKF.  

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/Invariant_EKF)

Searched codebase for "invariant extended kalman filter InEKF setup implementation", 2 results

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/Invariant_EKF/Invariant_EKF.md)

Based on your Lie Theory notes, here's how to set up an Invariant Extended Kalman Filter (InEKF) and how it relates to a standard EKF:

## Invariant EKF Setup Procedure

### **1. Choose Your State Representation on a Lie Group**
- **Standard EKF**: State lives in Euclidean space $\mathbb{R}^n$
- **InEKF**: State lives on a matrix Lie group (typically $SE(3)$, $SE_2(3)$, or other extended pose groups)
  - Example: $\mathcal{X} \in SE(3)$ for pose estimation
  - State includes rotation $\bold{R}$, position $\bold{t}$, and potentially velocity

### **2. Define State Error on the Tangent Space**
- **Standard EKF**: Error is additive: $\tilde{x} = x - \hat{x}$
- **InEKF**: Error is multiplicative on the group:
  - Right-invariant: $\tilde{\mathcal{X}} = \hat{\mathcal{X}}^{-1} \mathcal{X}$
  - Left-invariant: $\tilde{\mathcal{X}} = \mathcal{X} \hat{\mathcal{X}}^{-1}$
  - Map error to tangent space: $\xi = \text{Log}(\tilde{\mathcal{X}}) \in \mathbb{R}^m$

### **3. System Dynamics (Prediction Step)**
- **Standard EKF**: $x_{k+1} = f(x_k, u_k) + w_k$
- **InEKF**: Dynamics evolve on the manifold:
  ```
  State propagation: Ẋ = X · v^∧  (right-invariant form)
  Discrete: X_k = X_{k-1} ⊕ (v_k Δt)
            X_k = X_{k-1} · Exp(v_k Δt)
  ```
  - Covariance propagates using the adjoint matrix: $\bold{P}_{k+1} = \bold{Ad}_{\text{Exp}(v)} \bold{P}_k \bold{Ad}_{\text{Exp}(v)}^T + \bold{Q}$

### **4. Compute Jacobians on the Manifold**
- **Standard EKF**: Standard partial derivatives $\frac{\partial f}{\partial x}$
- **InEKF**: Right Jacobians (from [Lie_Theory.md](f:/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/Lie_Theory/Lie_Theory.md#1372-right-jacobians-on-lie-groups)):
  ```
  J^R = lim_{τ→0} [(f(X ⊕ τ) ⊖ f(X)) / τ]
  ```
  - Use elementary blocks: composition, inversion, exponential, group actions
  - Key tool: Adjoint matrix $\bold{Ad}_{\mathcal{X}}$ for transforming between tangent spaces

### **5. Measurement Update**
- **Standard EKF**: 
  ```
  Innovation: y = z - h(x̂)
  Kalman gain: K = PH^T(HPH^T + R)^{-1}
  Update: x̂+ = x̂ + Ky
  ```
- **InEKF**:
  ```
  Innovation in tangent space: ξ = h^{-1}(z) ⊖ h^{-1}(ĥ)
  Kalman gain: K = PH^T(HPH^T + R)^{-1}
  Update on manifold: X̂+ = X̂ ⊕ Kξ
                      X̂+ = X̂ · Exp(Kξ)
  ```
  - Jacobians computed using group action formulas

### **6. Covariance Update**
- **Standard EKF**: $P^+ = (I - KH)P$
- **InEKF**: Same form but with manifold Jacobians:
  ```
  P^+ = (I - KH)P(I - KH)^T + KRK^T
  ```
  - Covariance stays in $\mathbb{R}^{m \times m}$ (tangent space)

## Key Differences Summary

| Aspect | Standard EKF | Invariant EKF |
|--------|-------------|---------------|
| **State space** | $\mathbb{R}^n$ | Lie group $\mathcal{M}$ |
| **Error definition** | Additive: $\tilde{x} = x - \hat{x}$ | Multiplicative: $\tilde{\mathcal{X}} = \hat{\mathcal{X}}^{-1} \mathcal{X}$ |
| **Updates** | Addition: $x^+ = x + \Delta x$ | Composition: $\mathcal{X}^+ = \mathcal{X} \oplus \tau$ |
| **Jacobians** | Standard partials | Right/Left Jacobians + Adjoint |
| **Covariance** | Lives in $\mathbb{R}^{n \times n}$ | Lives in $\mathbb{R}^{m \times m}$ (tangent space) |
| **Key advantage** | Simple | **Invariant to transformations** (consistent errors regardless of reference frame) |

## Why InEKF?

The **invariance property** means estimation errors remain consistent regardless of the coordinate frame - a rotation error of 5° stays 5° whether you're facing north or east. Standard EKF errors can grow inconsistently depending on orientation, leading to filter divergence.

Your [Lie_Theory.md](f:/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/Lie_Theory/Lie_Theory.md) notes cover all the mathematical tools needed: exponential maps, adjoints, and manifold Jacobians.

User: if I come up with a state variable, an appropriate motion model, measurement model, and select the appropriate lie group onto which the state variable lives, then would the steps you outlined would be largely the same?

GitHub Copilot: Yes, exactly! Once you've made those design choices, the steps I outlined are **the systematic procedure** you follow regardless of your specific application.

## The Design Choices (Problem-Specific):
1. **State variable** - What you're estimating (pose, velocity, IMU biases, etc.)
2. **Motion model** - How state evolves: $\dot{\mathcal{X}} = f(\mathcal{X}, u)$
3. **Measurement model** - How sensors relate to state: $z = h(\mathcal{X})$
4. **Lie group** - Which manifold your state lives on ($SO(3)$, $SE(3)$, $SE_2(3)$, etc.)

## The Implementation Steps (Always the Same):
1. ✓ Express error multiplicatively in the group you chose
2. ✓ Propagate state using $\oplus$ operator (composition on manifold)
3. ✓ Compute right Jacobians using your chosen group's formulas
4. ✓ Propagate covariance using adjoint matrix from your group
5. ✓ Measurement update in tangent space with group-based innovation
6. ✓ Update state via $\mathcal{X}^+ = \mathcal{X} \oplus \tau$ 

The **only difference** between applications is:
- Which Jacobian formulas you plug in (from your group's appendix - e.g., [SE(2) Jacobians](f:/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/Lie_Theory/Lie_Theory.md#16-appendix-c-the-2d-rigid-motion-group-se2))
- The specific $\bold{Ad}_{\mathcal{X}}$ matrix structure
- Whether you use right-invariant vs left-invariant (typically right for robotics)

**The hard part** is choosing the right group and modeling. The **easy part** is that once chosen, InEKF implementation is formulaic - just plug your group's exponential, adjoint, and Jacobians into the standard structure.

Your Lie Theory notes already have the formulas for common groups (SO(2), SO(3), SE(2), SE(3)) in the appendices!

User: I am struggling thinking about variables. there are some state variables that I would consider adding need a more complicated manifold like theta, thus requiring SE2 (as per my notes), i think it would also benefit from having linear velocity and angular velocity, but I don't think SE2 can support those. furthermore, there's an issue that if there is no lie group that can describe this motion, I may have to go with a regular EKF implementation. how should I proceed?

GitHub Copilot: You're bumping into a **very common problem** that has an elegant solution: **extended Lie groups**!

## The Solution: Extended Lie Groups

SE(2) only represents **pose** (position + orientation). To include **velocities**, you use an **extended group** like $SE_2(2)$ (pronounced "SE-two-two") which is SE(2) augmented with velocity states.

### For Your 2D Case with Velocities:

The state would be: $\mathcal{X} = (R, \mathbf{t}, \mathbf{v}, \omega)$ where:
- $R \in SO(2)$: orientation (angle θ)
- $\mathbf{t} \in \mathbb{R}^2$: position
- $\mathbf{v} \in \mathbb{R}^2$: linear velocity (body frame)
- $\omega \in \mathbb{R}$: angular velocity

**Matrix representation** (5×5):
$$
\mathcal{X} = \begin{pmatrix}
R & \mathbf{t} & \mathbf{v} \\
0 & 1 & \omega \\
0 & 0 & 1
\end{pmatrix}
$$

This is exactly analogous to $SE_2(3)$ (used for 3D IMU-based navigation), but for 2D.

## When to Use InEKF vs Standard EKF

| Use InEKF when: | Use Standard EKF when: |
|-----------------|------------------------|
| States form natural groups (poses, rotations) | States are purely Euclidean with no geometric structure |
| You want **frame-invariant** estimation | Sensor biases only (scalars in $\mathbb{R}^n$) |
| IMU integration, SLAM, odometry | Simple beacon tracking in fixed frame |
| **Extended groups exist for your problem** | State mixing is truly incompatible with groups |

## Your Specific Case

For a 2D mobile robot with IMU/odometry:

**✓ InEKF is perfect:**
```
State: X ∈ SE_2(2) = {R, t, v, ω}
Motion model: IMU measurements (accel + gyro)
Measurements: GPS positions, compass heading, wheel encoders
```

**The key insight:** You don't need a pre-existing group in your notes - you construct the extended group as a **semi-direct product**. The group operations ($\circ$, $\exp$, $\log$, $\bold{Ad}$) follow systematic rules.

## What You Need to Derive 

Searched codebase for "SE_2 extended group velocity augmented semi-direct product", 2 results

For your **2D robot with velocities**, you'd construct $SE_2(2)$ yourself following these rules:

1. **Composition rule**: combines pose and velocity updates
2. **Exponential map**: closed-form or numerical (depends on coupling)
3. **Adjoint matrix**: transforms velocities between reference frames
4. **Jacobians**: systematic from the group structure

## Practical Next Steps

**Option 1: Use established extended group** (recommended)
- Research $SE_2(3)$ formulations and adapt to 2D
- Papers: "Contact-Aided Invariant Extended Kalman Filter" by Hartley et al.
- Many robotics InEKF implementations exist with velocity states

**Option 2: Simplify your state**
- If velocities are measured directly (encoders), treat them as separate Euclidean states alongside the pose group
- Still benefit from InEKF for the $(R, \mathbf{t})$ part
- Mix: InEKF for pose + standard EKF for velocities (common in practice)

**Option 3: Custom extended group**
- Derive $SE_2(2)$ operations yourself
- More work, but gives you exact control

## Recommendation

Start with **Option 2**: Keep pose in SE(2) with InEKF, handle velocities separately. This hybrid approach is *very common* and gives you most benefits without deriving a full extended group.

```
State decomposition:
- Pose part: X ∈ SE(2) with InEKF
- Velocity part: [vx, vy, ω] ∈ ℝ³ with standard EKF update
```

You're not "falling back" to standard EKF - you're using the right tool for each part of your state. InEKF shines for geometric quantities (rotations, poses), while standard EKF is fine for Euclidean velocities that don't suffer from frame-dependence issues.

Want me to help sketch what this hybrid approach would look like for your specific application?

User: I think on second-thought, I think I should simplify things. since I am using this for a simple 2d slam algorithm and I want to improve upon the two-wheeled odometry measurements and an IMU, I would rather stick with $SE(2)$. I assume if I wanted to estimate and propagate the velocity state i.e. incorporate it in the motion model and the current sensor model through the odometry/body accel measurements, all it would really do for me is to have a finer estimate on speed for a more fine position estimates. 

I see another alternative that if I did not implement 2d velocity that I would still have some leeway on the estimate the appropriate sensor model of this alternative's velocity would still inherently have a noise component added to it, right?

which perspective is more reasonable?

GitHub Copilot: **Perspective 2 (velocity NOT in state) is more reasonable for your application.** Here's why:

## For 2D SLAM with Wheel Odometry + IMU

**State: Just SE(2) pose** - $(R, \mathbf{t})$ or $(x, y, \theta)$

**Your sensors already measure velocity directly:**
- Wheel encoders → linear velocity $v$
- IMU gyro → angular velocity $\omega$
- IMU accel → body acceleration (can validate/augment odometry)

## How Noise is Handled Without Velocity in State

You're **exactly right** about the noise component! When velocity isn't in the state:

```
Motion Model:
  Ẋ = X · u^∧    where u = [v, ω] from measurements

Discrete propagation:
  X_{k+1} = X_k ⊕ (u_k Δt + w_k)
```

The **measurement noise in your velocity inputs** becomes **process noise** in pose propagation:

$$\mathbf{Q} = \mathbb{E}[w w^T] = \text{uncertainty from noisy velocity measurements}$$

This naturally propagates velocity uncertainty into pose uncertainty without explicitly estimating velocity.

## When to Include Velocity in State

You'd want velocity in state only if:
- ❌ Velocity measurements are very sparse/intermittent
- ❌ You need smoothed velocity estimates for control
- ❌ Velocity has complex dynamics (e.g., accelerating vehicles)

For a **two-wheeled robot with continuous encoder readings**, none of these apply.

## Standard 2D SLAM Approach

```
State: X ∈ SE(2) = [R, t]

Prediction (with odometry/IMU inputs):
  - Read: v, ω from encoders/gyro
  - Propagate: X̂_k = X̂_{k-1} · Exp([v·Δt, 0, ω·Δt])
  - Covariance: P_k = Ad·P_{k-1}·Ad^T + Q
    where Q accounts for velocity measurement noise

Update (with landmarks, GPS, etc.):
  - Standard InEKF measurement update
```

## Your Insight is Correct

> "velocity would still inherently have a noise component added to it"

**Yes!** The noise in your velocity measurements directly affects:
1. **Process noise Q**: represents uncertainty in how you propagate pose
2. **Pose uncertainty P**: grows during prediction based on noisy velocity inputs

You get the benefit of accounting for velocity noise **without the complexity** of estimating it.

## Bottom Line

**Stick with SE(2)**, use wheel odometry + IMU gyro as **inputs** (not measurements of states), and let their noise naturally flow through as process noise. This is simpler, standard practice, and perfectly adequate for 2D SLAM with your sensors.

User: I have a few more questions. in the same way that the madgwick's second quaternion algorithm, the extended complementary filter (ECF) uses both gyro and magnetometer measurements to estimate orientation while playing to both strengths.

in my interpretation of the research, the magnetometer, while noisy and susceptible to soft and hard biases, had filters to keep the arrows in a mostly stable direction. for all intents and purposes, this had a relatively stable reference point to look up to. 

the gyro, somewhat susceptible to gyro drift in consumer sensors like the icm20948, can give local changes quite accurately.

it was the use of both in the madgwick with hard filters (magnitude limits on the magnetometer measurement) which gave it some appeal to me. so if the magnetometer is wrong, then the current estimate is also at least partially wrong. in coming up with an inEKF design, how can we incorporate this magnetometer measurement in the same way that ECF does?

I might have a suggestion. can gyro orientation, as it is in an SE(2) paradigm, be part of the motion model and the magnetic compass measurement be part of the sensor model that's compared against?

if there are other research suggesting other ideas, I'd be happy to consider it.

GitHub Copilot: 

  

Research Madgwick/ECF and magnetometer usage

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/All_things_Kalman_Filter)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/EKF-SLAM)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/Learnbox)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/include)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/src)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/EKF-SLAM/python)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/Learnbox/python)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/src)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/inc)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/All_things_Kalman_Filter/platformio)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/All_things_Kalman_Filter/processing)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/include/MadgwickECF.h)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/include/MadgwickGDA.h#1-1), lines 1 to 200

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/include/data_structure.h)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/src/MadgwickECF.cpp)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/src/MadgwickGDA.cpp)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/src/main.cpp)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/ekf_example.c)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/src/ekf.c)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/inc/ekf.h)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/Learnbox/python/test.py)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/MadgwickFilter/include/MadgwickGDA.h#200-200), lines 200 to 410

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/src/floating_point_impl.h#1-1), lines 1 to 300

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/inc/matrices.h)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/src/matrices.c)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/pose_optimizations/main.c)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/EKF-SLAM)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/src/floating_point_impl.h#300-300), lines 300 to 510

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/EKF-SLAM/EKF-SLAM.md)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/helper_3dmath)

Read [](file:///f%3A/Users/gianf/Document%20Repository/Extra%20Notes/Robotics_Investigations/EKF-SLAM/resources)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/ICM_20948/include)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/helper_3dmath/include)

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/helper_3dmath/src)

Searched for regex `dataframe_t|mag|magnetometer|heading|yaw` (`**/helper_3dmath.h`), 20 results

Searched for regex `dataframe_t|mag|getProduct|getRotated|getMagnitude|getNormalized` (`**/ICM_20948.h`), 20 results

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/helper_3dmath/include/helper_3dmath.h#420-420), lines 420 to 500

Searched for regex `Get_Euler|Get_Gravity|heading|yaw|getYaw|Get_Yaw_Pitch_Roll` (`**/helper_3dmath.*`), 6 results

Read [](file:///c%3A/Users/gianf/Documents/Autonomous_Navigation_Project/autonomous_navigation_repo/robot_navigation_project/teensy_coprocessor/coprocessor_1/lib/helper_3dmath/include/helper_3dmath.h#570-570), lines 570 to 625

Your suggestion is **exactly the standard and correct approach** — and it mirrors what your ECF already does, just reframed in the InEKF language.

## Your Intuition Mapped to InEKF

| ECF (current) | InEKF equivalent |
|---|---|
| Gyro integrates quaternion (prediction) | Gyro drives **motion model** on SE(2) |
| Magnetometer cross-product corrects yaw | Magnetometer heading drives **measurement model** |
| `K_norm` gain blends correction | **Kalman gain** $K$ blends correction |
| Mag gating `2000 < norm < 6500` | **Innovation gating** (chi-squared test on residual) |

The structure is identical in philosophy. The InEKF just replaces heuristic gains with **statistically optimal** ones derived from covariance.

## Concrete SE(2) InEKF Design

### Motion Model (Prediction)
**Inputs:** wheel odometry $(v)$ + IMU gyro $(\omega)$

$$\mathcal{X}_{k} = \mathcal{X}_{k-1} \cdot \text{Exp}\begin{pmatrix} v \Delta t \\ 0 \\ \omega \Delta t \end{pmatrix}$$

Process noise $\mathbf{Q}$ captures encoder noise + gyro drift.

### Measurement Model (Update) — Magnetometer Heading
**Observation:** compass heading $\psi_{\text{mag}}$

$$h(\mathcal{X}) = \theta \quad \text{(extract heading from } \mathcal{X} \text{)}$$

$$z = \psi_{\text{mag}}, \quad \text{innovation: } y = z - h(\hat{\mathcal{X}})$$

The measurement Jacobian $\mathbf{H}$ is straightforward — it just picks out the $\theta$ component from the tangent space vector $\xi = [\delta x, \delta y, \delta \theta]^T$:

$$\mathbf{H} = \begin{pmatrix} 0 & 0 & 1 \end{pmatrix}$$

Measurement noise $\mathbf{R}$ captures magnetometer noise + soft/hard iron distortion.

### Mag Gating (Your Hard Filters, Formalized)

Your ECF already does this in MadgwickECF.h with the `2000 < mag_norm < 6500` check. In InEKF, the equivalent is:

1. **Pre-gate:** Same magnitude check — reject clearly corrupt readings before they enter the filter
2. **Innovation gate (chi-squared test):**
   $$y^T (\mathbf{H P H}^T + \mathbf{R})^{-1} y < \chi^2_{\alpha}$$
   If the magnetometer disagrees *too much* with the current estimate (given the uncertainty), **reject it automatically.** This is the statistical version of your hard filter.

## Why This Works Well

- **Gyro in motion model**: gives you the same accurate local rotation tracking you already have
- **Magnetometer in measurement model**: provides the same long-term drift correction, but now with **quantified uncertainty** rather than a tuned gain
- **When mag is wrong**: the innovation gate rejects it just like your ECF magnitude check, but also catches *directionally* wrong readings (e.g., passing near a motor)
- **When mag is good**: the Kalman gain automatically weights it appropriately based on how uncertain your current heading estimate is

## Regarding Other Approaches in Literature

Your design aligns with standard practice. Some alternatives:

1. **Magnetometer as a "pseudo-measurement" of the rotation matrix** — Instead of extracting scalar heading, compare the full 2D magnetic vector against an expected reference direction. More information, but more complex $\mathbf{H}$.

2. **Online mag bias estimation** — Augment state to include hard-iron bias: $\mathcal{X} \in SE(2)$, $\mathbf{b}_{\text{mag}} \in \mathbb{R}^2$. The bias part stays Euclidean (hybrid approach from our earlier discussion). This is what papers like Sola's recommend for robustness.

3. **Dual-rate fusion** — Run gyro prediction at full IMU rate (100 Hz from your ICM-20948 config), mag update at lower rate. Your ECF already implicitly does this since mag is only applied when `DRDY` is set.

**For your 2D SLAM case, your proposed approach (gyro → motion, mag heading → measurement) is the cleanest and most well-supported design.** Start there. Add mag bias estimation later only if you see systematic heading drift.