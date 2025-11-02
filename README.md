# Intrinsic rigid-body PID Control and Extended Kalman Filter

This is a compilation of a set of interactive notes and supplementary material for intrinsic PID control and DEKF for rigid body motion.

The following repo contains a Lie-Group oriented treatment of classical mechanics and a set of python tools for simulating roigid body motion and the DEKF.

https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view

**D. H. S. Maithripala, PhD.**  
smaithri@eng.pdn.ac.lk  
https://eng.pdn.ac.lk/ME/People/FacultyProfiles.php?id=6  
https://orcid.org/0000-0002-8200-2696


---

## 🌀 Geometric PID Controller on (\mathbb{R}^3 \times SO(3))

This repository implements a **geometric PID controller** for fully actuated rigid body systems evolving on the nonlinear configuration space (\mathbb{R}^3 \times SO(3)). The controller operates in the **spatial momentum space**, enabling coordinate-free tracking of position and orientation trajectories with provable convergence properties.

### ✒️ Controller Description

The control law is derived by lifting classical PID control to the Lie group (\mathbb{R}^3 \times SO(3)), ensuring compatibility with the group structure of rigid body motion. The key components include:

* **Right-invariant configuration error**: Defined as ((o_e, R_e) = (o_r - o, R_r R^T)), capturing position and orientation discrepancies between the current and reference trajectories.

* **Momentum-based error dynamics**: The controller is expressed in terms of linear and angular momentum errors ((p_e, \pi_e)), which evolve linearly under the influence of control inputs.

* **Geometric integral terms**: Integral errors are defined on the configuration space itself, preserving geometric consistency without resorting to local coordinates or quaternions.

* **Quadratic-like error function**: A scalar Lyapunov candidate function is constructed using a trace-based term on (SO(3)), whose gradient defines the proportional action in a group-consistent way.

### 🚀 Significance

This controller achieves **global coordinate-free tracking** of desired rigid body trajectories, avoiding singularities and ambiguities associated with parameterizations like Euler angles or quaternions.

By working directly in the **momentum space** and respecting the geometric structure of the configuration space, the design naturally accommodates:

* **Feedforward compensation** of desired momentum trajectories
* **Proportional-derivative-integral feedback** in a globally consistent way
* **Stability and convergence** guarantees under standard Lyapunov analysis on manifolds

### 🌍 Almost-Global Convergence

Due to the topological properties of the rotation group (SO(3)), **global asymptotic stabilization** is impossible using continuous state-feedback. However, this controller achieves **almost-global convergence**, meaning:

A key feature of this controller is that it leverages the **linearity of the rigid body momentum equations**. By formulating the dynamics in terms of spatial linear and angular momentum, the control design reduces to applying **standard PID structure** on a linear system — despite the nonlinear configuration space. This dramatically simplifies the controller implementation while preserving geometric correctness.


* The desired configuration is **asymptotically stable** from almost all initial conditions.
* The only exceptions are a measure-zero set of initial attitudes corresponding to 180° rotations around principal axes — these are **unstable saddle points** of the error function.

This is the best possible result achievable with smooth feedback on (SO(3)), making the controller **theoretically optimal** under the constraints of continuous control.

### References

* D.H.S. Maithripala, Jordan M. Berg,
An intrinsic PID controller for mechanical systems on Lie groups, Automatica, Volume 54, 2015, Pages 189-200, ISSN 0005-1098,
[PDF](https://www.sciencedirect.com/science/article/pii/S0005109815000060)

* Rama Seshan Chandrasekaran, Ravi N. Banavar, Arun D. Mahindrakar, D.H.S. Maithripala,
Geometric PID controller for stabilization of nonholonomic mechanical systems on Lie groups, Automatica, Volume 165, 2024, 111658, ISSN 0005-1098, [PDF](https://www.sciencedirect.com/science/article/pii/S0005109824001511)

* D. H. S. Maithripala, J. M. Berg and W. P. Dayawansa, "Almost-global tracking of simple mechanical systems on a general class of Lie Groups," in IEEE Transactions on Automatic Control, vol. 51, no. 2, pp. 216-225, Feb. 2006, doi: 10.1109/TAC.2005.862219. [PDF](https://ieeexplore.ieee.org/abstract/document/1593897)

---

## 🌀 Discrete Invariant Extended Kalman Filter (DEKF) on Lie Groups

This repository contains the **full derivation and simulation of a discrete-time invariant Extended Kalman Filter (EKF)** on Lie groups, implemented and demonstrated in Python/Colab.
The project develops the theory step by step — from stochastic rigid-body kinematics to the final discrete filter equations — and validates the results through Monte Carlo simulations on (SO(3)).

---

### 📘 Overview

The notebook presents a **geometrically consistent formulation** of the EKF directly on a Lie group (G), following the framework of [Barrau & Bonnabel (2017–2020)](https://arxiv.org/abs/1410.1465).
Unlike classical EKFs in Euclidean space, this approach respects the **group structure** of rotations and rigid-body motions, yielding **invariant error dynamics** and improved consistency.

The derivation starts from:
[
g_k = g_{k-1}\exp(\Delta T,\zeta_{k-1}), \quad
y_k = \phi_{g_k^{-1}}(\gamma) + z_k,
]
and constructs a discrete left-invariant observer:
[
\tilde g_k^- = \tilde g_{k-1}\exp(\Delta T\zeta_{k-1} + \sqrt{\Delta T}w_{k-1}), \qquad
\tilde g_k = \exp(L(y_k, \tilde y_k^-)),\tilde g_k^-,
]
leading to a **discrete invariant error recursion** on the Lie algebra:
[
\eta_k = \eta_{k-1}

* \Phi(\eta_{k-1})!\left(
  \operatorname{Ad}*{\tilde g*{k-1}}\Phi(-\Delta T\zeta_{k-1})\sqrt{\Delta T},w_{k-1}
  \right)

- \Phi(\eta_{k-1}),L(y_k,\tilde y_k^-)

* \mathcal{O}(|L|^2, |w|^2).
  ]

Linearization yields the discrete invariant EKF update equations:
[
\eta_k = (A_{k-1} - K_kH_k)\eta_{k-1} + (I - K_kH_k)G_{k-1}w_{k-1} - K_kz_k,
]
with:
[
A_{k-1} = I, \quad
G_{k-1} = \sqrt{\Delta T},\operatorname{Ad}*{\tilde g*{k-1}}\Phi(-\Delta T\zeta_{k-1}).
]

These equations mirror the **standard linear Kalman filter** form but are expressed intrinsically on the Lie group.

---

### 🧠 Features

* **Full symbolic derivation** of the discrete invariant error dynamics.
* **Implementation of the DEKF** for attitude estimation on (SO(3)).
* **Consistent handling of gyro and direction sensor noise**, with correct scaling between continuous- and discrete-time models.
* **Automatic tuning sweep** (`auto_tune_EKF`) to explore the effect of process and measurement noise ratios.
* **Diagnostic plots** for trace misalignment, covariance evolution, and steady-state uncertainty.
* **Covariance inflation / filter rejuvenation** to prevent overconfidence and maintain long-term stability.

---

### ⚙️ Implementation Details

#### **Sensor model**

The simulated IMU provides:
[
\Omega_\text{meas} = \Omega_\text{true} + \mathcal{N}(0,\sigma_\omega^2I), \qquad
A_i^\text{meas} = R^\top e_i + \mathcal{N}(0,\sigma_\text{dir}^2I),
]
with ( \sigma_\omega ) the per-sample gyro noise and ( \sigma_\text{dir} ) the unit-vector noise.

#### **Filter propagation**

[
\tilde R_k^- = \tilde R_{k-1}\exp(\Delta T,\Omega_{k-1}),
\quad
P_k^- = A P_{k-1}A^\top + G\Sigma_qG^\top,
]
where (A = I) and (G = \Delta T,R,\Phi(-\Delta T\Omega)).

#### **Correction**

[
K_k = P_k^-H_k^\top(H_kP_k^-H_k^\top+\Sigma_m)^{-1}, \qquad
\tilde R_k = \exp(L(y_k, \tilde y_k^-)),\tilde R_k^-.
]

#### **Covariance inflation (rejuvenation)**

To prevent covariance collapse and keep the EKF responsive:

```python
# Symmetrize and inflate
P = 0.5 * (P + P.T)
P = (1.02) * P + 1e-9 * np.eye(3)
```

This small inflation step is formally justified and ensures long-term numerical stability without restarting the filter.

---

### 📊 Simulation Results

The notebook includes diagnostic plots showing:

* **Trace error** (3 - \mathrm{tr}(R_\text{true}^\top R_\text{est})) — a scalar measure of attitude misalignment.
* **Covariance magnitude** ((\lambda_{\max}(P))) and equivalent 1σ angular uncertainty.
* **Parameter sweeps** for (\Sigma_q) and (\Sigma_m), illustrating the trade-off between smoothness and responsiveness.

A well-tuned configuration (e.g. `Σ_q_factor=5`, `Σ_m_factor=0.1`) yields:

* Steady-state 1σ ≈ 1°,
* Trace error < 0.05 → ≈ 10–12° misalignment,
* Stable and consistent performance under stochastic excitation.

---

### 🧩 References

* A. Barrau and S. Bonnabel, *The Invariant Extended Kalman Filter as a Stable Observer*, IEEE TAC, 2017.
* A. Barrau and S. Bonnabel, *Intrinsic Filtering on Lie Groups with Applications to Attitude Estimation*, SIAM Review, 2020.


---


## 🧭 Summary

This repository provides a **complete derivation-to-simulation pipeline** for the Discrete Invariant EKF on Lie groups, including:

* rigorous Lie-algebra-based derivations,
* correct stochastic discretization,
* practical tuning and stability enhancements (covariance inflation).

It bridges theoretical filtering on manifolds with implementable, numerically robust code — ready for use in **robotics, UAV attitude estimation, and navigation research**.

---

Would you like me to make it a bit shorter and more “GitHub landing page–style” (with badges and emojis, e.g. 🚀📘🧩), or keep it as this comprehensive academic-style README?


---

## 🔖 License

MIT © DHS Maithripala
