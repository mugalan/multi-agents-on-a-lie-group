This repository summarizes the geometric control framework for a network of fullyactuated simple mechanical systems evolving on a Lie group $ G $.  
The framework extends the **AGLES-PID** and **virtual-structure synchronization** methodology to multi-agent systems.

---

## 1. System Setup

### Agents and Virtual Systems
We consider $N$ agents
$$\mathscr{A} = \{A_1, A_2, \dots, A_N\},$$

each modeled as a fully actuated simple mechanical system on a Lie group $G$ with Lie algebra $\mathfrak{g}$ and dual $\mathfrak{g}^*$.

Each agent $A_i$ and its virtual counterpart $V_i$ evolve as:
$$
\begin{align}
(g_i, \pi_i) \in G \times \mathfrak{g}^*, \\
(g_{v_i}, \pi_{v_i}) \in G \times \mathfrak{g}^*,
\end{align}
$$
with left–invariant kinetic energy metrics defined by constant inertia tensors:
\begin{align}
\mathbb{I}_i : \mathfrak{g} \to \mathfrak{g}^*, \qquad
\mathbb{I}_{v_i} : \mathfrak{g} \to \mathfrak{g}^*.
\end{align}

---

## 2. Right–Invariant Dynamics

In momentum coordinates, each system evolves as:
$$
\begin{align}
\dot{g} = \omega \cdot g, \qquad
\dot{\pi} = f^e + f^u,
\end{align}
$$
where
- $ \omega = \mathbb{I}^{-1}\pi \in \mathfrak{g} $ is the body velocity,
- $ f^e \in \mathfrak{g}^* $ is an external (disturbance) force, and
- $ f^u \in \mathfrak{g}^* $ is the fully actuated control input.

For virtual systems $V_i$, $f^e_{v_i} \equiv 0$.

---

## 3. Agent Control Strategy

Each agent executes a **three–step strategy**:

1. **Local tracking objective**  
   Define
   $$
   \begin{align}
   g_r(t) = \bar{g}_i g_{v_i}(t),
   \end{align}
   $$
   where $ \bar{g}_i \in G $ is constant.  
   In $ SE(3) $, this represents maintaining a fixed pose offset relative to $V_i$.

2. **Agent tracking problem**  
   Choose $ f_i^u $ so that 
   $$ 
   \begin{align}
   \lim_{t \to \infty} g_i(t) = g_r(t).
   \end{align}
   $$
   If $ g_r(t) = \bar{g}_i g_{v_i}(t) $, then $ A_i $ converges to configuration $ \bar{g}_i $ relative to $V_i$.

3. **Virtual system synchronization**  
   Choose a communication strategy and $ f^u_{v_i} $ so that
   $$  
   \begin{align}
   \lim_{t \to \infty} g_{v_i}(t) = g_v(t), \quad \forall i,
   \end{align}
   $$
   ensuring asymptotic consensus of all virtual systems.

---

## 4. Error Dynamics

Define the **right–invariant tracking error**
$$
\begin{align}
e = g_v g_i^{-1}.
\end{align}
$$
Then:
$$
\begin{align}
\dot{e} &= \omega_e \cdot e, \qquad
\omega_e &= \omega_v - \operatorname{Ad}_e \omega_i.
\end{align}
$$

Define the **error momentum**
$$
\begin{align}
\pi_e = \operatorname{Ad}_{g_i}^* \mathbb{I}\operatorname{Ad}_{g_v^{-1}}\omega_e
       = \operatorname{Ad}_{e^{-1}}^*\pi_v - \pi_i.
\end{align}
$$

The **momentum error dynamics** (with right–invariant kinematics) are:
$$
\begin{align}
\boxed{
\dot{\pi}_e
= \operatorname{Ad}_{e^{-1}}^*\dot{\pi}_v
- \dot{\pi}_i
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi_v.
}
\end{align}
$$

---

## 5. AGLES–PID Control Law

Let $ f_e : G \to \mathbb{R} $ be a polar Morse function with unique minimum at the identity.  
Define

$$\begin{align}
df_e = \pi_e \cdot e, \qquad
\dot{\pi}_I = \pi_e.
\end{align}
$$

The **control law** is:
$$
\begin{align}
f_i^u =
\left(
\operatorname{Ad}_{e^{-1}}^*\dot{\pi}_r
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi_r
- f_i^e
\right)
- k_p\pi_e - k_d\pi_e - k_I\pi_I.
\end{align}
$$

The **closed-loop error dynamics** are:
$$
\begin{align}
\dot{e} = \omega_e \cdot e,\\
\dot{\pi}_I = \pi_e,\\
\dot{\pi}_e = -k_p\pi_e - k_d\pi_e - k_I\pi_I.
\end{align}
$$
> “The error dynamics do not get any simpler or more straightforward than this.”

---

## 6. Synchronization Strategies

### Centralized
A global controller broadcasts $g_v(t)$;  each agent uses its AGLES–PID law to track $g_v(t)$.  The broadcast can be encoded or key–protected.

### Decentralized
Each agent communicates with neighbors to share $(g_{v_i}(t), \pi_{v_i}(t))$ and applies a consensus-based control $ f^u_{v_i} $ to achieve
$$
\begin{align}
\lim_{t\to\infty} g_{v_i}(t) = g_v(t), \quad \forall i.
\end{align}
$$
---

## 7. Mathematical Conventions

**Coadjoint action:**  
$$
\begin{align}
\langle \operatorname{Ad}_g^*\mu, \zeta \rangle = \langle \mu, \operatorname{Ad}_{g^{-1}}\zeta \rangle
\end{align}
$$

**Right–invariant kinematics:**  
$$
\begin{align}
\dot{g} = \omega g
\end{align}
$$

**Derivative rule:**  
$$
\begin{align}
\frac{d}{dt}(\operatorname{Ad}_{e^{-1}}^*\pi)
= \operatorname{Ad}_{e^{-1}}^*\dot{\pi}
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi
\end{align}
$$


---

## 8. Key References

* D. H. S. Maithripala, J. M. Berg, D. H. A. Maithripala and S. Jayasuriya,  **"A geometric virtual structure approach to decentralized formation control,"**  *2014 American Control Conference*, Portland, OR, USA, 2014, pp. 5736–5741.  [PDF](https://ieeexplore.ieee.org/abstract/document/6859451)

* D. H. S. Maithripala and J. M. Berg,  **"An intrinsic PID controller for mechanical systems on Lie groups,"**  *Automatica*, Vol. 54, 2015, pp. 189-200.  [PDF](https://www.sciencedirect.com/science/article/pii/S0005109815000060)

* Rama Seshan Chandrasekaran, Ravi N. Banavar, Arun D. Mahindrakar, D. H. S. Maithripala,  **"Geometric PID controller for stabilization of nonholonomic mechanical systems on Lie groups,"**  *Automatica*, Vol. 165, 2024, 111658. [PDF](https://www.sciencedirect.com/science/article/pii/S0005109824001511)

* D. H. S. Maithripala, J. M. Berg, W. P. Dayawansa,  **"Almost–global tracking of simple mechanical systems on a general class of Lie Groups,"**  *IEEE Transactions on Automatic Control*, Vol. 51, No. 2, pp. 216–225, 2006. [PDF](https://ieeexplore.ieee.org/abstract/document/1593897)

* Reza Olfati-Saber and Naomi Ehrich Leonard,  **"Consensus and Cooperation in Networked Multi-Agent Systems,"**  *Proceedings of the IEEE*, Vol. 95, No. 1, 2007, pp. 215–233.  [PDF](https://ieeexplore.ieee.org/document/4118472)

---

### © 2025 — Research Notes on Geometric Control and Synchronization on Lie Groups
