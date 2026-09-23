# Robust Regulation and PSpice Validation of Three-Phase AC/DC Converters

![MATLAB](https://img.shields.io/badge/MATLAB-R2021a%2B-orange?logo=mathworks)
![PSpice](https://img.shields.io/badge/OrCAD-PSpice-blue)
![Status](https://img.shields.io/badge/Status-Under%20Review-yellow)
![License](https://img.shields.io/badge/License-MIT-green)

Simulation, mathematical modeling, and experimental validation repository for the manuscript:
> **"Robust Regulation of Three-Phase AC/DC Converters: Stability and Experimental Validation"**  
> *Rodolfo Verdín and Gerardo Flores*  
> Submitted to *IEEE Transactions on Control Systems Technology* (Under Review).

---

## 📌 Project Overview

This repository provides a complete open-source simulation environment and hardware implementation workflow for controlling a **three-phase PWM rectifier** with unity power factor under:
- Unknown constant and dynamic DC-bus load variations ($\delta(t)$).
- Actuator/modulation index saturation limits ($p \in \mathcal{S}$).
- Grid disturbances and parameter uncertainties.

The design features a **Passivity-Based Adaptive Controller** operating on the passive output of the converter, adapting the active current reference via an estimate of equivalent bus conductance ($\hat{g}$) paired with a filtered, bounded proportional action.

---

## 📐 Dynamic Model & Mathematical Formulation

Applying the amplitude-invariant **Park transformation** synchronized with the grid via PLL ($i_q = 0 \implies \text{Unity Power Factor}$), the averaged bilinear dynamic model of the rectifier in the $dq$ frame is given by:

$$L \dot{i}_d = -R i_d - \frac{1}{2} p_d v_o + L \omega i_q + E_m$$

$$L \dot{i}_q = -R i_q - \frac{1}{2} p_q v_o - L \omega i_d$$

$$C \dot{v}_o = \frac{3}{4} (p_d i_d + p_q i_q) - g_L v_o + C \delta(t)$$

Where:
- $i_d, i_q$ : Active and reactive currents in the synchronous $dq$-frame.
- $v_o$ : DC-link output voltage (Target reference: $v_o^d$).
- $p_d, p_q$ : Averaged modulation indices subject to saturation set $\mathcal{S} = [-a_d, a_d] \times [-a_q, a_q]$.
- $E_m, \omega$ : Grid voltage peak amplitude and fundamental frequency.
- $g_L, \delta(t)$ : Load conductance ($1/R_L$) and external current disturbances.

---

## 🛠️ Proposed Adaptive Control Law

The proposed non-cascaded controller is defined as follows:

**1. Saturated Port Modulation Feedforward & Damping:**

$$p_d = \text{sat}_{a_d} \left( p_d^{eq}(g_r) - \frac{2 k_s}{3} \left[ I(g_r) (v_o - v_o^d) - v_o^d (i_d - I(g_r)) \right] \right)$$

$$p_q = \text{sat}_{a_q} \left( p_q^{eq}(g_r) + \frac{2 k_q}{3 v_o^d} i_q \right)$$

**2. Conductance Reference Adaptation & Filtered Damping:**

$$g_r = \hat{g} - \kappa \bar{\nu} \tanh(\nu / \bar{\nu})$$

$$\dot{\nu} = \lambda_f (v_o - v_o^d - \nu)$$

$$\dot{\hat{g}} = \text{Proj} \left( \hat{g}, \, -2\gamma v_o^d (v_o - v_o^d) - \gamma \sigma (\hat{g} - g_c) \right)$$

Where $I(g)$ represents the active reference current derived from the non-linear power balance equation:

$$I(g) = \frac{E_m - \sqrt{E_m^2 - \frac{8}{3} R (v_o^d)^2 g}}{2 R}$$

---

## 📁 Repository Structure

```text
.
├── PSpice_files/          # OrCAD PSpice component-level schematics (.DSN, .OPJ)
├── Simulink_Simulation/   # Closed-loop converter models (.SLX) in Simscape/Power Electronics
└── matlab_files/          # Parameter scripts (.M) and dynamic validation tests

```

git clone [https://github.com/Rodolfo9706/Robustcontroller-for-three-phase-converters.git](https://github.com/Rodolfo9706/Robustcontroller-for-three-phase-converters.git)
cd Robustcontroller-for-three-phase-converters

run('matlab_files/paramet.m')

Metric / Method,Proposed,Grosso et al. [22],Pérez et al. [19],Verrelli et al. [21],Hernández et al. [20],Classical PI
Max Voltage Error (∣e3​∣),5.1 V,3.5 V,4.8 V,4.8 V,7.5 V,16.3 V
RMS Voltage Error (e3​),0.56 V,0.27 V,0.39 V,0.39 V,2.05 V,3.99 V
Minimum Power Factor,1.000,0.745,1.000,1.000,0.992,1.000
Startup Overshoot (vo​),169 V,213 V,311 V,2276 V (Unstable),235 V,198 V
1% Settling Time,0.10 s,0.41 s,Unstable,Failed,>1.5 s,0.44 s
Param. Error Drift,0.0 V,0.0 V,Unstable,0.8 V,19.2 V,0.0 V
Stability Guarantee,Global + Saturation,Global + Saturation,None,Local Exponential,Global Adaptive,None

@article{verdin2026robust,
  title={Robust Regulation of Three-Phase AC/DC Converters: Stability and Experimental Validation},
  author={Verd{\'i}n, Rodolfo and Flores, Gerardo},
  journal={submitted to IEEE Transactions on Control Systems Technology (Under Review)},
  year={2026}
}
