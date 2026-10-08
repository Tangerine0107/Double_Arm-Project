---
name: impedance-control
description: Impedance / compliance control knowledge base for the OpenARM humanoid robot arm — control theory, gravity compensation, friction modeling, teach mode, proprioceptive force estimation, and tuning guide. Use when implementing or tuning compliant controllers, gravity compensation, teach/demonstration mode, or force estimation for OpenARM.
version: 2.0.0
triggers: impedance control, compliance control, gravity compensation, friction model, teach mode, force estimation, proprioception, VIC, compliant control, Kp Kd tuning
metadata:
  openarm:
    requires:
      ros2_packages: ["ros2_control", "controller_manager"]
      cpp_headers: ["kdl/chainidsolver_recursive_newton_euler.hpp"]
    structure:
      - SKILL.md
      - references/
        - control-theory.md
        - implementation.md
        - tuning-guide.md
      - scripts/
        - gravity_comp_calibrator.py
---

# OpenARM Impedance Control Knowledge Base

Complete reference for impedance (compliance) control on the OpenARM humanoid robot arm platform. Covers control theory, implementation details, tuning, teach mode, and proprioceptive force estimation.

---

## Quick Start

### Core Equation

```
τ_cmd = τ_ff + Kp·(q_des - q) + Kd·(dq_des - dq)
```

| Term | Name | What it does |
|---|---|---|
| `τ_ff` | Feedforward torque | Cancels gravity + friction (lets arm "float") |
| `Kp·(q_des - q)` | Position spring | Pulls joint toward target (higher Kp = stiffer) |
| `Kd·(dq_des - dq)` | Velocity damping | Prevents overshoot (higher Kd = more damping) |

**Analogy:** Kp is the spring, Kd is the shock absorber. τ_ff cancels the arm's weight so the spring only fights disturbances, not gravity.

### Typical Impedance Profiles

| Profile | Kp (shoulder) | Kp (elbow) | Kp (wrist) | Use Case |
|---|---|---|---|---|
| **transit** | 60–80 | 50–70 | 8–15 | Moving to target (stiff, precise) |
| **approach** | 40–60 | 30–50 | 5–10 | Approaching object (medium compliance) |
| **contact** | 20–40 | 15–30 | 3–6 | Touching/grasping (soft, force-limited) |
| **teach** | 10–15 | 2–5 | 1–3 | Human demonstration (floating) |

### Tuning Order (Critical!)

1. **Gravity compensation first** — get τ_ff right before tuning Kp/Kd
2. **Damping (Kd) next** — add damping before stiffness to prevent oscillation
3. **Stiffness (Kp) last** — increase gradually, watch for oscillation
4. **Always rate-limit profile switches** — sudden Kp changes cause jerks

---

## Reference Documents

| Document | Contents | When to Read |
|----------|----------|--------------|
| [control-theory.md](references/control-theory.md) | Core equation, gravity compensation, friction model, force estimation, CompliantVLA relation | Understanding the physics and math |
| [implementation.md](references/implementation.md) | Control loop architecture, impedance profiles, teach mode, key modules | Building or modifying the controller |
| [tuning-guide.md](references/tuning-guide.md) | Step-by-step tuning procedure, troubleshooting table, safety considerations | Tuning gains or debugging behavior |

---

## Scripts

| Script | Purpose |
|--------|---------|
| [scripts/gravity_comp_calibrator.py](scripts/gravity_comp_calibrator.py) | Interactive gravity compensation calibration — measures drift in teach mode and suggests tau_ff_scale adjustments |

---

## Important Reminders

- **Gravity compensation is the foundation.** Get τ_ff right before tuning Kp/Kd.
- **Start soft, get stiffer.** Always start with low gains and increase gradually.
- **Damping before stiffness.** Add Kd first, then Kp — prevents oscillations.
- **Always rate-limit profile switches.** Sudden Kp changes cause jerks and can damage motors or the environment.
- **Force estimation is approximate.** Don't rely on proprioception for precision force control.
- **Safety first.** The arm can move fast and with force. Always have E-stop accessible.
- **Never leave teach mode unattended.** Imperfect gravity compensation causes slow drift.
