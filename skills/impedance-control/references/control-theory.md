# Impedance Control Theory

## 1. Core Control Equation

Computed every control cycle (typically 100–1000 Hz, depending on motor):

```
τ_cmd = τ_ff + Kp·(q_des - q) + Kd·(dq_des - dq)
```

| Term | Name | What it does |
|---|---|---|
| `τ_ff` | Feedforward torque | Cancels gravity + friction (lets arm "float") |
| `Kp·(q_des - q)` | Position spring | Pulls joint toward target (higher Kp = stiffer) |
| `Kd·(dq_des - dq)` | Velocity damping | Prevents overshoot (higher Kd = more damping) |

**Analogy:** Kp is the spring, Kd is the shock absorber. τ_ff cancels the arm's weight so the spring only fights disturbances, not gravity.

---

## 2. Gravity Compensation (τ_ff)

```
τ_ff = tau_ff_scale[j] × (τ_gravity[j] + τ_friction[j])
```

### Gravity Torque

Computed using **KDL recursive Newton-Euler** from URDF link masses and inertias, updated each cycle based on current joint positions.

If you don't have KDL available, you can implement your own recursive Newton-Euler or use a simplified static gravity model.

### Friction Model

```
τ_friction = Fc·tanh(0.1·k·dq) + Fv·dq + Fo
```

| Parameter | Meaning | How to get it |
|---|---|---|
| `Fc` | Coulomb friction magnitude | Calibrate from constant-velocity sweeps |
| `k` | Coulomb sharpness (tanh slope) | Higher = steeper zero-velocity transition |
| `Fv` | Viscous friction coefficient | Proportional to velocity |
| `Fo` | Static offset | Asymmetric friction (gravity bias at rest) |

### tau_ff_scale

Per-joint multiplier (typically 0.85–1.05) for fine-tuning gravity compensation:

| Condition | Behavior | Adjustment |
|---|---|---|
| Too low | Arm drifts down in teach mode | Increase tau_ff_scale |
| Too high | Arm drifts up in teach mode | Decrease tau_ff_scale |

### Calibration Guide

1. Put the arm in teach mode (Kp = kp_min)
2. For each joint, move it to various positions and hold
3. Observe drift direction (up = too much compensation, down = too little)
4. Adjust tau_ff_scale in small increments (±0.02 to ±0.05)
5. Repeat until drift is minimal across the workspace

---

## 3. Proprioceptive Force Estimation

Estimate external forces from motor output when no force/torque sensor is available.

### Algorithm

Each control cycle:

```
τ_ext_raw[i] = τ_motor[i] - τ_ff[i]
τ_ext[i] = alpha × τ_ext_raw[i] + (1 - alpha) × τ_ext_prev[i]
```

| Term | Source | Meaning |
|---|---|---|
| `τ_motor` | Motor output state interface | Actual torque per joint |
| `τ_ff` | Gravity + Coriolis + friction model | Expected torque without external forces |
| `alpha` | Low-pass filter coefficient (e.g., 0.05) | Cutoff ~0.8 Hz at 100Hz |

### Typical Accuracy

- **Expected accuracy:** ~±0.5 Nm (limited by friction)
- **Good for:** contact detection, collision detection, rough force feedback
- **Not good for:** precision force control (use a real F/T sensor for that)

### Use cases

1. **Collision detection:** If τ_ext exceeds threshold, stop or back off
2. **Human-robot interaction:** Detect when a human touches the arm
3. **Grasp verification:** Detect if an object is being held (gripper force estimation)
4. **Assembly tasks:** Detect contact with the environment

### Limitations

- **Accuracy limited by friction** (~±0.5 Nm typical)
- **No acceleration term** — M(q)·q̈ not accounted for → transient errors during fast motion
- **Sensor noise** — motor current sensing has noise; low-pass filter adds ~200ms latency
- **Simulation problem** — simulation may return effort=0, making τ_ext meaningless for testing

### Two Approaches to Closed-Loop Force Control

| Approach | How it works | Pros | Cons | Best For |
|---|---|---|---|---|
| **A: Estimate payload** | τ_ext → mass = τ/(g×lever) → set_payload → rebuild KDL | Physically correct model | Needs estimation algorithm | Grasping known objects |
| **B: Disturbance observer** | Direct τ_ff[i] += gain × τ_ext[i] | Immediate, adapts to any disturbance | Can amplify noise | Contact tasks, HRI |

---

## 4. Relation to CompliantVLA Research

The impedance controller implements the **VIC (Variable Impedance Controller)** layer of the CompliantVLA-adaptor framework:

```
VLM  (~1 Hz)   → decides impedance profile / task phase
VLA  (~3 Hz)   → generates target joint trajectories
VIC  (100+ Hz) → impedance / force control (this controller)
```

To achieve research-grade VLM-guided impedance: replace rule-based profile switching with VLM-generated Kp/Kd parameters. The framework is ready — the impedance controller can accept arbitrary Kp/Kd values at runtime.
