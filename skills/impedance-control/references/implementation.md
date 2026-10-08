# Impedance Control Implementation

## 1. Control Loop Architecture

### Typical control loop structure

```
┌─────────────────────────────────────────────────────┐
│                Control Loop (100 Hz)                 │
│                                                      │
│  1. Read joint positions/velocities from hardware    │
│  2. Compute gravity + friction compensation (τ_ff)   │
│  3. Compute impedance term: Kp·(q_des-q) + Kd·(...) │
│  4. Sum: τ_cmd = τ_ff + impedance                    │
│  5. Apply rate limits to Kp/Kd (profile switching)   │
│  6. Clamp τ_cmd to torque limits                     │
│  7. Send τ_cmd to motors                             │
│  8. Publish state (joints, forces, etc.)             │
└─────────────────────────────────────────────────────┘
```

### Key modules

| Module | Responsibility |
|---|---|
| Hardware interface | Read/write motor commands (CAN/EtherCAT/USB) |
| Gravity model | Compute τ_gravity from kinematics + inertial parameters |
| Friction model | Compute τ_friction from velocity + friction params |
| Impedance controller | Compute Kp·e + Kd·ė |
| Profile manager | Manage Kp/Kd presets and smooth transitions |
| Force estimator | Compute τ_ext from τ_motor - τ_ff |
| Safety monitor | Check torque/velocity/workspace limits, E-stop |

---

## 2. Impedance Profiles (Presets)

Typical impedance presets for different phases of a task:

| Profile | Kp (shoulder) | Kp (elbow) | Kp (wrist) | Use Case |
|---|---|---|---|---|
| **transit** | 60–80 | 50–70 | 8–15 | Moving to target (stiff, precise) |
| **approach** | 40–60 | 30–50 | 5–10 | Approaching object (medium compliance) |
| **contact** | 20–40 | 15–30 | 3–6 | Touching/grasping (soft, force-limited) |
| **grasp** | 60–80 | 50–70 | 8–15 | Holding object (stiff arm + firm grip) |
| **teach** | 10–15 | 2–5 | 1–3 | Human demonstration (near kp_min, floating) |

> These are starting points. Tune based on your motor's torque limits, link masses, and safety requirements.

### Rate Limiting

Kp/Kd changes should be rate-limited to prevent sudden jerks:
- `delta_kp_max` per cycle → full transit→teach transition in ~0.3–0.5 seconds
- `delta_kd_max` per cycle → smooth damping changes

### Safe Switching Rules

- Never switch from stiff to soft instantaneously — always ramp
- Monitor motor current during transitions
- Add a deadband around the target position during profile switches

---

## 3. Teach Mode (Demonstration Mode)

**Purpose:** Make the arm "weightless" so a human can drag it around to record demonstration trajectories (for VLA / imitation learning).

### How it works

1. Kp set to kp_min (near-zero stiffness)
2. τ_ff remains active (gravity compensation keeps arm floating)
3. q_des tracks current position (no restoring force)
4. Gripper also compliant (low grip_kp)

### Recording demonstrations

1. Enter teach mode
2. Human drags arm through desired trajectory
3. Record joint positions at ~50–100 Hz
4. Exit teach mode (arm returns to controlled state)
5. Save trajectory for training / playback

### Safety notes

- Never leave teach mode unattended. Due to imperfect gravity compensation, the arm will drift slowly.
- Always have an E-stop within reach during human-robot interaction.
- Clear the workspace of obstacles before entering teach mode.
- Joint velocity limits should still be enforced even in teach mode.

---

## 4. Related Repositories

| Repository | Purpose |
|---|---|
| OpenArm-ROS2-robot-control | ROS 2 hardware interface and bringup |
| Openarm_ROS2_Impedance_Control | Reference impedance/compliance controller implementation |
| Openarm_VLA | Vision-Language-Action integration |
