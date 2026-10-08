# Tuning Guide & Troubleshooting

## 1. Step-by-Step Tuning Procedure

1. **Start with Kp=0, Kd=0, τ_ff disabled**
   - Arm should be completely limp (be ready to catch it!)

2. **Enable gravity compensation first**
   - Set τ_ff with approximate tau_ff_scale values
   - Arm should float in place with minimal drift
   - Fine-tune tau_ff_scale per joint

3. **Add damping (Kd)**
   - Start with low Kd values
   - Push each joint and verify it returns to rest smoothly
   - Increase until oscillations are damped within 1–2 cycles

4. **Add stiffness (Kp)**
   - Start with low Kp (e.g., 5–10 for large joints)
   - Gradually increase until desired stiffness is achieved
   - Watch for oscillation — if it oscillates, increase Kd or decrease Kp

5. **Verify across workspace**
   - Test at different joint configurations
   - Gravity compensation quality varies with pose
   - Ensure no joint hits torque limits

---

## 2. Quick Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| Robot oscillates | Kp too high relative to Kd | Increase Kd or decrease Kp. Rule of thumb: Kd ≈ 2·√(Kp)/30 |
| Robot is too sluggish | Kp too low | Increase Kp in steps of 5–10, verify no oscillation |
| Arm drifts in teach mode | tau_ff_scale off for that joint | Adjust per joint by ±0.02 to ±0.05 |
| Gripper can't hold object | gripper_kp too low | Increase gripper_kp (max ~10.0 N·m/rad for small motors) |
| Jerky motion | Too-high Kp jumps or no rate limiting | Add rate limiting to Kp/Kd changes |
| Noisy force estimation | Low alpha or high friction | Lower alpha (more filtering), improve friction model |

---

## 3. Safety Considerations

### Torque limits

- Always enforce per-joint torque limits from the motor datasheet
- Add a software torque limit below the hardware limit for safety margin
- Monitor motor temperature during prolonged operation

### Velocity limits

- Enforce joint velocity limits in all modes (including teach mode)
- Derate velocity limits when carrying payloads

### Workspace limits

- Define safe workspace boundaries (joint limits + Cartesian limits)
- Soft limits: reduce Kp as joint approaches limit
- Hard limits: stop motion before physical end stops

### Emergency stop

- Hardware E-stop button that cuts motor power
- Software E-stop that sets all Kp/Kd/τ_ff to zero and engages brakes
- E-stop should be tested regularly
