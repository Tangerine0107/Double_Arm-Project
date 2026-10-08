## Parent
gravity_compensation_redesign_grill.md — Phase 2 (P1)，分支 2

### What to build
实现完整的摩擦补偿模型（grill 文档标准公式）：

```
τ_friction = Fc·tanh(0.1·k·dq) + Fv·dq + Fo
```

**四个参数**：

| 参数 | 含义 | 默认值 | 物理来源 |
|---|---|---|---|
| Fc | 库仑摩擦 (Nm) | 0.0 | 关节轴承、齿轮啮合 |
| k | 库仑锐度 | 100.0 | 过渡区宽度控制 |
| Fv | 粘性摩擦系数 (Nm·s/rad) | 0.0 | 润滑剂粘滞阻力 |
| Fo | 静态偏移 (Nm) | 0.0 | 重力不对称负载 |

**设计决策**（grill 文档分支 2b）：
- 在 `NewtonEulerGravityCompensator` 内部计算，与 gravity 统一返回
- `computeFeedforward()` 中：`tau_ff = scale × (tau_g + tau_f)`
- 参数全部默认 0，通过 YAML 逐步启用 — 效果等同关闭，现有行为不变

**为什么用 `tanh` 而不是 `sign`**（grill 文档分支 2 解释）：
- `sign(dq)` 在零速处不连续 → 控制抖动
- `tanh(0.1·k·dq)` 光滑、处处可微 → 零速附近无突变
- `k=100` 时 tanh 过渡区宽度约 0.02 rad/s

**计算位置**（grill 文档确认结论）：
- 不拆成独立类，直接在 NE 类内部新增 `computeFrictionCompensation()`
- 不对外暴露单独摩擦接口，统一通过 `computeFeedforward()` 返回
- `FeedforwardResult.tau_f` 填充摩擦分量值（调试用）

### Acceptance criteria
- [ ] `computeFrictionCompensation()` 实现正确（参数默认 0）
- [ ] `computeFeedforward()` 返回的 `tau_ff` = scale × (τ_g + τ_f)
- [ ] `FeedforwardResult.tau_f` 正确填充摩擦分量
- [ ] 零速时 τ_f = 0（无运动则无摩擦）
- [ ] 低速运动时 τ_f 方向与速度方向一致
- [ ] ROS2 物理计算满足 200Hz 周期（新增耗时 < 0.1ms）
- [ ] 单元测试：给定 (Fc,Fv,Fo) 和 dq，手工验算输出

### Blocked by
- #05 (tau_ff_scale) — 依赖 scale 通道就绪后串入总公式
