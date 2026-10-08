## Parent
gravity_compensation_redesign_grill.md — Phase 1 (P0)，分支 4a

### What to build
重构重力补偿的公共接口，从当前的 `computeGravityCompensation()` 改为 `computeFeedforward()`。

**新接口设计**（来自 grill 文档 §4a）：

```cpp
struct FeedforwardResult {
    std::array<double, 7> tau_ff;   // 总前馈力矩 = scale × (gravity + friction)
    std::array<double, 7> tau_g;    // 纯重力分量（调试/monitoring 用）
    std::array<double, 7> tau_f;    // 纯摩擦分量（调试/monitoring 用）
};

FeedforwardResult computeFeedforward(
    const std::array<double, 7>& joint_angles,
    const std::array<double, 7>& joint_velocities,
    double hand_mass,
    double payload);
```

变更范围（grill 文档确认记录 #1：**全改**）：
- `NewtonEulerGravityCompensator` 类的公共方法改名 + 签名扩展
- `arm_hardware.cpp` 中所有调用方一并修改
- 添加 `joint_velocities` 参数（当前为空，Phase 2 摩擦模型使用）

重构后 `FeedforwardResult.tau_f` 暂时全为 0（摩擦模型尚未实现）。

### Acceptance criteria
- [ ] `computeFeedforward()` 签名已改，返回 `FeedforwardResult` 结构体
- [ ] 所有调用方（`arm_hardware.cpp` 等）已同步更新，编译通过
- [ ] 旧接口 `computeGravityCompensation()` 已移除，无残留
- [ ] #1 的 RED 测试在新接口下编译通过（仍为 RED，因为内部逻辑未变）
- [ ] 接口文档注释完备（Doxygen 格式）

### Blocked by
- #01 (TDD RED tests) — 必须先有测试，再改接口，确保重构不破坏行为
