## Parent
gravity_compensation_redesign_grill.md — Phase 1 (P0)

### What to build
编写 `computeFeedforward()` 的 RED 测试用例。这是 TDD 第一步：在实现任何新功能之前，先写出预期失败的测试。

三个基础测试用例（来自 grill 文档 §4c）：

1. **全零位测试**：所有关节角度为 0（竖直向上），期望所有关节的 τ_ff = 0。机械臂竖直时没有重力矩，这是最基础的验证。

2. **L2 水平 90° 测试**：L2 = 90°（水平），其余关节为 0。手工计算期望力矩值（基于 URDF 中的质量/质心参数），对比 NE 递推输出。验证单个非零关节的重力力矩计算正确性。

3. **scale=0 测试**：设置 scale=0，期望所有输出为 0。验证 scale 系数正确传导到最终输出。

测试 seam 为 `computeFeedforward()`（或当前 `computeGravityCompensation()`），同时覆盖辅助函数 `computeRotationMatrix()` 和 `crossProduct()`。

### Acceptance criteria
- [ ] 全零位测试通过：所有 τ_ff = 0
- [ ] L2 水平 90° 测试通过：τ_ff 输出与手工计算值的误差 < 2%
- [ ] scale=0 测试通过：所有分量均为 0
- [ ] 测试可独立运行：`ctest -V -R gravity_compensation` 可执行
- [ ] 测试在当前未重构代码上运行结果为 RED（预期失败）— 以确认测试本身有效

### Blocked by
None - can start immediately
