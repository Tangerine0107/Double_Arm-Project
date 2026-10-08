## Parent
gravity_compensation_redesign_grill.md — Phase 1 (P0)，分支 1a + 分支 5

### What to build
将当前硬编码在 `gravity_compensation.cpp` 中的连杆物理参数（质量、质心、变换矩阵）改为从 URDF 动态加载。

**设计决策**（grill 文档 §1a）：
- 在 `NewtonEulerGravityCompensator` 构造函数中接受 URDF 路径或 `urdf::Model` 引用
- 遍历 7 个关节（L1~L7）的 `<link>/<inertial>/<mass>` 和 `<origin>` 提取参数
- 缓存到现有的 `link_inertias_[]` 和 `link_transforms_[]` 数组中
- 仅构造时解析一次（满足实时性要求）
- 保留当前硬编码参数作为编译期 fallback（找不到 URDF 时降级使用）

**配置管理原则**（grill 文档分支 5）：
- 物理参数（质量/质心/变换）→ 从 URDF 加载（单一真相来源）
- 调优参数（scale/摩擦）→ 从 YAML 加载（控制工程师的调参本）
- 惯性张量：从 URDF 顺带提取并保留在结构体中，但不参与当前重力计算（仅注释标注"预留"）

**编译依赖**：引入 `urdfdom`（或 `tinyxml2`）。grill 文档确认记录 #2：接受 urdfdom 依赖。

### Acceptance criteria
- [ ] `NewtonEulerGravityCompensator` 构造函数接受 `urdf::Model` 引用
- [ ] URDF 路径从 `arm_hardware` 的 `HardwareInfo` 中获取
- [ ] 质量/质心/变换参数从 URDF 正确提取并缓存
- [ ] 找不到 URDF 时降级到硬编码参数（不崩溃）
- [ ] #1 的测试在 URDF 加载模式下通过（GREEN）
- [ ] 编译依赖 `urdfdom` 已在 CMakeLists.txt 中声明
- [ ] `#include <urdf/model.h>` 已添加

### Blocked by
- #02 (接口重构) — 接口稳定后再改内部实现
