---
name: tdd
description: Test-driven development. Use when the user wants to build features or fix bugs test-first, mentions "red-green-refactor", or wants integration tests.
version: 1.0.0
triggers: tdd, test-driven, red-green-refactor, test first, write tests, integration test, unit test, 测试驱动, 先写测试, 红绿重构
metadata:
  openarm:
    structure:
      - SKILL.md
---

# TDD: 测试驱动开发

TDD 是 **红 → 绿** 的循环。本技能定义什么样的测试值得保留、测试应该放在哪里、反模式有哪些、以及循环的规则。每个规则在每个周期都适用——在循环之前和循环期间查阅，而不是循环之后。

---

## 循环规则

```
┌──────────┐     ┌──────────┐     ┌──────────┐
│  RED     │ →  │  GREEN   │ →  │ REFACTOR │
│ 写失败测试│     │ 最小实现  │     │ (Review) │
└──────────┘     └──────────┘     └──────────┘
      ↑                                  │
      └──────────────────────────────────┘
              下一个垂直切片
```

- **先红后绿**：先写失败的测试，然后只写足够的代码让它通过。不要预判未来的测试或添加投机性的功能。
- **一次一个切片**：一个 seam → 一个测试 → 一个最小实现，每个周期都如此。
- **重构不属于循环**：重构属于 Review 阶段（见 `code-review` skill），不属于红→绿实现循环。

---

## 什么是一个好的测试

测试通过**公共接口**验证行为，而非实现细节。代码可以完全改变；测试不应该变。

一个好测试的特征：
- **像规格说明一样可读**——"机械臂在激活后不移动"告诉你系统具备什么能力
- **不关心内部结构**——重构后测试不应失败
- **使用独立的期望值来源**——已知的字面量、手工计算的结果、或规格中的数据

```cpp
// GOOD: 测试可观测行为——通过公共接口验证
TEST(ArmHardwareTest, MotorNotEnabledAfterInit) {
    ArmHardware hw;
    EXPECT_FALSE(hw.motors_enabled());  // 通过公共查询接口
}

// GOOD: 期望值来自独立的已知数据
TEST(GravityCompensatorTest, CompensatesL2At90Degrees) {
    // 手工计算：sin(90°) = 1.0，factor=10.0 → 期望 10.0 Nm
    EXPECT_NEAR(compensator.compute(1, {0, 1.57, 0, 0, 0, 0, 0}), 
                10.0, 0.01);
}
```

---

## Seams —— 测试的放置位置

**Seam** 是测试所在的公共边界：可以不触及内部就能观察行为的接口。测试活在 seam 上，绝不在内部实现上。

**只在预先约定好的 seam 上写测试。** 写任何测试之前，先写下要测试的 seam 并与用户确认。不能在未确认的 seam 上写测试。无法测试所有内容——提前约定 seam 能让测试精力集中在关键路径和复杂逻辑上。

**问**："公共接口是什么？我们应该测试哪些 seam？"

### OpenARM 项目的典型 Seam

| 层 | Seam | 测试类型 |
|-----|------|---------|
| 硬件接口 | `export_state_interfaces()` / `export_command_interfaces()` | 接口合规性 |
| 硬件接口 | `read()` / `write()` 循环 | 集成行为 |
| 控制器 | `update()` 的公共输入/输出 | 单元行为 |
| 运动学 | IK 求解器的 `solve()` 接口 | 边界条件 |
| 视觉 | YOLO 检测的输入图像/输出框 | 集成行为 |
| Web | REST API 的请求/响应 | 端到端 |

---

## 反模式

### 1. 实现耦合（Implementation-coupled）

Mock 内部协作者、测试私有方法、或通过旁路验证（查询数据库而不是用接口）。标志：重构时测试断了但行为没变。

```cpp
// BAD: 测试实现细节——mock 了内部 CAN 调用
TEST(ArmHardwareTest, ReadCallsCanReceive) {
    MockCanPort mock_port;
    EXPECT_CALL(mock_port, Receive(_, _)).Times(7);  // 关注调用次数
    hw.read(time, period);
}

// GOOD: 测试可观测行为
TEST(ArmHardwareTest, ReadUpdatesJointPositions) {
    hw.read(time, period);
    // 直接验证公共状态接口的值
    EXPECT_GT(hw.get_position(0), -3.14);
    EXPECT_LT(hw.get_position(0), 3.14);
}
```

### 2. 同义反复（Tautological）

断言用和代码相同的方式重新计算了期望值，所以构造上必然通过。

```cpp
// BAD: 期望值是代码逻辑的镜像
TEST(GravityTest, ComputeCompensation) {
    double expected = factor * sin(total_theta) + load * g * length * sin(total_theta);
    EXPECT_EQ(compute(position), expected);  // 总是通过
}

// GOOD: 期望值来自独立来源（手工计算、已知物理意义）
TEST(GravityTest, L2At90DegreesCompensation) {
    // 机械臂水平伸展时重力矩 = m*g*L
    double expected = 2.5 * 9.8 * 0.3;  // 独立的物理计算
    EXPECT_NEAR(compute({0, M_PI/2, 0, 0, 0, 0, 0}), expected, 0.05);
}
```

### 3. 水平切片（Horizontal slicing）

先写所有测试，再写所有实现。批量测试验证的是**想象中的**行为——你测试的是事物的*形状*而不是面向用户的行为。

**用垂直切片代替**——一个测试 → 一个实现 → 重复，每个测试都是一个 **tracer bullet**，响应上一个周期教会你的东西。

---

## Mock 指南

### 何时 Mock

只在**系统边界**处 mock：

| Mock | 不 Mock |
|------|--------|
| CAN 总线通信（硬件边界） | 你自己的类/模块 |
| 传感器驱动（硬件边界） | 内部协作者 |
| 时间/随机数 | 你控制的任何东西 |
| 外部 API（LLM、视觉服务） | 运动学/控制器内部逻辑 |

### 依赖注入

```cpp
// GOOD: 可注入 CanPort，测试可替换
class ArmHardware {
    CanPort& can_port_;  // 通过构造注入
public:
    ArmHardware(CanPort& can) : can_port_(can) {}
};

// BAD: 内部创建，无法替换
class ArmHardware {
    CanPort can_port_;  // 硬编码创建
};
```

### 边界接口设计

```cpp
// GOOD: 每个操作独立可 mock
struct MotorAPI {
    virtual bool enter_mit_mode(int node_id) = 0;
    virtual bool send_cmd(int motor_id, MitCommand cmd) = 0;
    virtual bool read_fb(int motor_id, MitFeedback& fb) = 0;
};

// BAD: 通用接口需要条件逻辑
struct MotorAPI {
    virtual bool execute(int motor_id, int cmd_type, void* data) = 0;
};
```

---

## OpenARM 项目的 TDD 示例

### 示例 1：重力补偿 bug 修复（RED → GREEN）

```cpp
// RED：先写失败的测试
TEST(GravityCompensationTest, L1ReturnsZero) {
    // L1（基座旋转关节）不受重力影响
    EXPECT_DOUBLE_EQ(compute_gravity_compensation(0), 0.0);
}

TEST(GravityCompensationTest, OutOfRangeReturnsZero) {
    EXPECT_DOUBLE_EQ(compute_gravity_compensation(-1), 0.0);
    EXPECT_DOUBLE_EQ(compute_gravity_compensation(7), 0.0);
}

// GREEN：实现边界检查（已在 arm_hardware.cpp#L231-L233 中）
// if (joint_index < 0 || joint_index >= 7) { return 0.0; }
```

### 示例 2：启动保护新功能（RED → GREEN）

```cpp
// RED：写测试
TEST(StartupProtectionTest, IgnoresCommandsDuringProtection) {
    ArmHardware hw;
    hw.on_activate(prev_state);
    
    // 模拟控制器发送非零命令
    hw.set_command_position(0, 1.0);
    hw.write(time, period);
    
    // 保护期间位置应保持不变
    EXPECT_NEAR(hw.get_command_position(0), hw.get_position(0), 0.001);
}

// GREEN：实现启动保护逻辑（arm_hardware.cpp#L763-L792）
```

---

## 与 robot-sprint 的配合

```
robot-sprint 流程中 TDD 的使用时机：

Phase 3: BUILD
  └─ TDD 在此阶段使用
      RED   → 基于 to-issues 的 Issue 写失败测试
      GREEN → 最小实现让它通过
      → 下一个垂直切片

Phase 4: REVIEW
  └─ REFACTOR 在此阶段
      → code-review 双轴审查
      → 重构优化（不新增行为）
```
