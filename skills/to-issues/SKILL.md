---
name: to-issues
description: Break a plan, spec, or PRD into independently-grabbable issues on the project issue tracker using tracer-bullet vertical slices. Use when user wants to convert a plan into issues, create implementation tickets, or break down work into issues.
version: 1.0.0
triggers: to-issues, break into issues, create issues, issue breakdown, implementation tickets, task breakdown, work breakdown, vertical slices, tracer bullet, 拆解任务, 创建 issue, 任务拆分, 工作拆分
metadata:
  openarm:
    structure:
      - SKILL.md
---

# To-Issues: 计划拆解为独立 Issue

将计划、规格说明或 PRD 拆解为**可独立认领的 Issue**，使用 tracer-bullet（曳光弹）垂直切片策略。每个 Issue 贯穿所有层次（schema → API → UI → tests），是可单独交付和验证的完整路径。

---

## 流程

### Step 1: 收集上下文

- 使用会话中已有的计划/PRD 作为输入
- 如果用户传递了 Issue 引用（编号、URL、路径），从 issue tracker 获取完整内容
- 确保理解项目的领域词汇和 ADR（架构决策记录）

### Step 2: 探索代码库（可选）

- 如果尚未探索代码库，先了解当前代码状态
- Issue 标题和描述应使用项目的领域术语
- 尊重已有的 ADR 决策

### Step 3: 起草垂直切片

将计划拆解为 **tracer bullet** issue：

| 属性 | 说明 |
|------|------|
| **垂直切片** | 每个 Issue 贯穿所有集成层次，不是单层水平切分 |
| **完整路径** | schema → API → UI → tests，端到端 |
| **可演示** | 完成的切片可独立 demo 或验证 |
| **细粒度** | 宁可多个薄切片，不要少数厚切片 |

**Issue 类型分类**：

| 类型 | 含义 | 示例 |
|------|------|------|
| **HITL**（Human in the Loop） | 需要人工交互，如架构决策、设计评审 | 「确认 CAN-FD 迁移方案」 |
| **AFK**（Away From Keyboard） | 无需人工交互，可直接实现和合并 | 「实现 JointStateBroadcaster 发布」 |

> 优先使用 AFK，减少阻塞。

### Step 4: 与用户确认

以编号列表呈现拆解方案，每个切片显示：

- **标题**：简短描述
- **类型**：HITL / AFK
- **阻塞关系**：依赖哪些其他切片
- **覆盖的用户故事**：对应源材料中的哪些需求

向用户确认：
- 粒度是否合适？（太粗 / 太细）
- 依赖关系是否正确？
- 是否有切片需要合并或进一步拆分？
- HITL 和 AFK 标记是否正确？

迭代直到用户批准。

### Step 5: 发布到 Issue Tracker

对每个批准的切片，按以下模板创建 Issue。按依赖顺序发布（先发布被依赖的），以便在 "Blocked by" 中引用真实的 Issue ID。

---

## Issue 模板

```markdown
## Parent
<!-- 父 Issue 引用（如果源来自已有 issue） -->

### What to build
<!-- 简洁描述端到端行为，不要逐层描述。
     避免具体文件路径或代码片段——它们很快过时。
     例外：如果原型中的代码片段比文字更能精确表达决策
     （状态机、reducer、schema、类型定义），可内联并注明来源。 -->

### Acceptance criteria
- [ ] 验收条件 1
- [ ] 验收条件 2
- [ ] 验收条件 3

### Blocked by
<!-- 阻塞此 Issue 的依赖 Issue 引用，无依赖则写 "None - can start immediately" -->
```

---

## OpenARM 项目适配

### 机器人项目的 Tracer Bullet 示例

```
计划：「为左臂添加碰撞检测与柔顺停止功能」

垂直切片拆解：

Slice 1 [AFK]：在 arm_hardware 中导出 effort 状态接口
  ├─ 贯穿：URDF → SystemInterface → export_state_interfaces
  └─ 可验证：ros2 control list_hardware_interfaces 可见 effort

Slice 2 [AFK]：实现力矩残差计算（τ_ext = τ_motor - τ_ff - τ_friction）
  ├─ 贯穿：read() → compute_gravity_compensation → τ_ext 发布
  └─ 可验证：ros2 topic echo /joint_external_torque 有数据

Slice 3 [HITL]：确定碰撞阈值与停止策略
  ├─ 需要：实验数据 + 安全评审
  └─ 输出：阈值配置文档

Slice 4 [AFK]：实现碰撞检测节点
  ├─ 贯穿：订阅 τ_ext → 阈值判断 → 发布碰撞事件
  └─ 可验证：模拟碰撞信号触发停止

Slice 5 [AFK]：集成 MoveIt2 碰撞停止
  ├─ 贯穿：碰撞事件 → MoveIt2 trajectory 中断 → 停止
  └─ 可验证：规划中碰撞 → 运动停止 → 状态恢复
```

### 与 robot-sprint 的配合

```
robot-sprint 流程中 to-issues 的使用时机：

Phase 2: PLAN
  └─ to-issues 在此阶段使用
      → 将已对齐的设计拆解为独立 Issue

Phase 3: BUILD
  └─ 逐个认领 AFK Issue 实现
      → 每个 Issue 是新会话的起点

Phase 4: REVIEW → 按 Issue 逐一审查
Phase 5: TEST   → 按 Issue 逐一切片验证
```

---

## 注意事项

- **不要关闭或修改父 Issue**——只创建新的子 Issue
- **AFK Issue 应完全独立**——Agent 可自主完成从实现到合并的全流程
- **薄切片优于厚切片**——宁可 10 个小 PR 也不要 1 个巨型 PR
- **依赖顺序发布**——先发布被依赖的 Issue，后续 Issue 可引用真实 ID
- **领域语言一致**——使用项目中已定义的术语，保持与 URDF、MoveIt 配置一致
