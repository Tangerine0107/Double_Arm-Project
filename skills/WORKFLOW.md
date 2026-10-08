# OpenARM Skills 工作流：怎么用这四个技能做功能开发

> 适用目录：`skills/` + `tasks/`  
> 目标：把「领域知识 → 设计 → 验收规格 → 任务切片 → 实现」变成可重复流程  
> 示例任务：重力补偿（`tasks/gravity_compensation/`）

---

## 0. 先记住一句话

```text
skills/  = 可复用的能力（知识 + 规范 + 方法）
tasks/   = 当前任务的工件（设计稿、测试规格、Issue 清单）
```

四个 skill 各干一件事：

| Skill | 角色 | 一句话 |
|---|---|---|
| **domain skill**（如 impedance-control） | 教科书 | 告诉你“这个问题的正确物理/公式是什么” |
| **karpathy-guidelines** | 纪律 | 先想再写、少改、可验证再收工 |
| **tdd** | 验证方法 | 先定“怎么算过”，实现时红→绿 |
| **to-issues** | 施工拆解 | 把大设计拆成可独立交付的垂直切片 |

---

## 1. 正确总顺序（不要记反）

```text
① 读领域知识 (domain skill)
        ↓
② 设计分析 / Grill (karpathy: 先想再写)
        ↓  输出: tasks/<任务>/xxx_grill.md
③ 写测试策略 (tdd 的“规格层”，不是写测试代码)
        ↓  输出: tasks/<任务>/xxx_test_plan.md
④ 拆垂直切片 (to-issues)
        ↓  输出: tasks/<任务>/issues/01..N.md
⑤ 逐切片实现 (每个切片内部走 TDD 红→绿)
        ↓
⑥ Review / 真机验证 / 合并
```

**最容易搞错的一点：**

| 容易误以为 | 实际是 |
|---|---|
| 先拆 Issue，再想测什么 | **先写 test_plan（验收规格）**，再拆 Issue |
| 先写完功能，再补测试 | **每个 Issue 内部**才是“先写失败测试 → 再写实现” |
| test_plan = 测试代码 | test_plan = **测什么、过线是什么**（规格文档） |

---

## 2. 目录长什么样（做完后应有的工件）

```text
skills/                              # 长期复用，不随单次任务销毁
├─ impedance-control/                # 领域知识（可换成别的 domain skill）
├─ karpathy-guidelines/              # 全程编码纪律
├─ tdd/                              # 测试方法论
└─ to-issues/                        # 拆任务方法论

tasks/<task_name>/                   # 单次任务容器
├─ <task>_grill.md                   # 设计决策记录
├─ <task>_test_plan.md               # 验收/测试规格
└─ issues/
    ├─ 01-....md
    ├─ 02-....md
    └─ ...
```

重力补偿已经按这个结构落盘：

```text
tasks/gravity_compensation/
├─ gravity_compensation_redesign_grill.md
├─ gravity_compensation_test_plan.md
└─ issues/01 ~ 07
```

---

## 3. 六个阶段：每阶段“输入 / 动作 / 输出 / 引用哪个 skill”

### Phase 1 — 读领域知识

**目的：** 别闭门造车，先对齐正确公式与约束。

| 项 | 内容 |
|---|---|
| 输入 | 用户问题（例：重力补偿要重构） |
| 动作 | 打开对应 domain skill，读公式、约束、调参顺序 |
| 输出 | 团队共识：根公式、禁止事项、关键术语 |
| 引用 | `impedance-control`（或其他领域 skill） |

重力补偿根公式（来自 domain skill）：

```text
τ_cmd = τ_ff + Kp·(q_des − q) + Kd·(dq_des − dq)
τ_ff  = scale × (τ_gravity + τ_friction)
```

规则：**先把 τ_ff 做对，再调 Kp/Kd。**

---

### Phase 2 — Grill 设计分析（先想再写）

**目的：** 把设计树走完，再动代码。

| 项 | 内容 |
|---|---|
| 输入 | 领域公式 + 现有代码现状 |
| 动作 | 展开设计树：算法 / 摩擦 / scale / 架构 / 配置；逐分支决策；列出沉默假设 |
| 输出 | `tasks/<task>/*_grill.md` |
| 引用 | `karpathy-guidelines`（Think Before Coding）+ domain skill |

Grill 文档应回答：

1. 根公式是什么（不可改）  
2. 每个分支选了什么、为什么  
3. 与现有代码差在哪里  
4. 实现优先级 Phase 1/2/3  
5. 用户已确认的决策（HITL 结论）

**本阶段不写生产代码。**

---

### Phase 3 — 测试策略（规格，不是实现）

**目的：** 先定义“做完长什么样”，再拆活。

| 项 | 内容 |
|---|---|
| 输入 | grill 决策 + domain 验收要求 |
| 动作 | 分层：数学层 / 组件层 / 集成层 / 系统层；定义 seam、样本、Pass 线 |
| 输出 | `tasks/<task>/*_test_plan.md` |
| 引用 | `tdd`（什么是好测试、期望值独立来源） |

`test_plan` 写的是：

- 测哪些用例  
- 期望值从哪来（手工计算 / 真值测量）  
- 红线/黄线/绿灯  
- 哪些是 HITL 真机测  

**本阶段一般还不写 gtest/pytest 文件**（除非你把“规格用例 ID”先列出来）。

---

### Phase 4 — 拆垂直切片（to-issues）

**目的：** 把大设计切成可独立交付、可独立验证的 Issue。

| 项 | 内容 |
|---|---|
| 输入 | grill + test_plan |
| 动作 | 按 tracer-bullet 垂直切片；标 AFK/HITL；写依赖；验收条件挂钩 test_plan |
| 输出 | `tasks/<task>/issues/01..N.md` |
| 引用 | `to-issues` |

垂直切片原则：

```text
好：一个 Issue = 一条端到端可验证路径
坏：一个 Issue = 只改 YAML / 只改 cpp / 只写文档
```

每个 Issue 模板固定三块：

```text
What to build
Acceptance criteria   ← 最好引用 test_plan 里的 TC-xxx
Blocked by
```

重力补偿示例依赖链：

```text
#01 TDD RED
  → #02 接口重构
      → #03 URDF 加载
      → #04 YAML schema
          → #05 scale
              → #06 摩擦模型
                  → #07 标定脚本
```

---

### Phase 5 — 逐切片实现（切片内 TDD）

**目的：** 一个 Issue 做完就能验证，不攒成巨型 PR。

| 项 | 内容 |
|---|---|
| 输入 | 单个 `issues/0x-*.md` |
| 动作 | 红：写失败测试 → 绿：最小实现 →（大重构放到 Review） |
| 输出 | 代码 + 通过的测试 + 该 Issue 勾选验收 |
| 引用 | `tdd` + `karpathy-guidelines`（最小改动、目标驱动） |

单个 Issue 内部循环：

```text
打开 Issue #0N
  → 写/补该切片相关测试（应失败）
  → 只写让它通过的最少代码
  → 跑该切片验收命令
  → 勾选 Acceptance criteria
  → 再开下一个 Issue
```

**注意：**  
- 全局 test_plan 在 Phase 3  
- 具体测试代码在 Phase 5 每个切片里长出来  

---

### Phase 6 — Review / 系统验证 / 合并

| 项 | 内容 |
|---|---|
| 输入 | 已完成的切片 + test_plan 系统层用例 |
| 动作 | 代码审查、仿真/真机、回归门禁 |
| 输出 | 合并、任务关闭、必要时回写 domain skill（经验沉淀） |
| 引用 | karpathy（Verify Before Completion）+ domain skill 调参/安全条款 |

系统层（真机）例子：teach mode 漂移、scale 微调、8 字运动等——这些通常是 HITL。

---

## 4. 一张表：谁在什么时候出场

| 阶段 | domain skill | karpathy | tdd | to-issues |
|---|:---:|:---:|:---:|:---:|
| 1 读知识 | ★ 主用 | 约束“别瞎猜” | — | — |
| 2 Grill | 查公式 | ★ 主用 | — | — |
| 3 test_plan | 查合格线 | 约束可验证目标 | ★ 主用（策略） | — |
| 4 拆 issues | 术语一致 | — | 验收挂钩 | ★ 主用 |
| 5 实现 | 实现细节 | ★ 最小改动 | ★ 红绿循环 | 按 Issue 执行 |
| 6 Review | 调参/安全 | ★ 验证完成 | 回归 | — |

---

## 5. 用重力补偿走一遍（最短路径）

```text
1) 打开 skills/impedance-control/SKILL.md
   → 记住：先 τ_ff，再 Kp/Kd；τ_ff = scale×(g+f)

2) 写 tasks/gravity_compensation/gravity_compensation_redesign_grill.md
   → 15 个设计分支决策完；用户确认 5 项

3) 写 tasks/gravity_compensation/gravity_compensation_test_plan.md
   → 数学层/组件层/集成层/系统层 + TC 编号

4) 跑 to-issues
   → issues/01~07 垂直切片，依赖写清

5) 从 #01 开始：
   #01 写 RED 测试
   #02 改接口让结构正确
   #03 URDF 加载
   #04 YAML schema
   #05 scale
   #06 friction
   #07 标定脚本

6) 真机 teach mode 看漂移 → 调 scale → 归档 YAML
```

当前进度对照：

| 阶段 | 状态 |
|---|---|
| Grill | 已有 |
| Test plan | 已有 |
| Issues 拆分 | 已有 01~07 |
| 切片实现 | **下一步从 #01 开始** |

---

## 6. 换一个任务时怎么复用（同一工作流）

例如下次做「手眼标定」或「双臂协同」：

```text
1. 若领域复杂 → 新建 skills/<domain>/SKILL.md（知识可沉淀）
2. 新建 tasks/<task_name>/
3. 仍按：grill → test_plan → issues → 逐切片 TDD
4. karpathy / tdd / to-issues 三个流程 skill 不用改
```

**可变的是 domain skill；不变的是后三个流程 skill。**

---

## 7. 操作口令（给人和 Agent 用）

按顺序对 Agent / 自己说：

```text
1. “按 impedance-control skill，对齐重力补偿公式与约束”
2. “按 karpathy，先 grill，不写代码”
3. “按 tdd skill，先写 test_plan，不要先写实现”
4. “按 to-issues，把 grill+test_plan 拆成 issues 独立文件”
5. “从 issues/01 开始，红→绿实现，一次只做一个 Issue”
6. “对照 test_plan 系统层做真机验收后再合并”
```

---

## 8. 反模式（看见就停）

| 反模式 | 为什么坏 |
|---|---|
| 没 grill 直接改 cpp | 决策没记录，后面难回滚 |
| 没 test_plan 就拆 Issue | 验收条件虚，切片边界乱 |
| 一个大 PR 改完再测 | 无法定位哪层坏了 |
| 水平分层（先全改 YAML 再全改代码） | 中间态不可运行、不可验证 |
| 把 domain skill 当任务目录 | 知识和当前工件混在一起，无法复用 |
| 测试绑死私有实现细节 | 一重构测试全红，失去保护价值 |

---

## 9. 最终心智模型

```text
        skills（长期）
   ┌────────────────────┐
   │ domain:  知道什么   │
   │ karpathy:怎么自律   │
   │ tdd:     怎么证明   │
   │ to-issues:怎么拆活  │
   └─────────┬──────────┘
             │ 引用
             ▼
        tasks（当次）
   ┌────────────────────┐
   │ grill   = 想清楚    │
   │ test_plan = 过线    │
   │ issues  = 可执行工单 │
   └────────────────────┘
             │
             ▼
        代码仓库 + 真机验证
```

**顺序口诀：**

> **先懂 → 先想 → 先定过线 → 再拆活 → 再红绿实现 → 再真机收工。**

不是「写完再测」，也不是「先拆再想测什么」；  
而是 **规格（test_plan）约束切片，切片内 TDD 约束实现。**
