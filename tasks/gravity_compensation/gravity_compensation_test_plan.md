# 重力补偿测试用例设计

> **依据**：[impedance-control/SKILL.md](file:///c:/code/OpenARM-main/skills/impedance-control/SKILL.md) — 阻抗控制知识库  
> **被测对象**：[NewtonEulerGravityCompensator](file:///c:/code/OpenARM-main/custoemr_arm/arm_hardware-main/include/arm_hardware/gravity_compensation.hpp#L60-L120)  
> **参考设计**：[gravity_compensation_redesign_grill.md](file:///c:/code/OpenARM-main/tasks/v2/gravity_compensation_redesign_grill.md)  
> **状态**：设计完成，待实现

---

## 一、测试策略总览

### 1.1 四层测试金字塔

```
          ┌──────────────────────┐
          │   系统层测试 (HITL)  │  ← 真机验证 + 标定验证
          └──────────────────────┘
                 ▲
          ┌──────────────────────┐
          │  集成层测试 (ROS2)   │  ← hardware_interface 集成
          └──────────────────────┘
                 ▲
          ┌──────────────────────┐
          │  组件层测试 (C++ UT)  │  ← NewtonEulerGravityCompensator
          └──────────────────────┘
                 ▲
          ┌──────────────────────┐
          │  数学层测试 (C++ UT)  │  ← 旋转矩阵 / 叉乘 / 矩阵向量乘
          └──────────────────────┘
```

| 层级 | 测试对象 | 框架 | 数量 | 执行速度 |
|------|---------|------|:----:|---------|
| 数学层 | `computeRotationMatrix()` / `crossProduct()` / `matrixVectorMultiply()` | GoogleTest | ~10 | < 1ms |
| 组件层 | `computeGravityCompensation()` / 未来的 `computeFeedforward()` | GoogleTest | ~20 | < 10ms |
| 集成层 | `ArmHardware` 中重力补偿的端到端调用 | GoogleTest + ros2_control mock | ~10 | < 100ms |
| 系统层 | 真机 teach mode 漂移测试 + 标定验证 | Python + ROS2 bag | ~5 | 分钟级 |

### 1.2 测试 Seam 定位

根据 grill 文档 [§4c 测试seam](file:///c:/code/OpenARM-main/tasks/v2/gravity_compensation_redesign_grill.md#L554-L579)：

| Seam | 测试层级 | 测试什么 |
|------|---------|---------|
| `computeRotationMatrix()` | 数学层 | 三轴旋转矩阵正确性 |
| `crossProduct()` | 数学层 | 叉乘计算 |
| `matrixVectorMultiply()` | 数学层 | 矩阵向量乘法 |
| `computeGravityCompensation()` | 组件层 | 端到端：输入角度→输出力矩 |
| `write()` 中的调用 | 集成层 | 硬件接口中重力补偿的集成 |
| Teach mode 漂移 | 系统层 | 实机重力补偿效果验证 |

---

## 二、数学层测试（Unit - Math Primitives）

**测试对象**：3 个私有辅助函数  
**方法**：将辅助函数改为 `protected` + `FRIEND_TEST`，或提取到命名空间

### 2.1 旋转矩阵测试

#### TC-MATH-001: X 轴旋转 90° — 单位向量映射

```
输入: axis=X, angle = π/2 (90°)
期望:
  [1, 0, 0] → [1, 0, 0]    (x 轴不变)
  [0, 1, 0] → [0, 0, 1]    (y → z)
  [0, 0, 1] → [0, -1, 0]   (z → -y)
验证方法: R × v = v_rotated，逐元素对比，容差 1e-9
```

#### TC-MATH-002: Y 轴旋转 90° — 单位向量映射

```
输入: axis=Y, angle = π/2
期望:
  [1, 0, 0] → [0, 0, -1]
  [0, 1, 0] → [0, 1, 0]    (y 轴不变)
  [0, 0, 1] → [1, 0, 0]
```

#### TC-MATH-003: Z 轴旋转 90° — 单位向量映射

```
输入: axis=Z, angle = π/2
期望:
  [1, 0, 0] → [0, 1, 0]
  [0, 1, 0] → [-1, 0, 0]
  [0, 0, 1] → [0, 0, 1]    (z 轴不变)
```

#### TC-MATH-004: 零角度旋转 = 单位矩阵

```
输入: axis=任意, angle = 0
期望: R = Identity matrix
验证: 对角线 = 1, 非对角线 = 0
```

#### TC-MATH-005: 旋转矩阵正交性（任意角度）

```
输入: 随机 axis + 随机 angle (100 组)
期望: Rᵀ × R = I（正交矩阵性质）
验证: R 转置 × R 与 I 的差异 < 1e-9
```

#### TC-MATH-006: 旋转矩阵行列式 = 1

```
输入: 随机 axis + 随机 angle
期望: det(R) = 1（右手系）
验证: |det(R) - 1| < 1e-9
```

#### TC-MATH-007: 固定关节 = 单位矩阵

```
输入: axis=FIXED, angle = 任意
期望: R = Identity
```

---

### 2.2 叉乘测试

#### TC-MATH-008: 正交向量叉乘

```
输入: a = [1, 0, 0], b = [0, 1, 0]
期望: a × b = [0, 0, 1]    (i × j = k)
```

#### TC-MATH-009: 平行向量叉乘 = 0

```
输入: a = [1, 2, 3], b = [2, 4, 6]
期望: a × b = [0, 0, 0]
```

#### TC-MATH-010: 自叉乘 = 0

```
输入: a = [任意], b = a
期望: a × a = [0, 0, 0]
```

#### TC-MATH-011: 叉乘反交换律

```
输入: 任意 a, b
期望: a × b = -(b × a)
```

---

### 2.3 矩阵向量乘法测试

#### TC-MATH-012: 单位矩阵不改变向量

```
输入: I × v = v
验证: 对任意 v 成立
```

#### TC-MATH-013: 零向量相乘 = 零

```
输入: M × [0, 0, 0] = [0, 0, 0]
```

---

## 三、组件层测试（Component - Gravity Compensation）

**测试对象**：`computeGravityCompensation()`  
**核心方法**：**已知位姿 → 手工计算期望值 → 对比 NE 输出**

> 来自 SKILL.md 强调：Gravity compensation is the foundation. Get τ_ff right before tuning Kp/Kd.  
> 来自 grill 文档：首要测试用例是"零位→力矩=0"和"L2水平→手工期望值"

### 3.1 基础正确性测试

#### TC-COMP-001: 全零位（竖直向上）→ 所有关节 τ_g = 0

```
输入:
  joint_angles = [0, 0, 0, 0, 0, 0, 0]
  hand_mass = 0.0
  payload = 0.0
期望:
  tau_g[0..6] 全部 ≈ 0 Nm
物理原理:
  机械臂竖直向上时，所有连杆的重力作用线都穿过关节轴
  力臂 = 0 → 力矩 = 0
容差: < 0.01 Nm（考虑浮点误差）
优先级: P0 — 最基础的正确性验证
```

#### TC-COMP-002: scale = 0 → 所有输出为 0

> 注：scale 是 redesign 后的功能，当前实现还没有。Phase 2 后测试。

```
输入: 任意角度, scale = 0
期望: tau_g 全部 = 0
验证: scale 正确传导到输出
优先级: P1
```

---

### 3.2 单关节力矩验证（核心测试）

**策略**：每次只弯曲一个关节到 90°，其他关节保持 0°，验证该关节的重力矩。

#### TC-COMP-003: L2 关节水平 90° — 最大重力矩

```
输入:
  joint_angles = [0, π/2, 0, 0, 0, 0, 0]  ← L2 弯 90°
  hand_mass = 0.0
  payload = 0.0
期望:
  tau_g[1] (L2) ≈ m_L2 * g * com_L2_x
                   + (m_L3 + m_L4 + ... + m_L7) * g * L_total_horizontal
  tau_g[其他] ≈ 0
计算方法:
  1. 从硬编码参数中读取各连杆质量和质心
  2. L2 水平时，L2 质心在 X 方向，L3~L7 整体在 L2 末端前方
  3. 手工计算 L2 需要承受的总重力矩
容差: < 5%（考虑力臂简化的误差）
优先级: P0 — 验证重力矩量级是否正确
```

#### TC-COMP-004: L2 关节 -90° — 力矩反向

```
输入:
  joint_angles = [0, -π/2, 0, 0, 0, 0, 0]
期望:
  tau_g[1] ≈ -TC-COMP-003 的 tau_g[1]
验证: 对称性 — 角度反向则力矩反向
优先级: P1
```

#### TC-COMP-005: L4（肘）关节 90°

```
输入:
  joint_angles = [0, 0, 0, π/2, 0, 0, 0]
期望:
  tau_g[3] > 0（肘部需要托住前臂和手）
  量级 < L2 的最大力矩（前臂质量 < 整条臂）
优先级: P1
```

---

### 3.3 末端负载测试

#### TC-COMP-006: 增加 payload → 力矩线性增大

```
输入:
  L2 水平 90° 姿态
  payload = 0.0 kg → 1.0 kg → 2.0 kg
期望:
  tau_g 的增量 = Δpayload * g * r_horizontal
  即：每增加 1kg，L2 力矩增加 ~9.8 * 末端水平力臂 Nm
验证: 线性相关系数 > 0.999
优先级: P0 — 验证 payload 参数正确传导
```

#### TC-COMP-007: hand_mass = payload 等效性

```
输入:
  姿态 1: hand_mass=0.5, payload=0.5
  姿态 2: hand_mass=1.0, payload=0.0
期望:
  tau_g 完全相同（因为两者相加 = total_end_mass）
优先级: P1
```

---

### 3.4 能量守恒验证（高级测试）

#### TC-COMP-008: 重力场保守性 — 环形路径做功为 0

```
原理: 重力是保守力，沿闭合路径运动一周，重力做功为 0。
方法:
  1. 选择一条关节空间的闭合路径（如 L2: 0→π/2→π→π/2→0）
  2. 沿路径采样 N 个点，计算每个点的 tau_g
  3. 数值积分：W = ∫ τ_g · dq（沿路径）
  4. 返回起点，总功应为 0
期望: |W| < 0.01 J（考虑数值积分误差）
优先级: P2 — 物理一致性的深层验证
```

---

### 3.5 梯度/雅可比验证（高级测试）

#### TC-COMP-009: 力矩 = 势能梯度

```
原理: τ_gravity = -∂U/∂q（重力矩 = 重力势能对关节角的负梯度）
方法:
  1. 选择一个随机姿态 q0
  2. 用 NE 计算 tau_g(q0)
  3. 用有限差分近似梯度: ∂U/∂q ≈ [U(q0+ε) - U(q0-ε)] / (2ε)
     U(q) = Σ m_i * g * z_com_i(q) （总重力势能）
  4. 验证 tau_g = -∂U/∂q
期望: 每个关节误差 < 1%
优先级: P2 — 验证 NE 算法的物理一致性
```

---

### 3.6 摩擦模型测试（Phase 2）

>  redesign 后新增，当前实现无摩擦

#### TC-COMP-010: 零速度 → 摩擦力 = 0

```
输入: dq = 0
期望: tau_friction = 0
验证: Fo 是静态偏移，不算在速度为 0 的摩擦里（或需要明确）
优先级: P1
```

#### TC-COMP-011: 正速度 → 摩擦力为正（阻力方向）

```
输入: dq = 1.0 rad/s
期望: tau_friction > 0（阻碍运动）
优先级: P1
```

#### TC-COMP-012: 负速度 → 摩擦力为负

```
输入: dq = -1.0 rad/s
期望: tau_friction < 0
优先级: P1
```

#### TC-COMP-013: 摩擦对称性

```
输入: +dq 和 -dq 的绝对值相同
期望: |tau_friction(+dq)| ≈ |tau_friction(-dq)|（不含 Fo 时严格相等）
优先级: P1
```

#### TC-COMP-014: 粘性摩擦线性度

```
输入: dq = 1, 2, 3, 4, 5 rad/s（高速区，库仑项饱和）
期望: tau_friction 与 dq 线性相关（Fv 主导）
验证: 线性相关系数 > 0.99
优先级: P2
```

---

### 3.7 性能基准测试

#### TC-PERF-001: 单次调用耗时

```
方法:
  1. 调用 computeGravityCompensation 10000 次
  2. 测量总时间 / 10000 = 平均单次耗时
期望:
  平均 < 0.1 ms
  最差 < 0.5 ms（无堆分配、无系统调用）
优先级: P1 — 实时性保证
```

#### TC-PERF-002: 无堆分配验证

```
方法:
  1. 重载 operator new / operator delete
  2. 调用 computeGravityCompensation
  3. 检查是否触发了堆分配
期望: 0 次堆分配（全部在栈上完成）
优先级: P1 — 实时安全保证
```

---

## 四、集成层测试（Integration - Hardware Interface）

**测试对象**：`ArmHardware` 类中重力补偿的集成调用  
**方法**：mock 硬件通信，验证 write() 中重力补偿的正确调用

### 4.1 集成正确性

#### TC-INT-001: write() 中重力补偿被调用

```
方法:
  1. 创建 ArmHardware 实例
  2. mock CAN 通信
  3. 调用 read() → write() 循环一次
  4. 验证重力补偿结果被加到了电机命令中
期望: 电机命令 ≠ 0（因为有重力）
优先级: P1
```

#### TC-INT-002: gc_enabled = false → 无重力补偿

```
方法:
  1. 设置 gravity_compensation.yaml 中 gc_enabled = false
  2. 调用 write()
  3. 验证电机命令不包含重力补偿分量
期望: 电机命令 = 纯阻抗控制项
优先级: P1
```

---

### 4.2 参数加载测试

#### TC-INT-003: YAML 参数正确加载

```
方法:
  1. 用测试专用 YAML 配置初始化 ArmHardware
  2. 验证 tau_ff_scale、friction 参数等被正确读取
期望: 读取值与 YAML 文件一致
优先级: P1
```

#### TC-INT-004: 缺失必填字段 → 启动失败

> redesign 后启用，严格模式

```
方法:
  1. 故意构造一个缺少 scale 字段的 YAML
  2. 尝试初始化 ArmHardware
期望: 抛出异常或返回错误（不静默降级）
优先级: P2
```

---

## 五、系统层测试（System - HITL / Real Robot）

**测试对象**：真机上的重力补偿效果  
**方法**：teach mode + 数据录制 + 漂移分析

> 来自 [tuning-guide.md §1](file:///c:/code/OpenARM-main/skills/impedance-control/references/tuning-guide.md#L1-L27)：
> 调参顺序第一条就是"先搞定重力补偿"。

### 5.1 Teach Mode 漂移测试

#### TC-SYS-001: 全伸展姿态 — 最大漂移测试

```
配置:
  Teach mode 开启 (Kp = kp_min, Kd = 正常)
  重力补偿启用
步骤:
  1. 将机械臂手动拉到最大伸展姿态（L2 水平）
  2. 保持静止 10 秒
  3. 松开，开始计时
  4. 记录 30 秒内各关节角度变化
期望:
  L2 关节漂移 < 5° / 30s
  （标定良好的情况下应 < 2° / 30s）
判定:
  向下漂 → 补偿不足 → 增大 tau_ff_scale
  向上漂 → 补偿过度 → 减小 tau_ff_scale
优先级: P0 — 重力补偿质量的金标准
```

#### TC-SYS-002: 全零位（竖直）— 最小漂移

```
步骤:
  1. 机械臂竖直向上（理论零力矩姿态）
  2. 松开，记录 30s 漂移
期望:
  漂移 < 2° / 30s
  （理论上应该接近 0，但实际有摩擦/线缆等扰动）
优先级: P1
```

#### TC-SYS-003: 多个姿态一致性

```
步骤:
  1. 对 5 个典型姿态（全竖直、半弯、全伸展、L3弯、手腕弯）
  2. 分别测量 30s 漂移
期望:
  所有姿态漂移 < 5° / 30s
  最大漂移姿态与最小漂移姿态的差异 < 3x
优先级: P1 — 验证整个工作空间内补偿质量均匀
```

---

### 5.2 标定验证测试

#### TC-SYS-004: 标定脚本正确性

```
方法:
  1. 运行 friction calibration 脚本（恒速扫描）
  2. 输出 Fc, Fv, Fo 参数
  3. 用这些参数计算摩擦力矩
  4. 与恒速运动中实际测得的 τ_motor - τ_gravity 对比
期望:
  拟合残差 < 10%
优先级: P2
```

#### TC-SYS-005: scale 校准收敛性

```
方法:
  1. 初始 scale = 1.0
  2. 运行 drift measurement → adjust scale → 再测量
  3. 重复 3 次
期望:
  每次迭代漂移量减小
  第 3 次漂移 < 第 1 次的 50%
优先级: P1
```

---

### 5.3 安全测试

#### TC-SYS-006: 重力补偿关闭 → 臂掉落（安全警告）

```
警告: 必须有人扶住机械臂！
步骤:
  1.  stiff mode，臂在水平位置
  2.  突然关闭重力补偿
期望:
  臂会下坠（因为 Kp 扛不住）
  → 验证安全系统正常：急停可用、有人监护
优先级: P0 — 不是测试功能，是测试安全流程
```

---

## 六、测试数据与期望值计算指南

### 6.1 手工计算期望值的方法

对于单关节测试（如 TC-COMP-003），期望值计算步骤：

```
1. 从 initializeLinkParameters() 读取参数:
   - link_inertias_[i].mass  (各连杆质量)
   - link_inertias_[i].com_x  (各连杆质心 x 坐标)
   - link_transforms_[i].x   (各关节间的平移)

2. 对 L2 水平 90° 的姿态:
   - L2 质心水平距离 = link_transforms_[1].x + com_L2_x
   - L3 质心水平距离 = L2 末端 + link_transforms_[2].x + com_L3_x
   - ... 依此类推到 L7

3. 总力矩 = Σ (m_i * g * r_i_horizontal)

4. 注意: 这是近似值，因为 NE 考虑了完整的 3D 位姿
   容差设为 ±5% 是合理的
```

### 6.2 用 KDL 作为基准（可选，但强烈推荐）

如果项目引入了 KDL，最可靠的验证方法是：

```
1. 用相同的 URDF 参数创建 KDL Chain
2. 对随机 100 个姿态，分别用 KDL 和自实现 NE 计算
3. 对比结果
期望: 每个关节误差 < 2%
如果误差 > 5%，说明自实现 NE 有 bug
```

> 这是 grill 文档中"子问题 1c"建议的量化验证方法。

### 6.3 容差设定原则

| 测试类型 | 容差 | 理由 |
|---------|------|------|
| 数学层（矩阵/叉乘） | 1e-9 | 纯浮点运算，精度高 |
| 组件层（零位测试） | 0.01 Nm | 理论值应为 0 |
| 组件层（单关节 90°） | 5% | 力臂简化 + 手工估算误差 |
| 组件层（payload 线性） | 1% | 线性关系应该很准 |
| 系统层（漂移） | ~5°/30s | 实际机械有摩擦/线缆扰动 |

---

## 七、测试优先级与实施计划

### 7.1 优先级总表

| 优先级 | 测试用例 | 数量 | 预估工作量 |
|:------:|---------|:----:|-----------|
| **P0** | TC-COMP-001（零位=0） | 1 | 0.5 天 |
| **P0** | TC-COMP-003（L2水平90°） | 1 | 1 天 |
| **P0** | TC-COMP-006（payload线性） | 1 | 0.5 天 |
| **P0** | TC-SYS-001（全伸展漂移） | 1 | 0.5 天 + 真机 |
| **P1** | 数学层全部 (TC-MATH-001~013) | 13 | 1 天 |
| **P1** | TC-COMP-002/004/005/007 | 4 | 1 天 |
| **P1** | TC-PERF-001/002 | 2 | 0.5 天 |
| **P1** | TC-INT-001/002/003 | 3 | 1 天 |
| **P1** | TC-SYS-002/003/005 | 3 | 1 天 + 真机 |
| **P2** | TC-COMP-008/009（能量/梯度） | 2 | 1 天 |
| **P2** | 摩擦测试 (Phase 2) | 5 | 1 天 |
| **P2** | TC-INT-004 | 1 | 0.5 天 |
| **P2** | TC-SYS-004/006 | 2 | 1 天 + 真机 |

### 7.2 实施路线（对齐 grill 文档的 Phase）

```
Phase 0（当前）: 先写 P0 测试，验证当前实现
  ├─ TC-COMP-001: 零位测试 → 验证算法基本正确
  ├─ TC-COMP-003: L2水平 → 验证力矩量级正确
  └─ TC-COMP-006: payload线性 → 验证参数传导

Phase 1: URDF 加载 + 接口重构后
  ├─ 补充数学层测试
  ├─ 补充组件层 P1 测试
  └─ 性能基准测试

Phase 2: 摩擦 + scale 实现后
  ├─ 摩擦模型测试
  ├─ scale 校准测试
  └─ 集成层测试

Phase 3: 标定脚本完成后
  └─ 系统层测试 + 实机验证
```

---

## 八、与 TDD 流程的对应

根据 [tdd/SKILL.md](file:///c:/code/OpenARM-main/skills/tdd/SKILL.md) 的 RED→GREEN 循环：

### RED 阶段（先写失败的测试）

1. 写 TC-COMP-001（零位 = 0）
   - 如果当前实现通过 → 说明基础没问题
   - 如果失败 → 找到了 bug，先修这个

2. 写 TC-COMP-003（L2 水平 90°）
   - 手工计算期望值
   - 对比实际输出，看差异是否在 5% 以内

### GREEN 阶段

1. 修复 RED 阶段发现的 bug
2. 确保所有 P0 测试通过

### REFACTOR 阶段

1. 引入 URDF 加载
2. 接口改为 `computeFeedforward()`
3. 所有测试仍然通过 → 重构成功

---

## 九、测试文件结构建议

```
arm_hardware-main/
├── include/
│   └── arm_hardware/
│       └── gravity_compensation.hpp
├── src/
│   └── gravity_compensation.cpp
└── test/
    ├── CMakeLists.txt
    ├── math_primitives_test.cpp       ← 数学层 (TC-MATH-*)
    ├── gravity_compensation_test.cpp  ← 组件层 (TC-COMP-*)
    ├── performance_benchmark.cpp      ← 性能测试 (TC-PERF-*)
    └── test_data/                     ← 测试用数据
        ├── test_arm.urdf              ← 简化的测试 URDF
        └── expected_torques.csv       ← 预计算的期望值（可选）
```

---

## 十、关键结论

1. **P0 测试只有 4 个，但足够验证核心正确性**：
   - 零位 = 0 → 算法基本正确
   - L2 水平量级正确 → 物理量级合理
   - payload 线性 → 参数正确传导
   - 实机漂移可接受 → 端到端效果达标

2. **KDL 基准验证是"核武器"级别的验证**：
   如果能引入 KDL，100 个随机姿态的对比测试比所有手工计算的测试加起来都有说服力。强烈建议 Phase 1 完成后加这个。

3. **系统层测试是"最终真相"**：
   单元测试全过不代表实机效果好。Teach mode 漂移测试是重力补偿质量的金标准。

4. **测试是 redesign 的安全网**：
   按照 grill 文档的重构计划（URDF 加载、接口改名、摩擦新增），每一步都需要测试来确保"没改坏"。
