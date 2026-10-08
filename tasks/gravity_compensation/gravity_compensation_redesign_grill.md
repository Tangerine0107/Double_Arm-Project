# Grill-Me: 全新重力补偿功能设计分析

> **依据规则**：[grill-me/SKILL.md](file:///c:/code/OpenARM-main/skills/grill-me/SKILL.md) — 设计树拷问  
> **规格来源**：[impedance-control/SKILL.md](file:///c:/code/OpenARM-main/skills/impedance-control/SKILL.md) — 标准公式  
> **约束**：仅分析设计，不写代码  
> **分析日期**：2026-07-27  
> **状态**：✅ 设计树遍历完成，15/15 分支已决策，5/5 待确认项已通过用户确认

---

## 设计树根节点

SKILL.md §2 (L34) 定义了重力补偿的完整公式：

```
τ_ff = tau_ff_scale[j] × (τ_gravity[j] + τ_friction[j])
```

这是不可改变的根节点。所有下游决策都从这个公式展开。当前实现仅完成了 `τ_gravity`，缺失 `τ_friction` 和 `tau_ff_scale`。

---

## 公式原理详解

### 在控制回路中的位置

```
                              ┌──────────────┐
                   τ_ff       │  重力+摩擦    │
              ┌──────────────→│  前馈计算器    │
              │               └──────────────┘
              │                       ↑
              │               joint_angles, velocities
              │
┌─────────┐   │   ┌─────────┐   ┌─────────┐
│ 控制器   │──→│ + │ Σ  │──→│ 电机    │──→ 机械臂
│ Kp·e+   │   └─────────┘   └─────────┘     │
│ Kd·ė    │                                 │
└─────────┘                          ┌─────────┐
      ↑                              │ 编码器   │←── q, dq
      │                              └─────────┘
 q_des, dq_des
```

τ_ff 是**前馈项**（feedforward），在反馈回路之外直接注入电机命令。其物理意义：**如果重力补偿完美，电机不需要出力来对抗重力，位置环只需处理跟踪误差。**

完整的控制命令为：

```
τ_cmd = τ_ff + Kp·(q_des − q) + Kd·(dq_des − dq)
```

用弹簧-阻尼类比：

| 组件 | 类比 | 作用 |
|------|------|------|
| `τ_ff` | 抵消重力的**气垫/浮力** | 让机械臂"失重"——电机关闭时臂不掉落 |
| `Kp·(q − q_des)` | **弹簧** | 偏离目标位置时拉回去 |
| `Kd·(dq − dq_des)` | **减震器** | 抑制振荡，防止过冲 |

**关键分工**：τ_ff 负责"静力学"（抵消恒定的重力和摩擦），Kp/Kd 负责"动力学"（处理跟踪误差和扰动）。如果 τ_ff 不准，Kp 必须扛起重力的差额 → 需要更大的 Kp → 刚度更高 → 碰撞力更大 → 不安全。这就是 SKILL.md 反复强调的原因：**"Gravity compensation is the foundation. Get τ_ff right before tuning Kp/Kd."**

---

### τ_gravity — 重力力矩

```
τ_gravity = 重力对关节 j 产生的力矩
```

**物理本质**：串联机械臂中，每个连杆和末端负载的质量在重力场中产生的力矩通过连杆链传递到每个关节。例如 L2 关节（肩部俯仰）需要对抗的不只是 L2 连杆自身的重力，还有 L3~L7 和灵巧手一整条"尾巴"的累积重力。

**计算方式**（牛顿-欧拉递推，对应 [gravity_compensation.cpp#L126-L238](file:///c:/code/OpenARM-main/custoemr_arm/arm_hardware-main/src/gravity_compensation.cpp#L126-L238)）：

```
前向递推：从基座 → 末端
  ① 旋转矩阵累计: R[i+1] = R[i] × R_joint(θ_i)
  ② 坐标系原点累积: p[i+1] = p[i] + R[i] × p_link[i]
  → 得到每个连杆在世界坐标系中的位姿 (R, p)

后向递推：从末端 → 基座
  ① 初始化末端力: F = (hand_mass + payload) × g（向下）
  ② 对第 i 关节（从 L7 逆推到 L1）：
     a. 计算连杆质心在世界坐标系的位置: com_global
     b. 累计重力: F_total = F + link_mass[i] × g
     c. 计算力臂: r = com_global − p[i]
     d. 力矩: τ = r × F_total
     e. 轴向投影: τ_gravity[i] = N · (R[i] × joint_axis[i])
     f. 向上传递: F = F_total
```

**关键特性**：τ_gravity 只依赖**位姿**（q），不依赖速度和加速度。这是"静态"重力补偿——只要机械臂不动，重力力矩就不变。

---

### τ_friction — 摩擦力矩

```
τ_friction = Fc·tanh(0.1·k·dq) + Fv·dq + Fo
```

三个物理分量：

| 分量 | 公式 | 物理来源 | 速度关系 |
|------|------|---------|---------|
| **库仑摩擦** | `Fc·tanh(0.1·k·dq)` | 关节轴承、齿轮啮合的静/动摩擦力 | 符号函数（运动方向决定） |
| **粘性摩擦** | `Fv·dq` | 润滑剂、密封件的粘滞阻力 | 与速度成正比 |
| **静态偏移** | `Fo` | 重力引起的轴承不对称负载 | 与速度无关（恒定偏移） |

#### 为什么用 `tanh` 而不是 `sign`？

纯库仑摩擦是阶跃函数 `τ = Fc × sign(dq)`，在零速处不连续——这是控制系统的噩梦。`tanh(0.1·k·dq)` 在零速附近光滑过渡、处处可微，避免了 sign 函数引起的控制抖动。`k` 参数控制过渡区宽度，k 越大 tanh 越接近 sign。

```
            τ
         Fc ┤        ╱
            │      ╱
            │    ╱          ← tanh: 光滑、可微
         0  ┤──╱──── dq
            │ ╱
            │╱
        -Fc ┤
```

#### 为什么需要摩擦补偿？

无摩擦补偿的 teach mode：人推动机械臂 → 库仑摩擦抵抗运动 → 感到"卡顿" → 示教轨迹不自然 → VLA 数据质量差。有摩擦补偿后：电机输出抵消力矩 → 关节平滑运动 → 人只感受到微小惯性力。

此外，外力估计（τ_ext = τ_motor − τ_ff）的精度直接取决于摩擦模型的准确性——未建模的摩擦力全部被错误地解释为"外力"。

---

### tau_ff_scale — 校准系数

```
最终力矩 = scale × (τ_gravity + τ_friction)
```

| scale | 效果 |
|-------|------|
| 1.00 | 信任模型，不做缩放 |
| 1.03 | 补偿偏弱（臂向下漂），放大 3% |
| 0.97 | 补偿偏强（臂向上漂），缩小 3% |

#### 为什么重力补偿会有误差？

即使牛顿-欧拉计算完美，实际机械臂的重力力矩仍可能与模型不同：

1. **制造公差** — URDF 中连杆质量是设计值，实际加工后 ±5% 是正常的
2. **线缆/管路** — URDF 不包含线缆的弹力，但线缆确实对关节施加了力矩
3. **装配误差** — 关节零点偏差导致 `joint_angles` 输入与实际姿态不一致
4. **末端工具** — 灵巧手换型号后质量和质心变了，但 URDF 没更新

`tau_ff_scale` 是所有这些未建模因素的**一次性综合修正**——在 teach mode 中观察漂移方向，用 ±0.02~0.05 的步进微调，不需要重新标定每个物理参数。

---

### 极端情况分析

| 场景 | τ_ff | Kp | Kd | 行为 |
|------|------|-----|-----|------|
| 正常工作 | 完美补偿 | 60-80 | 30-50 | 臂定在目标位，轻微外力推不动 |
| Teach mode | 完美补偿 | ~2-5 | ~1-3 | 臂漂浮，人可随意拖动 |
| τ_ff=0（无补偿） | 0 | 60 | 30 | 关节因重力下坠，Kp 拼命拉回 → 稳态误差大 |
| τ_ff 过度（scale>1.2） | 偏大 | 2 | 1 | Teach mode 中臂向上漂 |
| 有 τ_f 无 τ_g | 仅摩擦 | 60 | 30 | 静止时臂下坠（重力未抵消） |
| 有 τ_g 无 τ_f | 仅重力 | 60 | 30 | 低速运动卡顿，零速附近有问题 |

---

## 设计树展开

```
根节点: τ_ff = scale × (gravity + friction)
│
├─ [分支1] τ_gravity 用什么算法？
│   ├─ 选项A: KDL (外部库)
│   ├─ 选项B: 自实现牛顿-欧拉 (当前方案)
│   └─ 选项C: 简化正弦模型
│       ↓ 选B后展开的子问题:
│       ├─ [1a] 参数来源: 硬编码 vs URDF动态加载?
│       ├─ [1b] 惯性张量是否需要?
│       └─ [1c] 后向递推的完整度: 静态只重力 vs 完整NE?
│
├─ [分支2] τ_friction 用什么模型?
│   ├─ 选项A: SKILL.md标准模型 (Fc·tanh + Fv·dq + Fo)
│   ├─ 选项B: 简化模型 (仅粘性摩擦 Fv·dq)
│   └─ 选项C: 暂不实现 (当前方案)
│       ↓ 选A后展开的子问题:
│       ├─ [2a] 摩擦参数如何标定?
│       ├─ [2b] 摩擦补偿在哪个位置计算?
│       └─ [2c] 是否需要运行时自适应?
│
├─ [分支3] tau_ff_scale 校准机制怎么设计?
│   ├─ 选项A: YAML静态配置
│   ├─ 选项B: 运行时ROS2参数动态调整
│   └─ 选项C: 自动标定脚本
│       ↓ 选A+B后展开的子问题:
│       ├─ [3a] 默认值是什么?
│       └─ [3b] 校准工作流是什么?
│
├─ [分支4] 集成架构: 放在哪里?
│   ├─ 选项A: arm_hardware 内部 (当前方案)
│   ├─ 选项B: 独立ROS2控制器节点
│   └─ 选项C: 可复用的库 + controller调用
│       ↓ 展开的子问题:
│       ├─ [4a] 接口签名设计
│       ├─ [4b] 实时性保证
│       └─ [4c] 测试seam在哪?
│
└─ [分支5] 配置管理: YAML vs URDF vs 两者结合?
    ├─ 选项A: 全部从YAML加载
    ├─ 选项B: 物理参数从URDF, 调优参数从YAML
    └─ 选项C: 全部运行时ROS2参数
```

---

## 逐分支拷问

### 分支1: τ_gravity 算法选择

**Q1**: τ_gravity 用什么算法？

推荐答案：**选项B — 保留自实现牛顿-欧拉，但重构为URDF动态加载参数。**

<details>
<summary>展开推理</summary>

| 选项 | 优势 | 劣势 | SKILL.md态度 |
|------|------|------|-------------|
| A: KDL | 成熟库、经过验证 | 引入重量级依赖、学习曲线 | "首选" (§2 L39) |
| B: 自实现NE | 零外部依赖、可控、轻量 | 维护负担、需自己保证正确性 | "允许" (§2 L41) |
| C: 简化正弦 | 极简、极快 | 物理模型错误、精度差 | "最不推荐" |

**选择B的理由**：
- 当前代码已有自实现NE，不需要从头写
- 机器人嵌入式/边缘部署场景中减少依赖是合理的工程决策
- SKILL.md明确允许："If you don't have KDL available, you can implement your own recursive Newton-Euler"
- 代价是需要确保实现的数学正确性

**但需要解决的子问题**：
- 当前参数硬编码 → 必须改为URDF动态加载
- 当前仅静态重力 → 确认这个简化对使用场景是否足够
</details>

---

#### 子问题 1a: 参数来源——硬编码 vs URDF动态加载?

**Q1a**: 连杆质量、质心、变换参数应该从哪里来？

推荐答案：**URDF动态加载，并缓存到类成员中，构造时加载一次。**

| 方案 | 优势 | 劣势 |
|------|------|------|
| 硬编码当前方案 | 编译期确定、零运行时开销 | URDF修改需重新编译；44行裸数字难维护 |
| URDF每次调用解析 | 始终与URDF同步 | 实时循环中解析XML不可接受 |
| URDF启动时加载+缓存 | 与URDF同步、实时安全 | 增加依赖(urdfdom/tinyxml) |

**推荐方案细节**：
- 在 `NewtonEulerGravityCompensator` 构造函数中接受 URDF 路径或 `urdf::Model` 引用
- 遍历 7 个关节的 `<link>/<inertial>/<mass>` 和 `<origin>` 提取参数
- 缓存到现有的 `link_inertias_[]` 和 `link_transforms_[]` 数组中
- `initializeLinkParameters()` 从硬编码变为 URDF 解析
- 向后兼容：提供静态 `createFromHardcoded()` 工厂方法用于测试

**上游依赖**：分支4（架构位置）决定——如果在 arm_hardware 内部，URDF 可以从 `HardwareInfo` 获取；如果独立，需要额外传入。

---

#### 子问题 1b: 惯性张量是否需要?

**Q1b**: `LinkInertia` 中已存有 `ixx/ixy/.../izz` 但从未使用。新设计要保留吗？

推荐答案：**保留结构但标记为"预留"，不参与当前计算。**

**理由**：
- 纯重力补偿不需要惯性张量（无角加速度，τ = r × mg，与转动惯量无关）
- 如果未来扩展到完整逆动力学（含 Coriolis 项），这些字段会用到
- 从 URDF 加载时可以顺带提取，零额外成本
- 需要明确注释："Static gravity only — inertia tensor reserved for future full-body dynamics"

**反方意见**（值得考虑）：
- 过度设计（Fowler 坏味 "Speculative Generality"）
- 增加结构体大小（7×6×8 bytes ≈ 336 bytes on stack）
- 可以考虑用条件编译或子类隔离

**最终推荐**：保留。336 bytes 的内存代价可忽略，而完整逆动力学的扩展路径明确（VLA需要高速运动）。

---

#### 子问题 1c: 后向递推的完整度

**Q1c**: 当前后向递推中"力臂简化"（代码注释明确承认）是否需要修正？

推荐答案：**保留当前简化，但文档化其误差范围。**

当前实现：
```
后向递推中: r = com_global[i] - p[i]    ← 只算本级质心
```

完整NE应该是：
```
后向递推中: r = com_global[i] - p[i]    ← 但对 F_total 需要算合成质心
```

**选择保留的理由**：
- 对于纯静态重力，F_total 只是向下的力向量，力臂不依赖于力的分布
- 只要力的方向恒定（向下），合力力矩 = Σ(r_i × m_i·g) = (Σ m_i·r_i) × g
- 但当前代码并未计算 Σ m_i·r_i（合成质心），只是用本级质心——这确实是误差源

**需要验证**：用KDL或理论计算量化这个误差。如果误差 <5%，接受；否则修正。

---

### 分支2: τ_friction 摩擦模型

**Q2**: 摩擦补偿用什么模型？

推荐答案：**先实现选项A的完整标准模型，但参数全部默认0（效果等同关闭）。**

SKILL.md §2 (L46-54) 规定的标准模型：
```
τ_friction = Fc·tanh(0.1·k·dq) + Fv·dq + Fo
```

| 参数 | 含义 | 默认值 | 标定方法 |
|------|------|--------|---------|
| Fc | 库仑摩擦 (Nm) | 0.0 | 恒速扫描实验 |
| k | 库仑锐度 | 100.0 | 经验值 |
| Fv | 粘性摩擦系数 (Nm·s/rad) | 0.0 | 多速测量 |
| Fo | 静态偏移 (Nm) | 0.0 | 静止力矩测量 |

**选择理由**：
1. SKILL.md明确要求，不是可选项
2. 没有摩擦补偿 → teach mode无法漂浮（静摩擦卡住）→ 整个力控体系的基础不牢
3. 外力估计（τ_ext = τ_motor - τ_ff）的精度直接取决于摩擦模型的准确性
4. 默认值全0意味现有行为不变，通过YAML逐步启用

---

#### 子问题 2a: 摩擦参数如何标定?

**Q2a**: 4个摩擦参数的实际值怎么获得？

推荐答案：**提供一个离线标定脚本 + YAML手动配置两个入口。**

**标定脚本设计**（不写代码，只设计）：

```
输入：关节索引、速度范围
输出：Fc, Fv, Fo估计值（写入YAML或stdout）

标定流程：
1. 将机械臂置于安全位姿
2. 依次对每个关节执行恒速扫描:
   - 驱动关节以恒定速度(-max_vel ... +max_vel)运动
   - 记录 τ_motor - τ_gravity = τ_friction
3. 对速度-力矩数据做最小二乘拟合:
   - Fo = 零速截距
   - Fv = 线性段斜率
   - Fc = 低速段非线性幅度
4. 输出标定结果
```

**注意事项**：
- 标定需要在无负载状态下进行（或已知负载）
- 重力补偿必须在标定时启用（τ_gravity 已校准）
- 多关节联动时摩擦力可能不同（交叉耦合），标定脚本需要支持单关节模式

---

#### 子问题 2b: 摩擦补偿在哪个位置计算?

**Q2b**: `compute_friction_compensation()` 放在哪一层？

推荐答案：**在 NewtonEulerGravityCompensator 内部，与 gravity 统一返回。**

**备选方案对比**：

| 方案 | 调用方式 | 优势 | 劣势 |
|------|---------|------|------|
| A: NE类内部 | `compensator.compute(joint_angles, velocities)` | 封装完整、单一接口 | NE类需要velocity参数 |
| B: arm_hardware内 | `gravity() + friction()` 分开调 | 不改NE接口 | 耦合在hardware层 |
| C: 独立FrictionModel类 | 独立的compute方法 | 可复用、可测试 | 多一层抽象 |

**选A的理由**：
- 对应SKILL.md的模块架构：Gravity model + Friction model → 统一 `τ_ff`
- 方法签名自然扩展：`computeGravityCompensation(angles, velocities)` 而非当前 `(angles, hand_mass, payload)`
- 保持单一真相来源——调用方不需要知道内部如何组合

**需要变更的接口**：
```cpp
// 当前
computeGravityCompensation(angles, hand_mass, payload)
// ↓ 变为
computeFeedforwardTorque(angles, velocities, hand_mass, payload)
// 返回 τ_ff = scale × (τ_gravity + τ_friction)
```

这就是 `computeGravityCompensation` 改名为 `computeFeedforwardTorque` 的契机——
SKILL.md的术语是 τ_ff（feedforward），不单是gravity。

---

#### 子问题 2c: 是否需要运行时自适应?

**Q2c**: 摩擦参数会随时间变化（磨损、温度），是否需要运行时自适应？

推荐答案：**当前版本不需要。YAML配置 + 定期重新标定即可。**

**理由**：
- 摩擦参数的变化是月/年级的（磨损），不需要毫秒级自适应
- 自适应需要持续的激励信号，影响正常操作
- 复杂度过高但收益有限

**如果未来需要**（Phase 2预留）：
- 在 teach mode 中记录低速运动的力矩残差
- 用滑动窗口平均更新 Fv
- 接口设计时预留 `updateFrictionEstimate(joint, Fc, Fv, Fo)` 方法

---

### 分支3: tau_ff_scale 校准机制

**Q3**: 校准系数怎么承载？

推荐答案：**YAML静态配置为主 + ROS2参数动态调整为辅。**

| 承载方式 | 默认值 | 范围 | 用途 |
|---------|--------|------|------|
| `gravity_compensation.yaml` 中的 `scale` 字段 | 1.0 | 0.85-1.05 | 持久化校准结果 |
| ROS2 parameter `gc_scale_L2` ~ `gc_scale_L7` | 1.0 (从YAML加载) | 0.85-1.05 | 运行时微调/调试 |

**设计意图**：
- YAML是持久化存储（校准结果保存）
- ROS2参数是临时覆盖（调试、在线微调）
- 启动时：YAML → ROS2参数默认值；运行时可通过 `ros2 param set` 临时调整
- 确认有效的调整后，手动写回YAML

---

#### 子问题 3a: 默认值为什么是1.0?

**Q3a**: scale 默认值为 1.0 吗？

推荐答案：**是。scale=1.0 表示"信任 τ_gravity 计算结果，不做额外缩放"。**

这符合 SKILL.md 的调优流程（§6, L186-189）：
1. 先用 scale=1.0 启用重力补偿
2. 观察 teach mode 漂移
3. 按需微调到 0.85-1.05 范围

**注意**：scale 不等于"重力补偿强度"。scale=1.0时如果臂漂移，说明 τ_gravity 或 τ_friction 模型有误，而不是 scale 的问题。scale 是最后的精细修整。

---

#### 子问题 3b: 校准工作流

**Q3b**: 用户怎么执行 scale 校准？

推荐答案：

```
1. 启动 teach mode (Kp=Kp_min)
2. 对每个关节:
   a. 手动将关节移到不同角度
   b. 松开，观察漂移方向
   c. 向下漂→增大scale；向上漂→减小scale
   d. 通过 ros2 param set /arm_hardware/gc_scale_L3 0.98 实时调整
3. 确认所有关节无漂移后，将最终值写入 gravity_compensation.yaml
4. 重启验证 YAML 值生效
```

**不需要专门的标定脚本**——人工观察 + ros2 param 足够。这本质上是"手感调参"，不是数学优化。

---

### 分支4: 集成架构

**Q4**: 重力补偿放在哪里？

推荐答案：**保留在 arm_hardware 内部，但重构 NewtonEulerGravityCompensator 为可独立测试的组件。**

| 方案 | 适用场景 | 不适用场景 |
|------|---------|-----------|
| A: arm_hardware内部(当前) | 重力补偿作为力矩前馈，紧耦合于write循环 | 需要独立控制器的场景 |
| B: 独立controller | 力控策略需要多个控制源 | 增加一个节点，延迟增加 |
| C: 共享库 | 多模块复用 | 过度设计 |

**选A的理由**：
- 重力补偿本质上是前馈（feedforward）——基于当前位姿计算理想力矩，不依赖控制策略
- 放在hardware层意味着所有控制器自动受益，无需每个控制器各自计算
- 与SKILL.md的架构一致（§8, L270）: "Gravity model — Compute τ_gravity from kinematics"

**但组件必须可独立测试**：
- 当前 `ne_compensator_` 是 private 成员，无法直接测试
- 重构为：arm_hardware持有 `unique_ptr<NewtonEulerGravityCompensator>`，通过构造函数注入
- 测试时注入 mock/测试实例

---

#### 子问题 4a: 接口签名设计

**Q4a**: 新接口的完整签名是什么？

推荐答案：

```cpp
// NewtonEulerGravityCompensator 的公共接口
struct FeedforwardResult {
    std::array<double, 7> tau_ff;   // 总前馈力矩 = scale*(gravity+friction)
    std::array<double, 7> tau_g;    // 纯重力分量（用于调试/monitoring）
    std::array<double, 7> tau_f;    // 纯摩擦分量（用于调试/monitoring）
};

FeedforwardResult computeFeedforward(
    const std::array<double, 7>& joint_angles,   // 当前位置 [rad]
    const std::array<double, 7>& joint_velocities, // 当前速度 [rad/s]
    double hand_mass,                              // 手质量 [kg]
    double payload);                               // 负载 [kg]
```

**设计要点**：
- 返回结构体而非裸数组 → 可扩展
- 同时返回三个分量 → 便于 monitoring 和外力估计
- 参数明确 → 调用方不需要了解内部组合逻辑

---

#### 子问题 4b: 实时性保证

**Q4b**: 这个函数在 `write()` 实时循环中被调用，如何保证不违反实时约束？

推荐答案：**全栈分配 + 预计算 + 无系统调用。**

| 检查项 | 当前实现 | 新设计确保 |
|--------|---------|-----------|
| 堆分配 | ✅ `std::array` 全栈 | ✅ 保持 |
| 系统调用 | ✅ 无 | ✅ 保持 |
| `cos/sin` | 14次调用 | 14次调用（不变） |
| URDF解析 | - | ❌ 仅构造时一次 |
| 浮点运算量 | ~200 FLOP | ~250 FLOP（增加摩擦项计算） |

**性能预算**（200Hz控制周期 = 5ms）：
- NE递推当前估计 <1ms
- 增加摩擦计算 <0.1ms
- 总预算充足

---

#### 子问题 4c: 测试seam在哪?

**Q4c**: 遵循TDD, 这个功能的测试seam应该在哪里？

推荐答案：**`computeFeedforward()` 是主测试seam；`computeRotationMatrix()`、`crossProduct()` 是辅助seam。**

| Seam | 测试什么 | 怎么测试 |
|------|---------|---------|
| `computeFeedforward()` | 端到端：输入角度→输出τ_ff | 已知位姿(如全零位)，手工计算期望力矩 |
| `computeRotationMatrix()` | 3个轴的旋转矩阵正确性 | 单位向量映射测试 |
| 摩擦模型 | 零速时摩擦力=0 | 传入dq=0，期望τ_f=0 |
| tau_ff_scale | scale=0.5时输出减半 | 固定输入，改变scale |

**首要测试用例**（RED阶段）：
```
测试1: 全零位（竖直向上）→ 所有关节τ_ff = 0
  - 机械臂竖直时没有重力矩
  - 这是最基础的验证

测试2: L2 水平90° → τ_ff[1] ≈ m_l2·g·L_com + (m_l3+...+m_l7)·g·L_total
  - 手工计算期望值，对比NE输出

测试3: scale=0 → 所有输出为0
  - 验证scale正确传导
```

---

### 分支5: 配置管理

**Q5**: 配置数据从哪里来？

推荐答案：**选项B — 物理参数从URDF加载，调优参数从YAML加载。**

```
物理参数（不可调、由设计决定）    调优参数（可调、由实验决定）
├─ 质量 mass                        ├─ tau_ff_scale
├─ 质心 com_x/y/z                   ├─ Fc, Fv, Fo (摩擦)
├─ 变换 x/y/z                       ├─ gravity (重力加速度)
├─ 关节轴 axis xyz                  └─ gc_enabled (开关)
└─ 关节轴方向 sign
      ↑                                      ↑
  从 URDF 加载                         从 YAML 加载
   (arm.urdf)                     (gravity_compensation.yaml)
```

| 方案 | 优势 | 劣势 |
|------|------|------|
| A: 全YAML | 统一管理 | 与URDF不同步，易出错 |
| B: URDF+YAML | 单一真相来源 | 需要两个加载路径 |
| C: 全ROS2参数 | 极大灵活性 | 启动依赖参数服务器 |

**选B的理由**：
- URDF是机械臂物理参数的"单一真相来源"，不应在YAML中重复
- YAML是"控制工程师的调参本"，不应混入不会变的物理常数
- 两者的变更节奏完全不同：URDF随设计迭代（周/月），YAML随实验调参（时/天）

**需要解决的问题**：
- 如何在运行时从URDF提取参数？需要依赖 `urdfdom` 或 `tinyxml2`
- 编译时如果找不到URDF怎么办？→ 保留硬编码fallback（当前代码的参数）

---

## 设计树完整性检查

| 分支 | 决策 | 状态 |
|------|------|------|
| 根节点 | 公式: scale×(gravity+friction) | ✅ 确定 |
| 1. τ_gravity算法 | 自实现NE，URDF动态加载 | ✅ 确定 |
| 1a. 参数来源 | URDF加载+缓存 | ✅ 确定 |
| 1b. 惯性张量 | 保留但不参与重力计算 | ✅ 确定 |
| 1c. 后向递推完整度 | 保留当前简化，文档化误差 | ✅ 确定 |
| 2. τ_friction模型 | 标准模型: Fc·tanh+Fv·dq+Fo | ✅ 确定 |
| 2a. 标定方法 | 离线标定脚本 + YAML | ✅ 确定 |
| 2b. 计算位置 | NE类内部，统一返回 | ✅ 确定 |
| 2c. 自适应 | 不需要，Phase 2预留 | ✅ 确定 |
| 3. tau_ff_scale | YAML + ROS2参数双通道 | ✅ 确定 |
| 3a. 默认值 | 1.0 | ✅ 确定 |
| 3b. 校准工作流 | Teach mode人工观察 + ros2 param | ✅ 确定 |
| 4. 架构位置 | arm_hardware内部，可注入测试 | ✅ 确定 |
| 4a. 接口签名 | computeFeedforward() → FeedforwardResult | ✅ 确定 |
| 4b. 实时性 | <2ms @ 200Hz，预算充足 | ✅ 确定 |
| 4c. 测试seam | computeFeedforward()端到端 + 已知位姿单元测试 | ✅ 确定 |
| 5. 配置管理 | URDF物理参数 + YAML调优参数 | ✅ 确定 |

**设计树遍历完成。所有 15 个分支节点均已决策。**

---

## 沉默假设清单

以下是在设计过程中主动挖掘出的隐含假设，需要用户确认或否决：

1. **假设机械臂基座竖直安装** — 重力方向恒为世界 Z 轴。如果平台倾斜或移动，需要导入 IMU 数据修正重力方向。
   - → 当前设计保留硬编码 Z 轴，但接口预留 `setGravityDirection(x,y,z)` 扩展点

2. **假设 URDF 参数与实物一致** — 如果加工误差导致实际质量/质心偏移，NE 计算结果会有系统误差。
   - → 这正是 tau_ff_scale 存在的理由：吸收制造公差

3. **假设低速运动** — Coriolis 和离心力忽略。如果VLA推理要求高速运动（>2 rad/s），需要考虑。
   - → Phase 2 扩展路径：NE类增加 velocity/acceleration 输入

4. **假设单臂独立工作** — 双臂协作时可能存在基座受力耦合，当前模型不考虑。
   - → 如果双臂同时高频运动，基座反力会影响两臂的重力方向感知

5. **假设摩擦参数在负载变化时不变** — 加载 payload 后关节轴承摩擦力可能增大。
   - → 标定脚本需要支持"空载标定"和"负载标定"两个模式

---

## 与现有代码的差异总结

| 方面 | 当前实现 | 新设计 |
|------|---------|--------|
| 公式 | `τ_gravity` 一项 | `scale × (τ_gravity + τ_friction)` 三项 |
| 参数来源 | 硬编码 | URDF(物理) + YAML(调优) |
| 摩擦 | 无 | 标准4参数模型 |
| 校准 | 无 | scale逐关节 + 标定脚本 |
| 接口 | `computeGravityCompensation(index)` | `computeFeedforward()` 返回结构体 |
| 可测试性 | 私有成员，难以单元测试 | 可注入，有明确测试seam |
| 配置 | 一个YAML: `gravity_compensation.yaml` | 两个来源: URDF + YAML |
| 实时性 | 堆分配安全 | 保持（增加<0.1ms） |
| 重力方向 | 硬编码Z轴 | 硬编码+预留扩展点 |

---

## 用户确认记录（2026-07-27）

| # | 决策点 | 确认结果 |
|---|--------|---------|
| 1 | 接口范围 | **全改**：computeGravityCompensation() → computeFeedforward()，所有调用方一并修改 |
| 2 | URDF 依赖 | **接受 urdfdom**：引入编译依赖，构造时从 URDF 动态加载物理参数 |
| 3 | 标定脚本 | **独立 Python 脚本**：`scripts/calibrate_friction.py`，用户手动运行 |
| 4 | 向后兼容 | **严格报错**：旧 YAML 缺少新字段时启动失败，强制用户更新配置 |
| 5 | 测试策略 | **TDD 先测试**：先写 RED 测试（已知位姿手工计算），再实现 GREEN |

---

## 最终实现优先级（基于确认结果）

```
Phase 1 (P0): TDD RED 测试 + 接口重构 + URDF 加载
  ├─ 写测试：零位→力矩=0，L2水平90°→手工期望值，scale=0→输出0
  ├─ 改签名：computeGravityCompensation() → computeFeedforward()
  ├─ 引入 urdfdom，构造函数加载 URDF 参数
  └─ 修改 gravity_compensation.yaml schema（新增 scale/friction 字段）

Phase 2 (P1): tau_ff_scale + 摩擦模型
  ├─ 实现 scale 校准逻辑
  ├─ 实现 compute_friction_compensation()（默认全0，等待标定）
  └─ 实现 YAML 严格校验（缺字段→报错）

Phase 3 (P2): 标定脚本
  └─ scripts/calibrate_friction.py（恒速扫描 + 最小二乘拟合）
```
