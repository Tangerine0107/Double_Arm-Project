## Parent
gravity_compensation_redesign_grill.md — Phase 3 (P2)，分支 2a

### What to build
提供一个离线 Python 标定脚本，用于获取每个关节的摩擦模型参数 (Fc, Fv, Fo)。

**脚本设计**（grill 文档 §2a）：
- 独立 Python 脚本 `scripts/calibrate_friction.py`
- 不需编译，用户手动运行
- 依赖：`rclpy` + `numpy` + `scipy.optimize.least_squares`

**标定流程**：
1. 将机械臂置于安全位姿
2. 对指定关节执行恒速扫描：`-max_vel ... +max_vel` (rad/s)，步长约 0.05 rad/s
3. 对每个速度点，记录：`(dq, τ_motor, τ_gravity)`  →  算出 `τ_friction = τ_motor - τ_gravity`
4. 收集足够数据点后，做最小二乘拟合：
   - `Fo` = 零速截距（dq=0 时的 τ_friction）
   - `Fv` = 线性段斜率（高速区的 dτ/d(dq)）
   - `Fc` = 低速段非线性幅度
5. 把结果输出到 stdout（建议同时写入临时 YAML 片段）

**注意事项**（grill 文档 §2a）：
- 重力补偿必须在标定时启用（τ_gravity 已知且已校准）
- 单关节模式，不支持多关节联动（交叉耦合复杂，Phase 2 预留）
- 空载模式和带灵巧手模式分别标定（摩擦力不同）
- 需要安全限位：确保标定运动不超过关节限位
- 每关节预计耗时 2~5 分钟

**输出格式**：
```yaml
# 可直接粘贴到 gravity_compensation.yaml
L1_joint:
  friction:
    Fc: 0.023
    Fv: 0.018
    Fo: 0.005
```

**使用示例**：
```bash
ros2 launch arm_moveit demo.launch.py  # 机械臂就绪
python3 scripts/calibrate_friction.py \
  --joint L2 \
  --max-vel 1.0 \
  --output friction_L2.yaml
```

### Acceptance criteria
- [ ] 脚本 `scripts/calibrate_friction.py` 存在且可执行
- [ ] 支持 `--joint` 指定关节、`--max-vel` 指定速度范围
- [ ] 恒速扫描逻辑正确，不超关节限位
- [ ] 最小二乘拟合输出 (Fc, Fv, Fo)，带 R² 拟合优度
- [ ] 输出格式为可直接粘贴的 YAML 片段
- [ ] 安全机制：扫描前检查限位、扫描中允许 Ctrl+C 中断并输出已采集数据
- [ ] 文档注释中包含用法示例

### Blocked by
- #06 (friction model) — 标定脚本依赖摩擦模型接口确定输出字段名和单位
