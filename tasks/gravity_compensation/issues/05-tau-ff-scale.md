## Parent
gravity_compensation_redesign_grill.md — Phase 2 (P1)，分支 3

### What to build
实现 `tau_ff_scale` 校准通道，支持 YAML 静态配置和 ROS2 参数运行时动态调整两个路径。

**公式**（grill 文档根节点）：
```
τ_ff = scale × (τ_gravity + τ_friction)
```

**双通道设计**（grill 文档分支 3）：
1. **YAML 静态配置**：从 `gravity_compensation.yaml` 加载每关节 `scale` 值，作为持久化存储
2. **ROS2 参数动态调整**：通过 `ros2 param set /arm_hardware/gc_scale_L2 0.98` 临时覆盖 YAML 值

优先级：ROS2 参数覆盖 YAML 值；若 ROS2 参数未设置，回退到 YAML。

**范围限制**：
- `scale` 裁剪到 [0.50, 1.50]，防止极端值
- 默认值 1.0（grill 文档 §3a）

**校准工作流**（grill 文档 §3b）：
1. 启动 teach mode（Kp=Kp_min）
2. 手动将关节移到不同角度 → 松开 → 观察漂移方向
3. 向下漂 → `ros2 param set gc_scale_L3 1.02`；向上漂 → 设 0.98
4. 确认无漂移后，手动将值写回 YAML

### Acceptance criteria
- [ ] `gravity_compensation.yaml` 中的 `scale` 字段被正确加载
- [ ] ROS2 参数 `gc_scale_L1` ~ `gc_scale_L7` 可被读取并覆盖 YAML 值
- [ ] ROS2 参数未设置时回退到 YAML
- [ ] `scale` 值被裁剪到 [0.50, 1.50]
- [ ] `computeFeedforward()` 输出的 `tau_ff` 已乘以 scale
- [ ] `ros2 param describe /arm_hardware gc_scale_L2` 可看到参数说明
- [ ] #1 的 scale=0 测试保持 GREEN

### Blocked by
- #03 (URDF loading) — 需要 URDF 加载完成后的重力力矩数据验证 scale 效果
- #04 (YAML schema) — schema 定义 scale 字段
