## Parent
gravity_compensation_redesign_grill.md — Phase 1 (P0) + Phase 2 (P1)，分支 5

### What to build
更新 `gravity_compensation.yaml` 的 schema，添加新字段并实现严格校验。

**YAML 新 schema**（grill 文档设计汇总）：

```yaml
gravity_compensation:
  joints:
    L1_joint:
      scale: 1.0              # 新增：tau_ff_scale，范围 0.85~1.05
      friction:                # 新增：摩擦模型参数
        Fc: 0.0                # 库仑摩擦 (Nm)
        k: 100.0               # 库仑锐度
        Fv: 0.0                # 粘性摩擦系数 (Nm·s/rad)
        Fo: 0.0                # 静态偏移 (Nm)
      gravity_offset: 0.0      # 保留已有字段
    # L2~L7 同理
```

**严格校验规则**（grill 文档确认记录 #4）：
- 旧 YAML 缺少 `scale` 或 `friction` 字段 → **启动失败并打印清晰错误信息**
- `scale` 值超出 [0.50, 1.50] → 警告，但仍加载（不阻塞启动）
- `Fc/Fv/Fo` 为负 → 报错（物理不合理）

**实现**：
- 用 `YAML::Node` 解析，手动校验每个字段存在性与合法性
- 错误信息格式：`[gravity_compensation] Missing field: joints.L1_joint.scale`
- 写单元测试验证：合法 YAML 通过、缺少字段报错、负值报错

### Acceptance criteria
- [ ] `gravity_compensation.yaml` schema 已更新，包含所有新字段
- [ ] 缺少必需字段时启动报错，错误信息明确指出缺失字段名
- [ ] `scale` 超范围时警告但不阻塞
- [ ] `Fc/Fv/Fo` 为负时报错
- [ ] 提供示例 YAML 文件（`gravity_compensation.example.yaml`）
- [ ] 旧 YAML 加载失败时提示："请参考 example.yaml 更新配置"

### Blocked by
- #02 (接口重构) — schema 字段命名需与 `FeedforwardResult` 结构体一致
