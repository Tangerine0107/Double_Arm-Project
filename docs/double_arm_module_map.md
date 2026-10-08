# Double Arm Robot 项目模块地图

> 基于 ROS 2 的 14 自由度双臂机器人操作系统，覆盖硬件驱动、运动规划、视觉抓取、遥操作、示教和策略部署。

## 总体数据流

```text
真实硬件 / XR / 相机 / AI策略
          ↓
ROS 2 通信与任务编排
          ↓
MoveIt2 / IK / 视觉定位 / 手部控制
          ↓
ros2_control
          ↓
ArmHardware
          ↓
MIT协议 + SocketCAN
          ↓
14个关节电机
```

## 模块总览

| 模块 | 目录 | 主要职责 | 面试关键词 |
|---|---|---|---|
| 机器人描述 | `arm_description` | URDF、mesh、关节/惯量/限位、TF基础 | 机器人模型、坐标系 |
| 底层控制 | `arm_hardware` | ros2_control插件、CAN、MIT协议、反馈与指令 | 硬件抽象、实时循环、安全边界 |
| 控制器与底层SDK | `arm_controller` | C/C++ CAN端口、MIT协议、回零、动作测试 | 协议编解码、BSP |
| 运动规划 | `arm_moveit` | MoveIt2、OMPL/Pilz、Ruckig、控制器连接 | 规划、碰撞检测、轨迹平滑 |
| 视觉感知 | `arm_vision` | Gemini 335L、YOLOv11、深度、TF坐标转换 | RGB-D、2D→3D、外参 |
| 抓取编排 | `grasp_bringup` | 识别→规划→抓取任务状态机 | Service/Action、任务模型 |
| 双臂遥操作 | `teleop` | XR输入、镜像映射、DLS IK、肘部/姿态控制 | 坐标变换、奇异处理、FK保护 |
| 示教模式 | `arm_hardware` + `grasp_bringup` | 共享内存、接管、hold轨迹、模式切换 | 状态同步、跳变保护 |
| OpenPI策略部署 | `arm_pi` | 策略观测、WebSocket、动作分发、夹爪适配 | 模型部署、协议桥接 |
| 仿真与验证 | `arm_simulation`、各模块 test | IK/FK、规划、协议、策略安全测试 | Sim2Real、回归测试 |

## 分层架构

### 1. 设备与模型层

- `arm_description/urdf/arm.urdf`：14个机械臂关节模型。
- `arm_description/urdf/double_arm_with_hands.urdf`：包含双臂和手部关节的完整模型。
- Gemini 335L：提供彩色图、深度图和相机内参。
- 左右臂电机：分别连接 `can0` 和 `can1`。
- LinkerHand / WHEELTEC：左右侧不同末端执行器。

### 2. 硬件抽象层：`arm_hardware`

```text
controller command[14]
        ↓
启动保护 / jump guard / 关节限位
        ↓
Pinocchio重力补偿 + 摩擦前馈
        ↓
MITCtrl_SendCommandValues()
        ↓
SocketCAN → 14个电机
```

反馈路径：

```text
电机反馈帧 → CAN缓冲区异步收割 → motor_id匹配
→ positions / velocities / efforts → ros2_control → /joint_states
```

主要安全机制：启动保护、位置软限位、电机故障检查、指令跳变保护、只读模式，以及前馈初始化失败时的零前馈降级。

### 3. 运动控制层：`arm_moveit`

```text
目标关节角 / 末端位姿
          ↓
MoveGroup
          ↓
OMPL / Pilz规划
          ↓
Ruckig时间参数化与平滑
          ↓
FollowJointTrajectory
          ↓
joint_trajectory_controller → arm_hardware
```

### 4. 视觉感知层：`arm_vision`

```text
Gemini RGB-D → YOLOv11 → 2D bbox
→ 深度窗口采样 → 像素+深度+内参
→ 相机系3D点 → TF/齐次变换 → base系目标
```

核心接口：`/yolo_detection`、`/get_target_pose`、`/run_local_perception_once`、`/run_local_perception`。

支持 HBM、Ultralytics 和 Mock 后端，便于真机、PC 和离线测试切换。

### 5. 抓取业务层：`grasp_bringup`

`grasp_manager_node.py` 维护任务配置、步骤游标、感知缓存、规划/执行记录、误差报告和后台任务线程。

支持步骤：`joint`、`pose`、`vision_pose`、`vision_track`、`offset_pose`、`hand_action`。

状态机：

```text
IDLE → DETECTING → DETECTED → PLANNING → PLANNED → EXECUTING → DONE
  └──────────────────────────────────────────────────────→ ERROR
                              EXECUTING → STOPPING
```

配套模块：`grasp_moveit_server.py`、`grasp_moveit_server_bezier.py`、`hand_grasp_server.py`、`vision_track.py`、`task_model.py`、`task_recorder/`。

### 6. XR双臂遥操作层：`teleop`

```text
PICO/XR手柄 → 坐标变换与镜像 → clutch锚点
→ 位姿滤波/预测 → 加权DLS IK
→ 姿态保持/arm-angle/肘部软任务
→ 奇异阻尼/限位/速度限幅
→ 真实FK方向检查 → ROS2 controller
```

核心方程：

\[
\dot q=(J^T W J+\lambda^2I)^{-1}J^T Wv
\]

重点设计：绝对位姿锚定减少积分漂移；自适应阻尼处理奇异区；关节加权阻尼抑制腕关节抢占位置任务；最终用真实FK前推检查末端位移方向。

### 7. 示教与人工接管层

```text
teach CLI/service → TeachModeServer → POSIX Shared Memory
→ TeachModeManager → ArmHardware::write()
→ 位置自指 / Kp-Kd覆盖 / jump guard → 电机
```

退出示教时通过 FollowJointTrajectory 发送当前位置 hold 轨迹，减少旧目标重新接管造成的阶跃。

### 8. OpenPI策略部署层：`arm_pi`

```text
OpenPI策略 ←WebSocket→ openpi_bridge
                          ├─ ObservationCollector
                          ├─ ActionDispatcher
                          ├─ EndEffector
                          └─ WebSocketServer
                                      ↓
                         ROS2关节命令 + 夹爪服务
```

Bridge 负责观测组织、动作协议、7维关节与1维夹爪拆分、末端适配和 stop 指令；最终安全约束仍应由硬件层统一兜底。

## 三条完整业务链路

### 视觉抓取

```text
相机 → YOLO → 深度/TF定位 → vision_pose或vision_track
→ MoveIt规划 → 轨迹执行 → 夹爪 → FK/关节误差报告
```

### XR遥操作

```text
PICO → 映射 → 锚点/滤波 → DLS IK → 奇异/限位保护
→ ROS2 controller → ros2_control → CAN/MIT电机
```

### OpenPI策略控制

```text
相机 + joint_states + gripper state
→ WebSocket观测 → OpenPI推理 → 8维动作
→ 7维关节 + 1维夹爪 → ROS2控制器/末端服务
```

## 面试定位

> 我负责的是一套基于 ROS2 的 14 自由度双臂机器人操作系统，打通了 SocketCAN/MIT 电机驱动、ros2_control 硬件抽象、MoveIt2 规划、RGB-D 视觉定位、YAML 任务编排、PICO 双臂遥操作、示教接管以及 OpenPI 策略部署。重点解决了真实机器人中的坐标系一致性、控制模式切换、前馈补偿、奇异位形、指令跳变和执行误差闭环问题。

## 当前工程化风险

1. `arm.urdf` 与 `double_arm_with_hands.urdf` 并存，模型可能不一致；
2. 部分安全关键配置失败后仍可能使用默认值，应改为拒绝启动；
3. 感知外参缺失时不应继续进入抓取规划；
4. OpenPI Bridge 默认监听 `0.0.0.0`，应增加认证、watchdog 和动作边界；
5. 根目录 README 和统一构建/部署文档不足；
6. 多个嵌套 Git 目录的仓库边界需要统一；
7. 硬件、动力学补偿和任务状态机还应增加更多纯单元测试。
