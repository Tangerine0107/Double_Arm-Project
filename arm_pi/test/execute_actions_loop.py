"""将策略推理的 10 步 action 下发到右臂控制器。

用法:
    source /opt/ros/humble/setup.bash
    source .venv/bin/activate
    export PYTHONPATH=packages/openpi-client/src:src:.:$PYTHONPATH
    python scripts/execute_actions.py

流程:
    1. 从 ROS2 topic 读取当前状态和图像
    2. 调用策略推理获取 10 步 action
    3. 将 action 作为 JointTrajectory 下发到 right_arm_controller
    4. 同时控制夹爪
"""

import argparse
import logging
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.action import ActionClient
from cv_bridge import CvBridge
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs

from openpi_client import websocket_client_policy

# 右臂关节名（与 urdf/controller 配置一致）
ARM_JOINT_NAMES = [
    "R1_joint",
    "R2_joint",
    "R3_joint",
    "R4_joint",
    "R5_joint",
    "R6_joint",
    "R7_joint",
]

# 短关节名（用于日志显示对齐）
ARM_JOINT_SHORT = ["R1", "R2", "R3", "R4", "R5", "R6", "R7", "G"]

# 每步之间的时间间隔（秒）
STEP_DURATION = 0.2

# 夹爪二值化阈值（带滞回，防止临界抖动）
GRIPPER_OPEN_TH = 0.6   # 策略输出 ≥ 0.6 视为"张开"，下发 1.0
GRIPPER_CLOSE_TH = 0.4  # 策略输出 ≤ 0.4 视为"闭合"，下发 0.0

# 图像尺寸
IMG_H, IMG_W = 256, 256

# 末端执行器 link 名称（根据实际 URDF 调整）
END_EFFECTOR_LINK = "R7_Link"
BASE_LINK = "base_link"


def format_joint_array(arr, joint_names=None, col_width=11):
    """格式化关节数组，保留3位小数，正数加+号，纵向对齐显示。

    Args:
        arr: 关节值数组 (list 或 np.ndarray)
        joint_names: 关节名列表，为 None 时使用 J1-J7 + G
        col_width: 每列固定宽度（含分隔空格）

    Returns:
        格式化后的字符串，列对齐便于纵向比较
    """
    if joint_names is None:
        joint_names = [f"J{i+1}" for i in range(min(len(arr), 7))]
        if len(arr) > 7:
            joint_names.append("G")

    parts = []
    for i, val in enumerate(arr):
        name = joint_names[i] if i < len(joint_names) else f"J{i+1}"
        # 格式：名=值，正数加+号，负数自带-号，保留3位小数
        formatted = f"{name}={val:+.3f}"
        # 固定宽度，右填充空格
        parts.append(f"{formatted:<{col_width}}")

    return "".join(parts)


class ActionExecutor(Node):
    """从 ROS2 topic 收集数据，调用策略推理，下发 action 到右臂控制器。"""

    def __init__(self, file_logger: logging.Logger | None = None) -> None:
        super().__init__("action_executor")
        self._file_logger = file_logger

        self.bridge = CvBridge()
        self.latest_state = np.zeros(8, dtype=np.float32)
        self.latest_front_img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        self.latest_wrist_img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        self._has_front_img = False
        self._has_wrist_img = False
        self._has_arm_state = False

        # TF2 用于获取末端坐标
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅
        self.create_subscription(Image, "/cropped/rgb", self._front_img_cb, 1)
        self.create_subscription(Image, "/cropped_wrist/rgb", self._wrist_img_cb, 1)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/right_arm_controller/controller_state",
            self._arm_state_cb,
            1,
        )
        self.create_subscription(Float32MultiArray, "/wheeltec_gripper_state", self._gripper_cb, 1)

        # Action 客户端（使用 FollowJointTrajectory action 而非 topic，确保轨迹被控制器正确执行）
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory"
        )
        self._gripper_pub = self.create_publisher(Float32MultiArray, "/wheeltec_gripper_action", 1)

        self.log_info("ActionExecutor 节点已启动")

    def log_info(self, msg: str) -> None:
        """同时输出到 ROS2 日志和文件日志。"""
        self.get_logger().info(msg)
        if self._file_logger:
            self._file_logger.info(msg)

    def log_error(self, msg: str) -> None:
        """同时输出到 ROS2 日志和文件日志。"""
        self.get_logger().error(msg)
        if self._file_logger:
            self._file_logger.error(msg)

    def _front_img_cb(self, msg: Image) -> None:
        self.latest_front_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self._has_front_img = True

    def _wrist_img_cb(self, msg: Image) -> None:
        self.latest_wrist_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self._has_wrist_img = True

    def _arm_state_cb(self, msg: JointTrajectoryControllerState) -> None:
        joints = list(msg.actual.positions[:7])
        self.latest_state[:7] = joints
        self._has_arm_state = True

    def _gripper_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 1:
            # data[0] 是位置百分比 (0~100)，归一化到 0~1
            self.latest_state[7] = msg.data[0] / 100.0

    def ready(self) -> bool:
        return self._has_front_img and self._has_wrist_img and self._has_arm_state

    def get_obs(self, prompt: str) -> dict:
        return {
            "observation/state": self.latest_state.copy(),
            "observation/image": self.latest_front_img.copy(),
            "observation/wrist_image": self.latest_wrist_img.copy(),
            "prompt": prompt,
        }

    def send_trajectory(self, actions: np.ndarray, current_state: np.ndarray | None = None) -> bool:
        """将 action 序列作为 FollowJointTrajectory goal 下发到右臂控制器。

        使用 action 接口（而非 topic），控制器会自动从当前状态过渡到轨迹起点。
        采用显式 spin_once 循环等待，避免 spin_until_future_complete 死锁。

        Args:
            actions: shape (N, 8)，前 7 维是关节位置，第 8 维是夹爪
            current_state: 当前关节状态 (7,)，用于构造轨迹起点；为 None 时使用 latest_state

        Returns:
            True 表示轨迹执行成功，False 表示失败
        """
        # 构造轨迹消息
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ARM_JOINT_NAMES

        # 第 0 点：当前关节角，确保 start tolerance 满足
        if current_state is not None:
            q0 = current_state[:7].tolist()
        else:
            q0 = self.latest_state[:7].tolist()

        p0 = JointTrajectoryPoint()
        p0.positions = q0
        p0.velocities = [0.0] * 7
        p0.accelerations = [0.0] * 7
        p0.time_from_start = Duration(sec=0, nanosec=0)
        traj_msg.points.append(p0)

        # 后续动作点
        for i in range(actions.shape[0]):
            point = JointTrajectoryPoint()
            point.positions = actions[i, :7].tolist()
            point.velocities = [0.0] * 7
            point.accelerations = [0.0] * 7
            t_ns = int((i + 1) * STEP_DURATION * 1e9)
            point.time_from_start = Duration(sec=0, nanosec=t_ns)
            traj_msg.points.append(point)

        total_duration = actions.shape[0] * STEP_DURATION
        self.log_info(f"轨迹: {len(traj_msg.points)} 点, 总时长 {total_duration:.1f}s")

        # 构造 action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj_msg

        # 同步等待服务器就绪
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.log_error("FollowJointTrajectory action 服务器不可用!")
            return False

        # 发送 goal（异步）并等待接受 —— 显式 spin_once 循环
        self.log_info("发送 goal...")
        send_future = self._traj_client.send_goal_async(goal)
        start = time.time()
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 5.0:
                self.log_error("发送 goal 超时!")
                return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.log_error("Goal 被控制器拒绝!")
            return False

        self.log_info("Goal 已接受，等待执行完成...")

        # 等待执行结果 —— 显式 spin_once 循环
        result_future = goal_handle.get_result_async()
        timeout = total_duration + 10.0
        start = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            if elapsed > timeout:
                self.log_error(f"等待轨迹执行结果超时 (>{timeout:.1f}s)")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        # result.status 是 goal 的状态（SUCCEEDED=4），result.result 是 FollowJointTrajectory_Result
        status = result.status
        # SUCCESSFUL = 4
        if status == 4:
            self.log_info(f"轨迹执行成功 (status={status})")
            return True
        else:
            self.log_error(f"轨迹执行失败 (status={status})")
            return False

    def send_gripper(self, gripper_value: float) -> None:
        """下发夹爪控制指令。

        Args:
            gripper_value: 夹爪值（1=开, 0=合）
        """
        msg = Float32MultiArray()
        msg.data = [float(gripper_value), -1.0]
        self._gripper_pub.publish(msg)
        self.log_info(f"已下发夹爪指令: {gripper_value}")

    def get_end_effector_position(self) -> str:
        """获取末端执行器在基坐标系下的位置。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_LINK, END_EFFECTOR_LINK, rclpy.time.Time()
            )
            t = transform.transform.translation
            return f"末端坐标 [{BASE_LINK}]: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}"
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return f"获取末端坐标失败: {e}"


def main() -> None:
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="执行策略推理的 10 步动作")
    parser.add_argument("--steps", type=int, default=10, choices=range(1, 11),
                        help="执行前 N 步动作（1-10，默认 10）")
    parser.add_argument("--start-step", type=int, default=1, choices=range(1, 11),
                        help="从第 N 步开始执行（1-10，默认 1）")
    args = parser.parse_args()
    num_steps = args.steps
    start_step = args.start_step

    # 配置文件日志（放到 logs 子目录）
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"execute_actions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    file_logger = logging.getLogger("execute_actions")
    file_logger.setLevel(logging.INFO)
    file_handler = logging.FileHandler(log_file, encoding="utf-8")
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter("[%(levelname)s] [%(asctime)s] %(message)s"))
    file_logger.addHandler(file_handler)
    file_logger.propagate = False  # 阻止日志传播到 root logger，避免重复打印

    rclpy.init()

    executor = ActionExecutor(file_logger=file_logger)
    executor.log_info(f"日志文件: {log_file}")

    # 等待数据就绪
    executor.log_info("等待 ROS2 topic 数据...")
    rclpy.spin_once(executor, timeout_sec=0.5)
    while not executor.ready() and rclpy.ok():
        rclpy.spin_once(executor, timeout_sec=0.5)
    executor.log_info("所有 topic 数据已就绪")

    # 等待 FollowJointTrajectory action 服务器就绪
    executor.log_info("等待 FollowJointTrajectory action 服务器...")
    if not executor._traj_client.wait_for_server(timeout_sec=10.0):
        executor.log_error("FollowJointTrajectory action 服务器不可用! 请确认 right_arm_controller 已启动")
        executor.destroy_node()
        rclpy.shutdown()
        return
    executor.log_info("FollowJointTrajectory action 服务器已就绪")

    # 连接策略服务
    client = websocket_client_policy.WebsocketClientPolicy(host="192.168.127.83", port=8002)

    # 初始化夹爪到全开状态（与采集数据的初始状态一致，
    # 避免观测中夹爪处于半开 0.5 附近导致策略输出也在 0.5 徘徊）
    executor.log_info("初始化夹爪到全开状态...")
    executor.send_gripper(1.0)
    time.sleep(2.0)
    
    prompt = "pick up the bottle and place it into the white plate"

    # 循环控制参数
    max_loops = 50  # 0 表示无限循环
    loop_count = 0

    executor.log_info("开始循环推理执行，Ctrl+C 停止")

    try:
        while rclpy.ok() and (max_loops == 0 or loop_count < max_loops):
            # ① 处理 ROS2 消息，获取最新状态
            rclpy.spin_once(executor, timeout_sec=0.1)

            # ② 获取最新观测数据（实时状态 + 图像）
            obs = executor.get_obs(prompt)
            state_str = format_joint_array(obs['observation/state'], ARM_JOINT_SHORT)
            executor.log_info(f"\n[循环 {loop_count + 1}] 当前关节状态:\n  {state_str}")

            # ③ 推理
            t0 = time.time()
            try:
                out = client.infer(obs)
                actions = np.asarray(out["actions"])
                executor.log_info(f"推理完成, 耗时 {time.time() - t0:.1f}s, shape={actions.shape}")
            except Exception as e:
                executor.log_error(f"推理失败: {e}")
                loop_count += 1
                time.sleep(0.5)
                continue

            # ④ NaN 检测
            if np.isnan(actions).any():
                executor.log_error("动作包含 NaN，跳过本次执行！")
                loop_count += 1
                time.sleep(0.5)
                continue

            # 完整打印全部推理步（含未执行的，用于观察夹爪闭合意图出现在哪一步）
            full_str = "\n".join(
                f"{format_joint_array(row, ARM_JOINT_SHORT)}"
                for row in actions.tolist()
            )
            executor.log_info(f"完整 {len(actions)} 步动作:\n{full_str}")

            # ⑤ 执行从 start_step 到 num_steps 的动作
            actions_to_execute = actions[start_step - 1:num_steps]
            exec_str = "\n".join(
                f"  {format_joint_array(row, ARM_JOINT_SHORT)}"
                for i, row in enumerate(actions_to_execute.tolist())
            )
            executor.log_info(
                f"执行第 {start_step} 到 {num_steps} 步动作（共 {len(actions_to_execute)} 步）:\n{exec_str}"
            )

            # ⑥ 提取夹爪目标值（取最后一部的夹爪值）
            gripper_target = float(actions_to_execute[-1, 7])
            executor.log_info(f"夹爪目标值: {gripper_target:.3f}")

            # ⑦ 先下发关节轨迹（阻塞等待执行完成）
            executor.log_info(f"开始执行轨迹 (action 模式，阻塞等待)...")
            current_state = obs['observation/state']
            success = executor.send_trajectory(actions_to_execute, current_state=current_state)

            if not success:
                executor.log_error("轨迹执行失败，跳过本次夹爪下发")
                loop_count += 1
                time.sleep(0.5)
                continue

            # ⑧ 轨迹执行完成后，spin 一下让状态更新
            rclpy.spin_once(executor, timeout_sec=0.2)

            # ⑨ 下发夹爪（策略输出二值化 + 滞回，恢复接近 0/1 的效果）
            if gripper_target >= GRIPPER_OPEN_TH:
                gripper_cmd = 1.0   # 张开
            elif gripper_target <= GRIPPER_CLOSE_TH:
                gripper_cmd = 0.0   # 闭合
            else:
                gripper_cmd = None  # 中间区 → 保持上一次指令，避免半开
            if gripper_cmd is not None:
                executor.log_info(f"夹爪: 策略 {gripper_target:.3f} → 下发 {gripper_cmd:.1f}")
                executor.send_gripper(gripper_cmd)
            else:
                executor.log_info(f"夹爪: 策略 {gripper_target:.3f} 在中间区 [{GRIPPER_CLOSE_TH}, {GRIPPER_OPEN_TH}]，保持上次指令")

            executor.log_info(f"[循环 {loop_count + 1}] 执行完成")

            # ⑩ 打印末端坐标
            end_pos = executor.get_end_effector_position()
            executor.log_info(end_pos)

            loop_count += 1
            time.sleep(0.1)  # 短暂等待，让状态更新

    except KeyboardInterrupt:
        executor.log_info(f"用户中断，共执行 {loop_count} 次循环")

    executor.log_info("执行完成")
    
    # 刷新日志缓冲区，确保所有日志写入文件
    for handler in file_logger.handlers:
        handler.flush()
    
    executor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
