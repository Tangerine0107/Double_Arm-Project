import mujoco
import mujoco.viewer
import numpy as np
import time
import random

class ArmSimulation:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 获取左右臂的关节和末端 ID
        self.l_joint_names = [f'L{i}_joint' for i in range(1, 8)]
        self.r_joint_names = [f'R{i}_joint' for i in range(1, 8)]
        
        self.l_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.l_joint_names]
        self.r_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.r_joint_names]
        
        self.l_qpos_adr = [self.model.jnt_qposadr[jid] for jid in self.l_joint_ids]
        self.r_qpos_adr = [self.model.jnt_qposadr[jid] for jid in self.r_joint_ids]
        
        self.l_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'L_ee')
        self.r_ee_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'R_ee')
        
        self.target_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'target')

    def get_ik(self, target_pos, target_quat, arm='L'):
        """
        简单的数值 IK 求解
        """
        ee_id = self.l_ee_id if arm == 'L' else self.r_ee_id
        qpos_adr = self.l_qpos_adr if arm == 'L' else self.r_qpos_adr
        
        # 初始猜测使用当前位置
        q = self.data.qpos[qpos_adr].copy()
        
        # IK 参数
        step_size = 0.5
        tol = 1e-4
        max_iters = 100
        
        for i in range(max_iters):
            mujoco.mj_kinematics(self.model, self.data)
            
            # 当前末端位置和姿态
            curr_pos = self.data.site_xpos[ee_id]
            curr_mat = self.data.site_xmat[ee_id].reshape(3, 3)
            
            # 计算位置误差
            pos_err = target_pos - curr_pos
            
            # 计算旋转误差 (简化的旋转误差计算)
            target_mat = np.zeros(9)
            mujoco.mju_quat2Mat(target_mat, target_quat)
            target_mat = target_mat.reshape(3, 3)
            
            rot_err_mat = target_mat @ curr_mat.T
            rot_err_quat = np.zeros(4)
            mujoco.mju_mat2Quat(rot_err_quat, rot_err_mat.flatten())
            rot_err = rot_err_quat[1:] * np.sign(rot_err_quat[0]) # 旋转向量近似
            
            err = np.hstack([pos_err, rot_err])
            
            if np.linalg.norm(err) < tol:
                return q
            
            # 计算雅可比
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, ee_id)
            
            # 提取相关关节的雅可比
            dof_indices = [self.model.jnt_dofadr[jid] for jid in (self.l_joint_ids if arm == 'L' else self.r_joint_ids)]
            J = np.vstack([jacp[:, dof_indices], jacr[:, dof_indices]])
            
            # 更新 q
            dq = np.linalg.pinv(J) @ err
            q += step_size * dq
            
            # 限制关节范围
            for j, jid in enumerate(self.l_joint_ids if arm == 'L' else self.r_joint_ids):
                j_range = self.model.jnt_range[jid]
                q[j] = np.clip(q[j], j_range[0], j_range[1])
            
            self.data.qpos[qpos_adr] = q
            
        return q

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                # 1. 随机生成目标位置
                # 假设在机器人的前方工作空间内
                arm_choice = random.choice(['L', 'R'])
                side = 1 if arm_choice == 'L' else -1
                
                target_pos = np.array([
                    random.uniform(0.3, 0.6), # 前向 x
                    random.uniform(0.1, 0.4) * side, # 侧向 y
                    random.uniform(0.8, 1.2) # 高度 z
                ])
                target_quat = np.array([1, 0, 0, 0]) # 默认朝向
                
                # 更新可视化目标点
                self.data.mocap_pos[0] = target_pos
                
                print(f"Moving {arm_choice} arm to {target_pos}")
                
                # 2. 求解 IK
                q_sol = self.get_ik(target_pos, target_quat, arm=arm_choice)
                
                # 3. 控制移动 (使用简单的插值模拟 MoveIt 轨迹)
                qpos_adr = self.l_qpos_adr if arm_choice == 'L' else self.r_qpos_adr
                start_q = self.data.qpos[qpos_adr].copy()
                
                steps = 100
                collision_detected = False
                for s in range(steps):
                    interp_q = start_q + (q_sol - start_q) * (s / steps)
                    
                    if arm_choice == 'L':
                        self.data.ctrl[0:7] = interp_q
                    else:
                        self.data.ctrl[7:14] = interp_q
                        
                    mujoco.mj_step(self.model, self.data)
                    
                    # 碰撞检测
                    if self.data.ncon > 0:
                        # 检查是否有非地面的碰撞
                        for i in range(self.data.ncon):
                            contact = self.data.contact[i]
                            geom1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                            geom2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                            
                            # 忽略与地面的正常碰撞（如果有的话，虽然通常 floor 不参与）
                            if geom1_name != 'floor' and geom2_name != 'floor':
                                print(f"Collision detected between {geom1_name} and {geom2_name}!")
                                collision_detected = True
                                break
                    
                    if collision_detected:
                        print("Stopping due to collision risk.")
                        # 可以在这里添加回退逻辑
                        break

                    viewer.sync()
                    time.sleep(0.01)
                
                if not collision_detected:
                    print(f"Successfully reached target with {arm_choice} arm.")
                
                time.sleep(1.0) # 到达后停顿

if __name__ == '__main__':
    sim = ArmSimulation('d:/iatrobot/double_arm_robot/arm_simulation/arm_world.xml')
    sim.run()
