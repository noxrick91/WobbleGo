from __future__ import annotations

import math
import torch
from collections import deque
from collections.abc import Sequence
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform

from .wobblego_env_cfg import WobbleGoEnvCfg

if TYPE_CHECKING:
    from .wobblego_env_cfg import DomainRandomizationCfg, SwingUpCfg


class WobbleGoEnv(DirectRLEnv):
    """飞轮倒立摆环境"""
    
    cfg: WobbleGoEnvCfg

    def __init__(self, cfg: WobbleGoEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # 获取关节索引
        self._pendulum_dof_idx, _ = self.robot.find_joints(self.cfg.pendulum_dof_name)
        self._flywheel_dof_idx, _ = self.robot.find_joints(self.cfg.flywheel_dof_name)

        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        
        # 域随机化：每个环境的参数缩放因子
        self._init_domain_randomization()
        
        # 记录上一步的角度用于计算进度奖励
        self._prev_pole_angle = torch.zeros(self.num_envs, device=self.device)
        
        # 记录上一步动作，用于计算动作平滑度惩罚
        self._prev_actions = torch.zeros(self.num_envs, int(self.cfg.action_space), device=self.device)
        
        # 观测延迟缓冲区（模拟真实系统的传感器和通信延迟）
        self._obs_delay_buffer: deque[torch.Tensor] = deque(
            maxlen=max(self.cfg.observation_delay_steps, 1)
        )

    def _init_domain_randomization(self):
        """初始化域随机化参数存储"""
        dr_cfg = self.cfg.domain_randomization
        
        # 每个环境的扭矩缩放因子
        self._torque_scale = torch.ones(self.num_envs, 1, device=self.device)
        
        # 存储原始物理属性（用于重置时参考）
        self._default_body_masses = None
        self._default_body_inertias = None
        
        
        if dr_cfg.enable:
            # 初始化随机化参数
            self._randomize_domain_params(list(range(self.num_envs)))

    def _randomize_domain_params(self, env_ids: Sequence[int]):
        """为指定环境随机化域参数"""
        dr_cfg = self.cfg.domain_randomization
        
        if not dr_cfg.enable:
            return
            
        num_envs = len(env_ids)
        device = self.device
        
        # 将 env_ids 转换为 tensor（Isaac Lab API 需要 tensor）
        if not isinstance(env_ids, torch.Tensor):
            env_ids_tensor = torch.tensor(env_ids, dtype=torch.long, device=device)
        else:
            env_ids_tensor = env_ids
        
        # === 电机扭矩随机化 ===
        torque_scale = sample_uniform(
            dr_cfg.motor_torque_scale[0],
            dr_cfg.motor_torque_scale[1],
            (num_envs, 1),
            device,
        )
        self._torque_scale[env_ids] = torque_scale
        
        # === 物理属性随机化（质量和惯量）===

        # 随机化摆臂关节阻尼
        arm_damping = sample_uniform(
            dr_cfg.arm_damping_range[0],
            dr_cfg.arm_damping_range[1],
            (num_envs,),
            device,
        )
        
        # 随机化电机阻尼
        motor_damping = sample_uniform(
            dr_cfg.motor_damping_range[0],
            dr_cfg.motor_damping_range[1],
            (num_envs,),
            device,
        )
        
        # 应用阻尼随机化
        # 只创建需要更新的环境的阻尼张量（形状为 [num_envs, num_joints]）
        num_joints = self.robot.data.default_joint_damping.shape[1]
        joint_damping = self.robot.data.default_joint_damping[env_ids_tensor].clone()
        
        # 更新对应关节的阻尼值
        joint_damping[:, self._pendulum_dof_idx[0]] = arm_damping
        joint_damping[:, self._flywheel_dof_idx[0]] = motor_damping
        
        # 写入新的阻尼值（使用 tensor 类型的 env_ids）
        self.robot.write_joint_damping_to_sim(joint_damping, env_ids=env_ids_tensor)
        
        # === 刚体质量随机化 ===
        try:
            flywheel_body_ids = []
            try:
                wheel_ids = self.robot.find_bodies("wheel")[0]
                flywheel_body_ids.extend(list(wheel_ids))
            except (ValueError, IndexError):
                pass
            try:
                weight_ids = self.robot.find_bodies("weight")[0]
                flywheel_body_ids.extend(list(weight_ids))
            except (ValueError, IndexError):
                pass
            
            arm_body_ids = self.robot.find_bodies("arm")[0]
            
            # 获取默认质量
            if self._default_body_masses is None:
                self._default_body_masses = self.robot.root_physx_view.get_masses().clone()
            
            # 只获取需要更新的环境的质量（形状为 [num_envs, num_bodies]）
            masses = self._default_body_masses[env_ids_tensor].clone()
            
            # 飞轮质量随机化
            if len(flywheel_body_ids) > 0:
                flywheel_mass_scale = sample_uniform(
                    dr_cfg.flywheel_mass_scale[0],
                    dr_cfg.flywheel_mass_scale[1],
                    (num_envs,),
                    device,
                )
                for body_id in flywheel_body_ids:
                    masses[:, body_id] *= flywheel_mass_scale
            
            # 摆臂质量随机化
            if len(arm_body_ids) > 0:
                arm_mass_scale = sample_uniform(
                    dr_cfg.arm_mass_scale[0],
                    dr_cfg.arm_mass_scale[1],
                    (num_envs,),
                    device,
                )
                for body_id in arm_body_ids:
                    masses[:, body_id] *= arm_mass_scale
            
            # 电机质量随机化
            motor_body_ids = []
            try:
                motor_ids = self.robot.find_bodies("motor")[0]
                motor_body_ids.extend(list(motor_ids))
            except (ValueError, IndexError):
                pass
            
            if len(motor_body_ids) > 0:
                motor_mass_scale = sample_uniform(
                    dr_cfg.motor_mass_scale[0],
                    dr_cfg.motor_mass_scale[1],
                    (num_envs,),
                    device,
                )
                for body_id in motor_body_ids:
                    masses[:, body_id] *= motor_mass_scale
            
            # 写入新质量（使用 tensor 类型的 env_ids）
            self.robot.root_physx_view.set_masses(masses, env_ids_tensor)
            
        except (ValueError, AttributeError, RuntimeError):
            # 如果找不到对应的刚体或 API 不支持，跳过质量随机化
            pass
        
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["robot"] = self.robot
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()
        
        # 添加动作噪声（域随机化）
        dr_cfg = self.cfg.domain_randomization
        if dr_cfg.enable and dr_cfg.action_noise_std > 0:
            action_noise = torch.randn_like(self.actions) * dr_cfg.action_noise_std
            self.actions = self.actions + action_noise
        
            

    def _apply_action(self) -> None:
        # 将归一化动作转换为力矩并应用到飞轮
        scaled_torque = self.actions * self.cfg.action_scale * self._torque_scale
        
        flywheel_vel = self.joint_vel[:, self._flywheel_dof_idx[0]:self._flywheel_dof_idx[0]+1]
        vel_limit = 90.0
        # 当飞轮超速且力矩方向与速度方向相同时，置零力矩
        same_direction = (flywheel_vel * scaled_torque) > 0
        over_limit = torch.abs(flywheel_vel) > vel_limit
        scaled_torque = torch.where(same_direction & over_limit, torch.zeros_like(scaled_torque), scaled_torque)
        
        self.robot.set_joint_effort_target(
            scaled_torque, 
            joint_ids=self._flywheel_dof_idx
        )
        

    def _get_observations(self) -> dict:
        """观测空间：[cos(θ), sin(θ), 摆杆角速度, 飞轮角速度]
        """
        pole_angle = self.joint_pos[:, self._pendulum_dof_idx[0]]
        pole_vel = self.joint_vel[:, self._pendulum_dof_idx[0]]
        flywheel_vel = self.joint_vel[:, self._flywheel_dof_idx[0]]
        
        # 添加观测噪声
        dr_cfg = self.cfg.domain_randomization
        if dr_cfg.enable:
            # 角度噪声
            if dr_cfg.obs_noise_angle > 0:
                angle_noise = torch.randn_like(pole_angle) * dr_cfg.obs_noise_angle
                pole_angle = pole_angle + angle_noise
            
            # 角速度噪声
            if dr_cfg.obs_noise_velocity > 0:
                vel_noise = torch.randn_like(pole_vel) * dr_cfg.obs_noise_velocity
                pole_vel = pole_vel + vel_noise
                flywheel_vel_noise = torch.randn_like(flywheel_vel) * dr_cfg.obs_noise_velocity
                flywheel_vel = flywheel_vel + flywheel_vel_noise
        
        obs = torch.stack([
            torch.cos(pole_angle),
            torch.sin(pole_angle),
            pole_vel / 10.0,  # 归一化
            torch.clamp(flywheel_vel / 90.0, -1.0, 1.0),  # 归一化并裁剪到[-1,1]
        ], dim=-1)
        
        # 应用观测延迟（模拟真实系统的传感器+通信延迟）
        delay = self.cfg.observation_delay_steps
        if delay > 0:
            # 如果缓冲区已满，取出最早的观测作为延迟观测
            if len(self._obs_delay_buffer) >= delay:
                delayed_obs = self._obs_delay_buffer[0].clone()
            else:
                # 缓冲区未满（刚 reset），使用当前观测
                delayed_obs = obs.clone()
            self._obs_delay_buffer.append(obs.clone())
            return {"policy": delayed_obs}
        
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        """奖励函数 - 支持起摆和平衡"""
        pole_angle = self.joint_pos[:, self._pendulum_dof_idx[0]]
        pole_vel = self.joint_vel[:, self._pendulum_dof_idx[0]]
        flywheel_vel = self.joint_vel[:, self._flywheel_dof_idx[0]]
        
        
        reward = compute_reward_with_swing(
            pole_angle,
            pole_vel,
            flywheel_vel,
            self._prev_pole_angle,
            self.actions,
            self._prev_actions,
            self.cfg.rew_scale_upright,
            self.cfg.rew_scale_velocity,
            self.cfg.rew_scale_effort,
            self.cfg.rew_scale_swing_energy,
            self.cfg.rew_scale_progress,
            self.cfg.rew_scale_stable,
            self.cfg.rew_scale_action_rate,
            self.cfg.rew_scale_flywheel_vel,
            self.cfg.rew_scale_flywheel_overspeed,
            self.cfg.flywheel_speed_threshold,
        )
        
        # 更新上一步角度和动作
        self._prev_pole_angle = pole_angle.clone()
        self._prev_actions = self.actions.clone()
        
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self.joint_pos = self.robot.data.joint_pos
        self.joint_vel = self.robot.data.joint_vel
        
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        out_of_bounds = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        
        return out_of_bounds, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES.tolist()
        super()._reset_idx(env_ids)

        num_envs = len(env_ids)
        swing_cfg = self.cfg.swing_up
        
        joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        joint_vel = torch.zeros_like(joint_pos)
        
        # === 决定哪些环境从顶部附近开始，哪些从底部起摆 ===
        start_from_top = torch.rand(num_envs, device=self.device) < swing_cfg.top_start_ratio
        start_from_bottom = ~start_from_top
        
        # === 底部起摆模式（原有逻辑）===
        if start_from_bottom.any():
            n_bottom = int(start_from_bottom.sum().item())
            
            angle_offset = sample_uniform(
                swing_cfg.initial_angle_offset[0],
                swing_cfg.initial_angle_offset[1],
                (n_bottom,),
                str(joint_pos.device),
            )
            joint_pos[start_from_bottom, self._pendulum_dof_idx[0]] += angle_offset
            
            if swing_cfg.random_direction:
                initial_vel = sample_uniform(
                    swing_cfg.initial_velocity_range[0],
                    swing_cfg.initial_velocity_range[1],
                    (n_bottom,),
                    str(joint_vel.device),
                )
            else:
                vel_magnitude = sample_uniform(
                    0.0,
                    max(abs(swing_cfg.initial_velocity_range[0]), 
                        abs(swing_cfg.initial_velocity_range[1])),
                    (n_bottom,),
                    str(joint_vel.device),
                )
                initial_vel = -torch.sign(angle_offset) * vel_magnitude
            
            joint_vel[start_from_bottom, self._pendulum_dof_idx[0]] = initial_vel
            
            flywheel_vel_bottom = sample_uniform(
                swing_cfg.flywheel_initial_velocity[0],
                swing_cfg.flywheel_initial_velocity[1],
                (n_bottom,),
                str(joint_vel.device),
            )
            joint_vel[start_from_bottom, self._flywheel_dof_idx[0]] = flywheel_vel_bottom
        
        # === 顶部平衡模式（新增！大幅增加平衡训练样本）===
        if start_from_top.any():
            n_top = int(start_from_top.sum().item())
            
            # 在 π 附近随机偏移（随机选择 +π 或 -π 侧）
            top_offset = sample_uniform(
                swing_cfg.top_angle_offset[0],
                swing_cfg.top_angle_offset[1],
                (n_top,),
                str(joint_pos.device),
            )
            # 随机选择从 +π 或 -π 侧开始
            side = (torch.rand(n_top, device=self.device) > 0.5).float() * 2.0 - 1.0
            joint_pos[start_from_top, self._pendulum_dof_idx[0]] = side * math.pi + top_offset
            
            # 顶部附近的小角速度扰动
            top_vel = sample_uniform(
                swing_cfg.top_velocity_range[0],
                swing_cfg.top_velocity_range[1],
                (n_top,),
                str(joint_vel.device),
            )
            joint_vel[start_from_top, self._pendulum_dof_idx[0]] = top_vel
            
            # 飞轮初始角速度（可能非零，模拟刚起摆完的状态）
            flywheel_vel_top = sample_uniform(
                swing_cfg.top_flywheel_velocity[0],
                swing_cfg.top_flywheel_velocity[1],
                (n_top,),
                str(joint_vel.device),
            )
            joint_vel[start_from_top, self._flywheel_dof_idx[0]] = flywheel_vel_top

        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel
        
        # 重置上一步角度和动作记录
        self._prev_pole_angle[env_ids] = joint_pos[:, self._pendulum_dof_idx[0]]
        self._prev_actions[env_ids] = 0.0

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        
        # 域随机化：重置时重新随机化参数
        if self.cfg.domain_randomization.enable:
            self._randomize_domain_params(env_ids)
            
            # 重置外部扰动状态
            dr_cfg = self.cfg.domain_randomization
            env_ids_tensor = torch.tensor(env_ids, dtype=torch.long, device=self.device) if not isinstance(env_ids, torch.Tensor) else env_ids
            


@torch.jit.script
def compute_reward_with_swing(
    pole_angle: torch.Tensor,
    pole_vel: torch.Tensor,
    flywheel_vel: torch.Tensor,
    prev_pole_angle: torch.Tensor,
    actions: torch.Tensor,
    prev_actions: torch.Tensor,
    rew_scale_upright: float,
    rew_scale_velocity: float,
    rew_scale_effort: float,
    rew_scale_swing_energy: float,
    rew_scale_progress: float,
    rew_scale_stable: float,
    rew_scale_action_rate: float,
    rew_scale_flywheel_vel: float,
    rew_scale_flywheel_overspeed: float,
    flywheel_speed_threshold: float,
) -> torch.Tensor:
    """支持摆动起摆和平衡
    
    角度约定：
    - 角度 0 = 下垂（向下）
    - 角度 ±π = 竖直向上（目标）
    """
    target_angle = math.pi  # 目标：竖直向上 (180°)
    
    # 计算距离目标的角度（处理周期性）
    angle_error = torch.abs(pole_angle - target_angle)
    angle_error = torch.minimum(angle_error, 2 * math.pi - angle_error)
    
    prev_angle_error = torch.abs(prev_pole_angle - target_angle)
    prev_angle_error = torch.minimum(prev_angle_error, 2 * math.pi - prev_angle_error)
    
    # === 1. 竖直奖励 ===
    upright_reward = torch.cos(angle_error) + torch.exp(-5.0 * angle_error ** 2)
    
    # 精确平衡奖励：角度误差 < 0.1 rad (~6°) 时额外奖励
    precise_balance = (angle_error < 0.1).float() * 2.0
    
    # === 2. 进度奖励（鼓励向上摆）===
    progress = prev_angle_error - angle_error
    progress_reward = torch.clamp(progress, -0.5, 0.5)
    
    # === 3. 能量奖励（鼓励摆动积累能量）===
    height = 1.0 - torch.cos(pole_angle)  # 相对高度 [0, 2]
    kinetic_energy = 0.5 * (pole_vel / 10.0) ** 2
    mechanical_energy = height + kinetic_energy
    
    # 只在起摆阶段（远离顶部）给能量奖励
    in_swing_phase = angle_error > 0.5  # ~30° 偏离目标
    energy_reward = in_swing_phase.float() * mechanical_energy
    
    # === 4. 速度惩罚（接近顶部时鼓励减速）===
    near_top = (angle_error < 0.5).float()
    proximity_weight = torch.exp(-3.0 * angle_error)
    velocity_penalty = near_top * proximity_weight * pole_vel ** 2
    
    # === 5. 能耗惩罚 ===
    effort_penalty = torch.sum(actions ** 2, dim=-1)
    
    # === 6. 动作平滑度惩罚（抑制抖动，改善 Sim2Real）===
    action_rate_penalty = torch.sum((actions - prev_actions) ** 2, dim=-1)
    
    # === 7. 稳定平衡奖励（同时要求角度小且速度低）===
    stable_reward = torch.exp(-10.0 * angle_error ** 2) * torch.exp(-2.0 * pole_vel ** 2)
    
    # === 8. 飞轮速度持续惩罚 ===
    # 二次惩罚，在正常速度范围（0~50）内提供平滑梯度
    flywheel_vel_penalty = flywheel_vel ** 2
    
    # === 9. 飞轮超速惩罚（线性！）===
    # 用线性代替二次：常数梯度，不会在极端速度时爆炸
    # 二次在 vel=90 时产生 (90-50)²=1600 → 完全淹没正向奖励
    # 线性在 vel=90 时产生 (90-50)=40   → 可控且梯度恒定
    overspeed = torch.clamp(torch.abs(flywheel_vel) - flywheel_speed_threshold, min=0.0)
    flywheel_overspeed_penalty = overspeed  # 线性，不是 overspeed²
    
    # === 总奖励 ===
    reward = (
        rew_scale_upright * (upright_reward + precise_balance) +
        rew_scale_progress * progress_reward +
        rew_scale_swing_energy * energy_reward +
        rew_scale_velocity * velocity_penalty +
        rew_scale_effort * effort_penalty +
        rew_scale_action_rate * action_rate_penalty +
        rew_scale_stable * stable_reward +
        rew_scale_flywheel_vel * flywheel_vel_penalty +
        rew_scale_flywheel_overspeed * flywheel_overspeed_penalty
    )
    
    # 裁剪总奖励：防止极端状态（随机探索时飞轮失控）导致价值函数梯度爆炸
    # 正值不裁剪（保留完整的平衡奖励梯度），负值裁剪到 -10
    # 这样极端负奖励不会主导训练，但正常范围内的惩罚梯度完好保留
    reward = torch.clamp(reward, min=-10.0)
    
    return reward