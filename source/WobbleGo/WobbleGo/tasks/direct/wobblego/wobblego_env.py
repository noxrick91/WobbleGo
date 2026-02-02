from __future__ import annotations

import math
import torch
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

    def _init_domain_randomization(self):
        """初始化域随机化参数存储"""
        dr_cfg = self.cfg.domain_randomization
        
        # 每个环境的扭矩缩放因子
        self._torque_scale = torch.ones(self.num_envs, 1, device=self.device)
        
        # 存储原始物理属性（用于重置时参考）
        self._default_body_masses = None
        self._default_body_inertias = None
        
        # 外部扰动状态（模拟线缆干扰）
        self._disturbance_torque = torch.zeros(self.num_envs, device=self.device)
        self._disturbance_step_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        
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
        
        # 更新外部扰动（模拟线缆干扰）
        if dr_cfg.enable and dr_cfg.enable_external_disturbance:
            self._update_external_disturbance()
    
    def _update_external_disturbance(self):
        """更新外部扰动力矩（模拟线缆拉扯）"""
        dr_cfg = self.cfg.domain_randomization
        
        # 增加步数计数器
        self._disturbance_step_counter += 1
        
        # 检查是否需要更新扰动（基于时间间隔）
        should_update = self._disturbance_step_counter >= dr_cfg.disturbance_change_interval
        
        if should_update.any():
            # 重置计数器
            self._disturbance_step_counter[should_update] = 0
            
            # 以一定概率改变扰动方向
            change_direction = torch.rand(self.num_envs, device=self.device) > dr_cfg.disturbance_persistence
            change_mask = should_update & change_direction
            
            if change_mask.any():
                # 生成新的随机扰动（正态分布）
                new_disturbance = torch.randn(size=(int(change_mask.sum()),), device=self.device) * dr_cfg.arm_disturbance_torque
                self._disturbance_torque[change_mask] = new_disturbance

    def _apply_action(self) -> None:
        # 将归一化动作转换为力矩并应用到飞轮
        # 考虑域随机化的扭矩缩放和电机方向
        scaled_torque = self.actions * self.cfg.action_scale * self._torque_scale
        self.robot.set_joint_effort_target(
            scaled_torque, 
            joint_ids=self._flywheel_dof_idx
        )
        
        # 应用外部扰动力矩到摆杆（模拟线缆干扰）
        dr_cfg = self.cfg.domain_randomization
        if dr_cfg.enable and dr_cfg.enable_external_disturbance:
            disturbance = self._disturbance_torque.unsqueeze(-1)
            self.robot.set_joint_effort_target(
                disturbance,
                joint_ids=self._pendulum_dof_idx
            )

    def _get_observations(self) -> dict:
        """观测空间：[cos(θ), sin(θ), 摆杆角速度, 飞轮角速度]
        
        支持观测噪声（域随机化)
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
            flywheel_vel / 50.0,  # 归一化
        ], dim=-1)
        
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
            self.cfg.rew_scale_upright,
            self.cfg.rew_scale_velocity,
            self.cfg.rew_scale_effort,
            self.cfg.rew_scale_swing_energy,
            self.cfg.rew_scale_progress,
        )
        
        # 更新上一步角度
        self._prev_pole_angle = pole_angle.clone()
        
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
        
        # === 起摆模式：随机初始状态 ===

        # 随机初始角度偏移（在下垂位置附近）
        angle_offset = sample_uniform(
            swing_cfg.initial_angle_offset[0],
            swing_cfg.initial_angle_offset[1],
            (num_envs,),
            str(joint_pos.device),
        )
        joint_pos[:, self._pendulum_dof_idx[0]] += angle_offset
        
        # 随机初始角速度（关键：实现左右摆动）
        if swing_cfg.random_direction:
            # 随机方向的角速度
            initial_vel = sample_uniform(
                swing_cfg.initial_velocity_range[0],
                swing_cfg.initial_velocity_range[1],
                (num_envs,),
                str(joint_vel.device),
            )
        else:
            # 随机幅度，但方向与角度偏移相关（更自然的摆动）
            vel_magnitude = sample_uniform(
                0.0,
                max(abs(swing_cfg.initial_velocity_range[0]), 
                    abs(swing_cfg.initial_velocity_range[1])),
                (num_envs,),
                str(joint_vel.device),
            )
            # 角速度方向与位置偏移相反（模拟摆动）
            initial_vel = -torch.sign(angle_offset) * vel_magnitude
        
        joint_vel[:, self._pendulum_dof_idx[0]] = initial_vel
        
        # 飞轮初始角速度
        flywheel_vel = sample_uniform(
            swing_cfg.flywheel_initial_velocity[0],
            swing_cfg.flywheel_initial_velocity[1],
            (num_envs,),
            str(joint_vel.device),
        )
        joint_vel[:, self._flywheel_dof_idx[0]] = flywheel_vel

        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel
        
        # 重置上一步角度记录
        self._prev_pole_angle[env_ids] = joint_pos[:, self._pendulum_dof_idx[0]]

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        
        # 域随机化：重置时重新随机化参数
        if self.cfg.domain_randomization.enable:
            self._randomize_domain_params(env_ids)
            
            # 重置外部扰动状态
            dr_cfg = self.cfg.domain_randomization
            env_ids_tensor = torch.tensor(env_ids, dtype=torch.long, device=self.device) if not isinstance(env_ids, torch.Tensor) else env_ids
            
            if dr_cfg.enable_external_disturbance:
                # 随机初始化扰动
                self._disturbance_torque[env_ids_tensor] = torch.randn(len(env_ids), device=self.device) * dr_cfg.arm_disturbance_torque
                self._disturbance_step_counter[env_ids_tensor] = 0


@torch.jit.script
def compute_reward_with_swing(
    pole_angle: torch.Tensor,
    pole_vel: torch.Tensor,
    flywheel_vel: torch.Tensor,
    prev_pole_angle: torch.Tensor,
    actions: torch.Tensor,
    rew_scale_upright: float,
    rew_scale_velocity: float,
    rew_scale_effort: float,
    rew_scale_swing_energy: float,
    rew_scale_progress: float,
) -> torch.Tensor:
    """支持摆动起摆和平衡
    
    角度约定：
    - 角度 0 = 下垂（向下）
    - 角度 ±π = 竖直向上（目标）
    
    奖励组成：
    1. 竖直奖励：接近目标角度的奖励
    2. 进度奖励：向上摆动的奖励（鼓励起摆）
    3. 能量奖励：摆动能量积累的奖励
    4. 速度惩罚：接近顶部时的稳定性惩罚
    5. 能耗惩罚：动作能耗惩罚
    """
    target_angle = math.pi  # 目标：竖直向上 (180°)
    
    # 计算距离目标的角度（处理周期性）
    angle_error = torch.abs(pole_angle - target_angle)
    # 处理 -π 和 +π 都是目标的情况
    angle_error = torch.minimum(angle_error, 2 * math.pi - angle_error)
    
    prev_angle_error = torch.abs(prev_pole_angle - target_angle)
    prev_angle_error = torch.minimum(prev_angle_error, 2 * math.pi - prev_angle_error)
    
    # === 1. 竖直奖励 ===
    # 使用 cos 奖励 + 指数奖励，接近垂直时奖励急剧增加
    upright_reward = torch.cos(angle_error) + torch.exp(-5.0 * angle_error ** 2)
    
    # 精确平衡奖励：角度误差 < 0.1 rad (~6°) 时额外奖励
    precise_balance = (angle_error < 0.1).float() * 2.0
    
    # === 2. 进度奖励（鼓励向上摆）===
    # 如果角度误差减小，给予正奖励
    progress = prev_angle_error - angle_error
    progress_reward = torch.clamp(progress, -0.5, 0.5)  # 限制范围防止过大
    
    # === 3. 能量奖励（鼓励摆动积累能量）===
    # 高度 h = 1 - cos(θ)，速度能量 = 0.5 * v^2
    # 总机械能 = 势能 + 动能
    height = 1.0 - torch.cos(pole_angle)  # 相对高度 [0, 2]
    kinetic_energy = 0.5 * (pole_vel / 10.0) ** 2  # 归一化动能
    mechanical_energy = height + kinetic_energy
    
    # 只在起摆阶段（角度误差大）给能量奖励
    in_swing_phase = angle_error > 0.5  # ~30° 偏离目标
    energy_reward = in_swing_phase.float() * mechanical_energy
    
    # === 4. 速度惩罚（接近顶部时鼓励稳定）===
    near_top = angle_error < 0.3  # ~17°
    velocity_penalty = near_top.float() * pole_vel ** 2
    
    # === 5. 能耗惩罚 ===
    effort_penalty = torch.sum(actions ** 2, dim=-1)
    
    # === 6. 飞轮速度惩罚（防止飞轮转速过高）===
    flywheel_speed_penalty = torch.clamp(torch.abs(flywheel_vel) / 80.0 - 1.0, min=0.0) ** 2
    
    # === 总奖励 ===
    reward = (
        rew_scale_upright * (upright_reward + precise_balance) +
        rew_scale_progress * progress_reward +
        rew_scale_swing_energy * energy_reward +
        rew_scale_velocity * velocity_penalty +
        rew_scale_effort * effort_penalty +
        -0.1 * flywheel_speed_penalty  # 飞轮超速惩罚
    )
    
    return reward