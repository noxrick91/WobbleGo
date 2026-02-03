# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

from WobbleGo.robots.wobble_go import WOBBLEGO_CFG

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from dataclasses import replace


@configclass
class DomainRandomizationCfg:
    """域随机化配置"""
    
    # 是否启用域随机化
    enable: bool = True
    
    # === 飞轮参数随机化 ===
    # 飞轮质量缩放范围 [min, max]（相对于原始质量的比例）
    # 增大范围以覆盖不同惯量场景（惯量 ∝ 质量）
    flywheel_mass_scale: tuple[float, float] = (0.8, 1.2)
    # 飞轮惯量缩放范围（相对于原始惯量的比例）
    flywheel_inertia_scale: tuple[float, float] = (0.8, 1.2)
    
    # === 电机参数随机化 ===
    # 电机最大扭矩缩放范围（相对于 action_scale 的比例）
    # 真实电机最大力矩 ≈ 0.5 Nm，需要覆盖这个范围
    motor_torque_scale: tuple[float, float] = (0.8, 1.2)
    # 电机速度限制缩放范围（相对于原始速度限制的比例）
    motor_velocity_scale: tuple[float, float] = (0.8, 1.2)
    # 电机阻尼随机化范围
    motor_damping_range: tuple[float, float] = (0.002, 0.025)
    
    # === 摆臂参数随机化 ===
    # 摆臂质量缩放范围（相对于原始质量的比例）
    arm_mass_scale: tuple[float, float] = (0.8, 1.5)  # ±50%
    # 摆臂惯量缩放范围（相对于原始惯量的比例）
    arm_inertia_scale: tuple[float, float] = (0.8, 1.5)  # ±50%
    # 摆臂关节阻尼范围（模拟轴承摩擦+线缆阻力，增大范围）
    arm_damping_range: tuple[float, float] = (0.01, 0.05)  # 线缆会增加阻力
    
    # === 外部扰动（模拟线缆干扰）===
    # 是否启用外部扰动
    enable_external_disturbance: bool = True
    # 摆杆外部扰动力矩范围 [Nm]（模拟线缆拉扯）
    arm_disturbance_torque: float = 0.02  # 随机扰动力矩幅值
    # 扰动变化频率（每多少步更新一次扰动方向）
    disturbance_change_interval: int = 10  # 每10步（约0.17秒@60Hz）可能改变扰动
    # 扰动持续概率（扰动方向保持不变的概率）
    disturbance_persistence: float = 0.9  # 90%概率保持当前扰动方向
    
    # === 观测噪声（增加噪声水平）===
    # 角度观测噪声标准差 [rad]
    obs_noise_angle: float = 0.02  # 约1.1度
    # 角速度观测噪声标准差 [rad/s]
    obs_noise_velocity: float = 0.1
    
    # === 动作噪声 ===
    # 动作噪声标准差（模拟电机控制不精确）
    action_noise_std: float = 0.05


@configclass
class SwingUpCfg:
    """起摆模式配置"""
    
    # 是否启用摆动起摆
    enable_swing: bool = True
    
    # 初始角速度范围 [rad/s]（给摆杆一个初始摆动）
    initial_velocity_range: tuple[float, float] = (-3.0, 3.0)
    
    # 是否随机选择摆动方向（True: 随机方向, False: 双向都有）
    random_direction: bool = True
    
    # 初始角度偏移范围 [rad]（在下垂位置附近）
    initial_angle_offset: tuple[float, float] = (-0.3, 0.3)
    
    # 飞轮初始角速度范围 [rad/s]
    flywheel_initial_velocity: tuple[float, float] = (-5.0, 5.0)


@configclass
class WobbleGoEnvCfg(DirectRLEnvCfg):
    """飞轮倒立摆环境配置"""
    
    # 环境基础配置
    decimation = 2
    episode_length_s = 15.0  # 增加到15秒，给起摆更多时间
    action_space = 1  # 飞轮力矩
    observation_space = 4  # [cos(θ), sin(θ), 摆杆角速度, 飞轮角速度]
    state_space = 0

    # 仿真配置
    sim: SimulationCfg = SimulationCfg(dt=1 / 300, render_interval=decimation)

    # 机器人配置
    robot_cfg: ArticulationCfg = replace(WOBBLEGO_CFG, prim_path="/World/envs/env_.*/Robot")

    # 场景配置
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=1.0, replicate_physics=True)

    # 关节名称
    pendulum_dof_name = "base_arm_joint"  # 摆杆关节
    flywheel_dof_name = "motor_wheel_joint"  # 飞轮关节

    # 动作缩放（基础值，会被域随机化调整）
    action_scale = 0.50  # 电机最大扭矩 [Nm]
    
    # 奖励系数
    rew_scale_upright = 2.0  # 竖直奖励
    rew_scale_velocity = -0.15  # 角速度惩罚（稍微减小，允许起摆时有更大速度）
    rew_scale_effort = -0.003  # 能耗惩罚（减小，鼓励更积极的起摆）
    rew_scale_swing_energy = 0.5  # 摆动能量奖励（鼓励能量积累）
    rew_scale_progress = 1.0  # 进度奖励（鼓励向上摆）
    
    # 域随机化配置
    domain_randomization: DomainRandomizationCfg = DomainRandomizationCfg()
    
    # 起摆配置
    swing_up: SwingUpCfg = SwingUpCfg()
    
    # 旧的初始状态配置（保留兼容性，但优先使用 swing_up 配置）
    initial_pole_angle_range = [-0.2, 0.2]  # 随机偏移 [rad]
