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
    # 飞轮质量缩放范围
    flywheel_mass_scale: tuple[float, float] = (1.2, 2.0)
    # 飞轮惯量缩放范围
    flywheel_inertia_scale: tuple[float, float] = (1.2, 2.0)
    
    # === 电机参数随机化 ===
    # 电机最大扭矩缩放范围
    motor_torque_scale: tuple[float, float] = (0.8, 1.2)
    # 电机速度限制缩放范围
    motor_velocity_scale: tuple[float, float] = (0.8, 1.2)
    # 电机阻尼随机化范围
    motor_damping_range: tuple[float, float] = (0.001, 0.002)
    # 电机质量缩放范围
    motor_mass_scale: tuple[float, float] = (0.8, 1.6)
    
    # === 摆臂参数随机化 ===
    # 摆臂质量缩放范围
    arm_mass_scale: tuple[float, float] = (0.8, 1.2)  # ±50%
    # 摆臂惯量缩放范围
    arm_inertia_scale: tuple[float, float] = (0.8, 1.2)  # ±50%
    # 摆臂关节阻尼范围
    arm_damping_range: tuple[float, float] = (0.001, 0.002)  
    
    
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
    
    # === 从顶部附近开始的配置（大幅增加平衡训练样本）===
    # 从顶部附近初始化的 episode 比例 (0.0~1.0)
    # 0.3 = 30% 的 episode 从顶部附近开始，70% 从底部起摆
    top_start_ratio: float = 0.3
    # 顶部初始化时的角度偏移范围 [rad]（相对于 π）
    top_angle_offset: tuple[float, float] = (-0.3, 0.3)
    # 顶部初始化时的角速度范围 [rad/s]
    top_velocity_range: tuple[float, float] = (-2.0, 2.0)
    # 顶部初始化时的飞轮角速度范围 [rad/s]
    top_flywheel_velocity: tuple[float, float] = (-10.0, 10.0)


@configclass
class WobbleGoEnvCfg(DirectRLEnvCfg):
    """飞轮倒立摆环境配置"""
    
    # 环境基础配置
    decimation = 2
    episode_length_s = 15.0
    action_space = 1  # 飞轮力矩
    observation_space = 4  # [cos(θ), sin(θ), 摆杆角速度, 飞轮角速度]
    state_space = 0

    # 仿真配置
    sim: SimulationCfg = SimulationCfg(dt=1 / 200, render_interval=decimation)

    # 机器人配置
    robot_cfg: ArticulationCfg = replace(WOBBLEGO_CFG, prim_path="/World/envs/env_.*/Robot")

    # 场景配置
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=1.0, replicate_physics=True)

    # 关节名称
    pendulum_dof_name = "base_arm_joint"  # 摆杆关节
    flywheel_dof_name = "motor_wheel_joint"  # 飞轮关节

    # 动作缩放
    action_scale = 0.35  # 电机最大扭矩 [Nm]
    
    # 奖励系数
    rew_scale_upright = 2.5       # 竖直奖励
    rew_scale_velocity = -0.25    # 摆杆角速度惩罚（接近顶部时）
    rew_scale_effort = -0.003     # 能耗惩罚
    rew_scale_swing_energy = 0.4  # 摆动能量奖励
    rew_scale_progress = 0.8      # 进度奖励
    rew_scale_stable = 3.0        # 稳定平衡奖励
    rew_scale_action_rate = -0.05 # 动作变化率惩罚
    rew_scale_flywheel_vel = -0.003   # 飞轮速度持续惩罚
    rew_scale_flywheel_overspeed = -0.3  # 飞轮超速惩罚（线性惩罚的系数）
    flywheel_speed_threshold = 50.0  # 飞轮速度惩罚阈值
    
    # 域随机化配置
    domain_randomization: DomainRandomizationCfg = DomainRandomizationCfg()
    
    # 起摆配置
    swing_up: SwingUpCfg = SwingUpCfg()
    
    # 观测延迟（模拟真实系统的 IMU 采样 + 通信 + 推理延迟）
    observation_delay_steps: int = 2