import cawlib as cw
from src import imu
import time
import math
import torch
import numpy as np
from typing import Optional

# ==================== 配置参数 ====================
K_T = 0.353
ACTION_SCALE = 0.5  # 降低增益减少超调，训练范围 0.24~0.50 Nm
CONTROL_FREQ = 150.0  # 仿真: dt=1/200, decimation=2 → 100Hz
POLE_VEL_SCALE = 10.0
FLYWHEEL_VEL_SCALE = 50.0
MAX_IQ = 1.5
MAX_TORQUE = K_T * MAX_IQ

# 方向补偿（仿真已调整为与真实环境一致，无需补偿）
TORQUE_DIRECTION = -1.0       # 仿真与真实环境方向一致
FLYWHEEL_VEL_DIRECTION = +1.0

# IMU方向（这个配置能起摆）
POLE_ANGLE_DIRECTION = -1.0
POLE_VEL_DIRECTION = +1.0

# 飞轮速度安全阈值
MAX_FLYWHEEL_SPEED = 120.0  # rad/s，超过此值停止控制


class PolicyWrapper:
    def __init__(self, model_path: str, device: str = "cpu"):
        self.device = torch.device(device)
        self.actor = None
        self.input_dim = 4
        self.load_model(model_path)
        
    def load_model(self, model_path: str):
        print(f"加载模型: {model_path}")
        checkpoint = torch.load(model_path, map_location=self.device)
        model_state = checkpoint.get('model_state_dict', checkpoint)
        
        actor_state = {}
        for key, value in model_state.items():
            if key.startswith('actor.'):
                actor_state[key.replace('actor.', '')] = value
        
        layer_dims = []
        layer_idx = 0
        while True:
            weight_key = f"{layer_idx}.weight"
            if weight_key not in actor_state:
                break
            weight = actor_state[weight_key]
            if layer_idx == 0:
                layer_dims.append(weight.shape[1])
            layer_dims.append(weight.shape[0])
            layer_idx += 2
        
        self.input_dim = layer_dims[0]
        print(f"网络结构: {layer_dims}")
        
        modules = []
        for i in range(len(layer_dims) - 1):
            modules.append(torch.nn.Linear(layer_dims[i], layer_dims[i + 1]))
            if i < len(layer_dims) - 2:
                modules.append(torch.nn.ELU())
        
        self.actor = torch.nn.Sequential(*modules)
        
        idx = 0
        for module in self.actor:
            if isinstance(module, torch.nn.Linear):
                module.weight.data = actor_state[f"{idx}.weight"]
                module.bias.data = actor_state[f"{idx}.bias"]
                idx += 2
        
        self.actor.eval()
        print("模型加载成功！")
    
    @torch.no_grad()
    def get_action(self, obs: np.ndarray) -> float:
        if self.actor is None:
            return 0.0
        obs_tensor = torch.tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        action = self.actor(obs_tensor)
        return np.clip(action.squeeze().cpu().item(), -1.0, 1.0)


class InvertedPendulumController:
    def __init__(self, model_path: str):
        self.policy = PolicyWrapper(model_path)
        self.obs = np.zeros(4, dtype=np.float32)
        self.control_enabled = False
        self.safety_stop = False
        
        self.debug_info = {
            'pole_angle_deg': 0.0,
            'pole_vel': 0.0,
            'flywheel_vel': 0.0,
            'action': 0.0,
            'iq': 0.0,
        }
    
    def update_observation(self, imu_obs: np.ndarray, flywheel_vel_rad: float):
        # imu_obs = [cos(θ), sin(θ), pole_vel/10]
        self.obs[0] = imu_obs[0]
        self.obs[1] = imu_obs[1] * POLE_ANGLE_DIRECTION
        self.obs[2] = imu_obs[2] * POLE_VEL_DIRECTION
        # 飞轮速度方向补偿
        self.obs[3] = (flywheel_vel_rad * FLYWHEEL_VEL_DIRECTION) / FLYWHEEL_VEL_SCALE
        
        self.debug_info['pole_angle_deg'] = math.degrees(math.atan2(imu_obs[1], imu_obs[0]))
        self.debug_info['pole_vel'] = imu_obs[2] * POLE_VEL_SCALE
        self.debug_info['flywheel_vel'] = flywheel_vel_rad
        
        if abs(flywheel_vel_rad) > MAX_FLYWHEEL_SPEED:
            if not self.safety_stop:
                print(f"\n飞轮超速! {flywheel_vel_rad:.1f} rad/s > {MAX_FLYWHEEL_SPEED}")
                self.safety_stop = True
    
    def enable_control(self):
        self.control_enabled = True
        self.safety_stop = False
        print("控制已启用！")
    
    def compute_control(self) -> float:
        if not self.control_enabled or self.safety_stop:
            return 0.0
        
        action = self.policy.get_action(self.obs)
        
        # 力矩计算（修正方向）
        torque = action * ACTION_SCALE * TORQUE_DIRECTION
        torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)
        
        iq = torque / K_T
        iq = np.clip(iq, -MAX_IQ, MAX_IQ)
        
        self.debug_info['action'] = action
        self.debug_info['iq'] = iq
        
        return iq


# ==================== 全局变量 ====================
controller: Optional[InvertedPendulumController] = None
latest_imu_obs = np.zeros(3, dtype=np.float32)
imu_data_ready = False


@cw.precision_timer(CONTROL_FREQ)
def update(motor: cw.Motor):
    global latest_imu_obs, imu_data_ready
    
    watch_data = motor.get_watch_data()
    flywheel_vel_rad = watch_data["speed"]
    
    if imu_data_ready and controller:
        controller.update_observation(latest_imu_obs, flywheel_vel_rad)
        iq = controller.compute_control()
        motor.set_torque(iq)
    else:
        motor.set_torque(0.0)


@cw.run_async
def imu_obs_task(imu_reader_local: imu.JY61PReader):
    global latest_imu_obs, imu_data_ready
    
    last_print_time = time.time()
    print_interval = 0.1
    
    while True:
        if imu_reader_local.update():
            obs = imu_reader_local.get_observation()
            latest_imu_obs[:] = obs
            imu_data_ready = True
            
            now = time.time()
            if now - last_print_time >= print_interval:
                if controller:
                    d = controller.debug_info
                    angle = d['pole_angle_deg']
                    
                    if abs(angle) < 20:
                        pos = "下垂"
                    elif abs(angle) > 160:
                        pos = "顶部!"
                    elif angle > 0:
                        pos = "左侧"
                    else:
                        pos = "右侧"
                    
                    status = "停止" if controller.safety_stop else "运行中"
                    
                    obs = controller.obs
                    print(f"{pos:4} | "
                          f"角度:{angle:+7.1f}° | "
                          f"角速度:{d['pole_vel']:+6.2f} | "
                          f"飞轮:{d['flywheel_vel']:+6.1f} | "
                          f"act:{d['action']:+5.2f} | "
                          f"iq:{d['iq']:+5.2f}A | "
                          f"obs:[{obs[0]:+.2f},{obs[1]:+.2f},{obs[2]:+.2f},{obs[3]:+.2f}] | {status}")
                
                last_print_time = now


def stop_flywheel(motor: cw.Motor, timeout: float = 5.0):
    print("\n让飞轮停止...")
    
    start = time.time()
    while time.time() - start < timeout:
        watch_data = motor.get_watch_data()
        speed = watch_data["speed"]
        
        if abs(speed) < 1.0:  # 速度足够小
            print(f"飞轮已停止 (speed={speed:.2f} rad/s)")
            motor.set_torque(0.0)
            return True
        
        # 施加反向力矩减速
        brake_iq = -np.sign(speed) * 0.3  # 0.3A 制动
        motor.set_torque(brake_iq)
        
        print(f"  减速中... speed={speed:+.1f} rad/s, brake_iq={brake_iq:+.2f}A")
        time.sleep(0.2)
    
    motor.set_torque(0.0)
    print("超时，飞轮可能未完全停止")
    return False


if __name__ == "__main__":
    MODEL_PATH = "logs/rsl_rl/flywheel_pendulum_direct/2026-02-02_03-47-50/model_499.pt"
    CAN_INTERFACE = "can0"
    MOTOR_TX_ID = 0x200
    MOTOR_RX_ID = 0x100
    IMU_PORT = "/dev/ttyACM0"
    IMU_BAUDRATE = 115200
    
    print("=" * 70)
    print("飞轮倒立摆 Sim2Real 部署")
    print("=" * 70)
    
    print("\n[1/5] 加载策略模型...")
    controller = InvertedPendulumController(MODEL_PATH)
    
    print("\n[2/5] 初始化电机...")
    motor = cw.Motor(CAN_INTERFACE, MOTOR_TX_ID, MOTOR_RX_ID)
    
    print("\n[3/5] 初始化 IMU...")
    imu_reader = imu.JY61PReader(port=IMU_PORT, baudrate=IMU_BAUDRATE)
    if not imu_reader.connect():
        raise Exception("IMU 连接失败")
    time.sleep(0.5)
    
    print("\n[4/5] IMU 校准...")
    print("请将摆杆放在下垂位置，按 Enter 校准...")
    input()
    imu_reader.calibrate_zero(2.0)
    
    # ===== 关键步骤：先让飞轮停止 =====
    print("\n[5/5] 停止飞轮...")
    stop_flywheel(motor, timeout=5.0)
    time.sleep(0.5)

    print("\n" + "=" * 70)
    print("选择模式:")
    print("  1. 正常控制模式")
    print("  2. IMU方向一致性测试（关键！）")
    print("=" * 70)
    
    user_input = input("按回车启动控制").strip()
    
    print("🚀 WobbleGo 启动！")
    
    _ = imu_obs_task(imu_reader)
    _ = update(motor)
    
    try:
        input("\n按 Enter 启用控制...")
        controller.enable_control()
        print("\n控制中...\n")
        cw.block_all()
    except KeyboardInterrupt:
        pass
    
    motor.set_torque(0.0)
    imu_reader.disconnect()
    print("\n已安全退出")