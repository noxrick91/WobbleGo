import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg, DCMotorCfg


USD_PATH = "https://data.noxcaw.com/downloads/wobble-go/usds/WobbleGo.usd"

WOBBLEGO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),  # 底座位置
        rot=(0.707, -0.707, 0.0, 0.0),  # (w, x, y, z)
        joint_pos={
            "base_arm_joint": 0.0,
            "motor_wheel_joint": 0.0,
        },
        joint_vel={
            "base_arm_joint": 0.0,
            "motor_wheel_joint": 0.0,
        },
    ),
    actuators={
        # 飞轮执行器
        # K_T = 0.353 Nm/A，MAX_IQ = 1.5A 最大力矩 ≈ 0.53 Nm
        "flywheel": DCMotorCfg(
            joint_names_expr=["motor_wheel_joint"],
            effort_limit=0.53,        # 峰值力矩: 1.5A * 0.353 Nm/A
            saturation_effort=0.53,   # 饱和力矩
            velocity_limit=100.0,     # 20V下理论极限转速 (rad/s)
            stiffness=0.0,            # 力矩控制模式
            damping=0.005,            # 小阻尼，惯量效应由飞轮质量/惯量本身体现
        ),
        # 摆杆（被动关节）
        "pendulum": ImplicitActuatorCfg(
            joint_names_expr=["base_arm_joint"],
            stiffness=0.0,
            damping=0.005,  # 小阻尼模拟轴承摩擦
        ),
    },
)
