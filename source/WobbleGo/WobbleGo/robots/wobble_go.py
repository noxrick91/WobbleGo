import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg


USD_PATH = "usds/WobbleGo.usd"

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
        "flywheel": ImplicitActuatorCfg(
            joint_names_expr=["motor_wheel_joint"],
            effort_limit=0.35,        # 最大力矩 [Nm]
            velocity_limit=90.0,      # 物理极限转速
            stiffness=0.0,            # 力矩控制模式
            damping=0.001,            # 阻尼
        ),
        # 摆杆（被动关节）
        "pendulum": ImplicitActuatorCfg(
            joint_names_expr=["base_arm_joint"],
            stiffness=0.0,
            damping=0.005,
        ),
    },
)
