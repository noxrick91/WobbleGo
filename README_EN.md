**English** | **[中文](README.md)**

# 🎯 WobbleGo

## 📖 About

WobbleGo is a beginner-friendly reinforcement learning project built on [Isaac Lab](https://isaac-sim.github.io/IsaacLab/), implementing simulation and control of a **Flywheel Inverted Pendulum (FIP)** robot. This project demonstrates how to build a complete RL environment from scratch, including swing-up control, balancing tasks, and domain randomization techniques for sim-to-real transfer.

**✨ Key Features:**

- 🎡 **Flywheel Inverted Pendulum**: A classic control problem where the robot must swing up from a hanging position and maintain balance using flywheel torque
- 🚀 **Swing-Up & Balance**: Full task implementation supporting swing-up from any initial state and precise balancing
- 🎲 **Domain Randomization**: Comprehensive parameter randomization (mass, inertia, motor torque, damping, external disturbances) for robust policy learning
- 🔧 **Multi-Framework Support**: Compatible with RSL-RL, RL Games, Stable-Baselines3, and SKRL
- 🤖 **Sim-to-Real Ready**: Designed for real hardware deployment (motor limits, sensor noise, cable interference simulation)

**🏷️ Keywords:** isaaclab, robotics, reinforcement learning, inverted pendulum, flywheel

[![Discord](https://img.shields.io/badge/Discord-Join%20Community-7289da?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/geH9ed5Q3N)
[![Forum](https://img.shields.io/badge/Discourse-Forum-FF3300?style=for-the-badge&logo=discourse&logoColor=white)](https://noxcaw.com/c/robot/9)
[![Isaac Lab](https://img.shields.io/badge/Simulator-Isaac%20Lab-orange?style=for-the-badge)](https://isaac-sim.github.io/IsaacLab/)


[Screencast from 2026-02-02 18-28-36.webm](https://github.com/user-attachments/assets/f338d787-c501-449f-a4fb-301595f98dc4)

---

## 🚀 OnShape 3D Model

https://cad.onshape.com/documents/345d5754db0b35ab26d74afa/w/a47014c097ea2e746a23cdd7/e/d60f97581313bd901aedc762

---

## 📁 Project Structure

```
WobbleGo/
├── 📂 source/WobbleGo/
│   ├── 📂 WobbleGo/
│   │   ├── 📂 robots/              # Robot USD models and configs
│   │   │   └── wobble_go.py        # Robot joint configuration
│   │   └── 📂 tasks/
│   │       └── 📂 direct/
│   │           └── 📂 wobblego/
│   │               ├── wobblego_env.py      # Environment implementation
│   │               ├── wobblego_env_cfg.py  # Environment configuration
│   │               └── 📂 agents/           # RL agent configs
│   ├── config/extension.toml       # Isaac Lab extension config
│   └── pyproject.toml
├── 📂 scripts/                      # Training and evaluation scripts
│   ├── 📂 rsl_rl/                   # RSL-RL training scripts
│   ├── 📂 rl_games/                 # RL Games training scripts
│   ├── 📂 sb3/                      # Stable-Baselines3 scripts
│   ├── 📂 skrl/                     # SKRL training scripts
│   ├── list_envs.py                 # List available environments
│   ├── zero_agent.py                # Zero-action test agent
│   └── random_agent.py              # Random-action test agent
└── 📂 deployment/                   # Sim-to-Real deployment code
    ├── init.sh                      # CAN bus initialization
    ├── main.py                      # Deployment main program
    └── 📂 src/
        └── imu.py                   # JY61P IMU driver
```

---

## 🎮 Environment Details

### Task: WobbleGo-Direct-v0

| Property | Value |
|----------|-------|
| 📊 Observation Space | 4D (cos θ, sin θ, pole angular velocity, flywheel angular velocity) |
| 🎯 Action Space | 1D (flywheel torque) |
| ⏱️ Episode Length | 15 seconds |
| 🔄 Simulation Frequency | 300 Hz |
| 🕹️ Control Frequency | 150 Hz (decimation=2) |

### 🏆 Reward Function

The reward function supports both swing-up and balancing:

| Component | Description |
|-----------|-------------|
| 🎯 Upright Reward | Encourages the pole to reach vertical position |
| 📈 Progress Reward | Rewards movement toward the target angle (encourages swing-up) |
| ⚡ Energy Reward | Rewards mechanical energy accumulation during swing-up |
| 🛑 Velocity Penalty | Penalizes high angular velocity near the top (encourages stability) |
| 💡 Energy Penalty | Penalizes excessive motor torque usage |

### 🎲 Domain Randomization

For robust sim-to-real transfer, the environment includes the following randomizations:

| Parameter Type | Randomization Range |
|----------------|-------------------|
| 🎡 Flywheel Mass | 0.5x - 2.8x |
| 📏 Pendulum Arm Mass | 0.5x - 1.5x |
| ⚙️ Motor Torque | 0.6x - 1.5x |
| 🔧 Joint Damping | Variable arm and motor damping |
| 💨 External Disturbance | Simulated cable interference torque |
| 📡 Observation Noise | Angle (~1°) and velocity noise |
| 🎛️ Action Noise | Motor control imprecision simulation |

---

## 🔧 Installation

### 1️⃣ Install Isaac Lab

Follow the [official installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) to install Isaac Lab.

> 💡 We recommend using conda or uv installation for easy Python script execution from the terminal.

### 2️⃣ Clone This Repository

```bash
git clone https://github.com/noxrick91/WobbleGo.git
cd WobbleGo
```

### 3️⃣ Install the Extension

```bash
# If Isaac Lab is not in your Python venv/conda, replace 'python' with 'PATH_TO_isaaclab.sh -p'
python -m pip install -e source/WobbleGo
```

### 4️⃣ Verify Installation

- 📋 List available tasks:

  ```bash
  python scripts/list_envs.py
  ```

  You should see `WobbleGo-Direct-v0`.

- 🧪 Zero-action test (verify environment setup):

  ```bash
  python scripts/zero_agent.py --task=WobbleGo-Direct-v0
  ```

- 🎲 Random-action test:

  ```bash
  python scripts/random_agent.py --task=WobbleGo-Direct-v0
  ```

---

## 🏋️ Training

### Using RSL-RL (Recommended) ⭐

```bash
python scripts/rsl_rl/train.py --task=WobbleGo-Direct-v0 --headless
```

### Using RL Games

```bash
python scripts/rl_games/train.py --task=WobbleGo-Direct-v0
```

### Using Stable-Baselines3

```bash
python scripts/sb3/train.py --task=WobbleGo-Direct-v0
```

### Using SKRL

```bash
python scripts/skrl/train.py --task=WobbleGo-Direct-v0
```

### TensorBoard

```bash
tensorboard --logdir logs
```

<img width="1853" height="1053" alt="Screenshot from 2026-02-04 04-53-00" src="https://github.com/user-attachments/assets/8e54cb37-3a4d-4749-9dde-cdb9c94bae22" />

---

## 🎬 Evaluation / Play

After training, evaluate your policy:

```bash
# RSL-RL
python scripts/rsl_rl/play.py --task=WobbleGo-Direct-v0 --num_envs=64

# RL Games
python scripts/rl_games/play.py --task=WobbleGo-Direct-v0

# Stable-Baselines3
python scripts/sb3/play.py --task=WobbleGo-Direct-v0

# SKRL
python scripts/skrl/play.py --task=WobbleGo-Direct-v0
```

---

## 🔄 Sim-to-Real Deployment

This project supports deploying simulation-trained policies directly to real flywheel inverted pendulum hardware.

### Hardware Requirements

| Component | Description |
|-----------|-------------|
| 🔧 Brushless Motor | Brushless motor with CAN bus interface (for driving the flywheel) |
| 📡 IMU Sensor | JY61P attitude sensor (serial communication, for pole angle and angular velocity) |
| 🖥️ Controller | Linux host with CAN bus and serial port support (e.g., Jetson, Raspberry Pi) |
| 🔌 CAN Adapter | USB-to-CAN or onboard CAN interface |

### Deployment Directory Structure

```
deployment/
├── init.sh            # CAN bus initialization script
├── main.py            # Deployment main program
└── src/
    ├── __init__.py
    └── imu.py         # JY61P IMU driver
```

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| K_T | 0.235 | Motor torque constant (Nm/A) |
| ACTION_SCALE | 0.35 | Action scaling factor |
| CONTROL_FREQ | 100 Hz | Control frequency |
| MAX_IQ | 1.5 A | Maximum current limit |
| MAX_FLYWHEEL_SPEED | 100 rad/s | Flywheel safety speed threshold |
| FLYWHEEL_VEL_SCALE | 90.0 | Flywheel velocity normalization coefficient |

### Deployment Steps

#### 1️⃣ Initialize CAN Bus

```bash
cd deployment
sudo bash init.sh
```

This script configures the CAN0 interface (1Mbps baud rate, TX queue length 1000):

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

#### 2️⃣ Prepare the Trained Model

Place the simulation-trained model file (`.pt`) in the appropriate location and update the model path in `main.py`:

```python
MODEL_PATH = "logs/rsl_rl/wobblego_direct/2026-02-09_22-14-15/model_300.pt"
```

#### 3️⃣ Configure Hardware Parameters

Modify the following parameters in `main.py` according to your actual hardware connections:

```python
CAN_INTERFACE = "can0"         # CAN interface name
MOTOR_TX_ID = 0x200            # Motor TX ID
MOTOR_RX_ID = 0x100            # Motor RX ID
IMU_PORT = "/dev/ttyACM0"      # IMU serial port
IMU_BAUDRATE = 115200           # IMU baud rate
```

#### 4️⃣ Run the Deployment Program

```bash
cd deployment
python main.py
```

The program runs through the following sequence:

1. **Load Policy Model** — Automatically parses network architecture and loads weights
2. **Initialize Motor** — Connects to the brushless motor via CAN bus
3. **Initialize IMU** — Connects to the JY61P attitude sensor via serial port
4. **IMU Calibration** — Place the pole in the hanging position and press Enter to calibrate zero point
5. **Flywheel Braking** — Automatically decelerates the flywheel to a stop
6. **Start Control** — Press Enter to enable RL policy control

### Safety Mechanisms

- **Flywheel Overspeed Protection**: Automatically stops control when flywheel speed exceeds 100 rad/s
- **Voltage Saturation Speed Limiting**: Limits same-direction torque when flywheel approaches speed limit
- **Current Clamping**: Output current limited to ±1.5A
- **Pre-Start Braking**: Automatically decelerates the flywheel before startup
- **Safe Shutdown**: Press `Ctrl+C` to safely stop the motor and disconnect IMU

### Observation Space Mapping

The deployment code uses the same 4D observation space as the simulation environment:

| Index | Observation | Processing |
|-------|------------|------------|
| 0 | cos(θ) | Directly from IMU |
| 1 | sin(θ) | From IMU with direction compensation |
| 2 | Pole angular velocity | From IMU, normalized by /10 |
| 3 | Flywheel angular velocity | From motor encoder, normalized by /90 and clipped to [-1, 1] |

### Direction Calibration

If the robot behaves abnormally (e.g., applying force in the wrong direction), adjust the following direction parameters:

```python
TORQUE_DIRECTION = +1.0         # Torque direction
FLYWHEEL_VEL_DIRECTION = +1.0   # Flywheel velocity reading direction
POLE_ANGLE_DIRECTION = +1.0     # Pole angle direction
POLE_VEL_DIRECTION = +1.0       # Pole angular velocity direction
```

> 💡 **Tip**: During deployment, first observe the observation values in the debug output. Ensure they are consistent with the simulation directions, then adjust the direction parameters accordingly.

---

## 💻 IDE Setup (Optional)

Configure VSCode for a better development experience:

1. Press `Ctrl+Shift+P`, select `Tasks: Run Task`, and run `setup_python_env`
2. Enter the absolute path of Isaac Sim when prompted

This creates a `.python.env` file in the `.vscode` directory for intelligent code completion.

---

## 🔌 Omniverse Extension (Optional)

Enable this project as an Omniverse extension:

1. **Add Search Path**:
   - Open `Window` -> `Extensions`
   - Click the **hamburger menu** -> `Settings`
   - Add the absolute path of this project's `source` directory to `Extension Search Paths`
   - If not already added, also add the Isaac Lab extensions directory (`IsaacLab/source`)
   - Click the **hamburger menu** -> `Refresh`

2. **Enable Extension**:
   - Find `WobbleGo` under the `Third Party` category
   - Toggle the switch to enable

---

## 🎨 Code Formatting

This project uses pre-commit for code formatting:

```bash
pip install pre-commit
pre-commit run --all-files
```

---

## ❓ FAQ

### 🔍 Missing Pylance Indexing

If VSCode cannot properly index the extension, add the path in `.vscode/settings.json`:

```json
{
    "python.analysis.extraPaths": [
        "<path-to-this-repo>/source/WobbleGo"
    ]
}
```

### 💥 Pylance Crashes

If Pylance crashes due to memory issues, exclude unused Omniverse packages in `.vscode/settings.json` under `python.analysis.extraPaths`:

```json
"<isaac-sim-path>/extscache/omni.anim.*"
"<isaac-sim-path>/extscache/omni.kit.*"
"<isaac-sim-path>/extscache/omni.graph.*"
"<isaac-sim-path>/extscache/omni.services.*"
```

---

## 📄 License

This project is open-sourced under the MIT License.

---

## 👤 Author

- **noxrick91** - [GitHub](https://github.com/noxrick91)

---

<p align="center">
  🌟 If this project helps you, please give it a Star! 🌟
</p>
