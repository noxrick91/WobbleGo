# 🎯 WobbleGo

## 📖 项目简介

WobbleGo 是一个基于 [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) 的入门级强化学习项目，实现了**飞轮倒立摆 (Flywheel Inverted Pendulum, FIP)** 机器人的仿真与控制。本项目展示了如何从零开始构建一个完整的强化学习环境，包括起摆控制、平衡任务以及用于 sim-to-real 迁移的域随机化技术。

**✨ 核心特性：**

- 🎡 **飞轮倒立摆**：经典控制问题，机器人需要通过飞轮扭矩从下垂位置摆起并保持平衡
- 🚀 **起摆与平衡**：完整的任务实现，支持从任意初始状态起摆和精确平衡
- 🎲 **域随机化**：全面的参数随机化（质量、惯量、电机扭矩、阻尼、外部扰动），实现鲁棒策略学习
- 🔧 **多框架支持**：支持 RSL-RL、RL Games、Stable-Baselines3 和 SKRL
- 🤖 **Sim-to-Real 就绪**：针对真实硬件部署设计（电机限制、传感器噪声、线缆干扰模拟）

**🏷️ 关键词：** isaaclab, 机器人, 强化学习, 倒立摆, 飞轮

[![Discord](https://img.shields.io/badge/Discord-Join%20Community-7289da?style=for-the-badge&logo=discord&logoColor=white)](https://discord.gg/geH9ed5Q3N)
[![Forum](https://img.shields.io/badge/Discourse-Forum-FF3300?style=for-the-badge&logo=discourse&logoColor=white)](https://noxcaw.com/c/robot/9)
[![Isaac Lab](https://img.shields.io/badge/Simulator-Isaac%20Lab-orange?style=for-the-badge)](https://isaac-sim.github.io/IsaacLab/)

---

## 🚀 OnShape 3D模型

https://cad.onshape.com/documents/345d5754db0b35ab26d74afa/w/a47014c097ea2e746a23cdd7/e/d60f97581313bd901aedc762

---

## 📁 项目结构

```
WobbleGo/
├── 📂 source/WobbleGo/
│   ├── 📂 WobbleGo/
│   │   ├── 📂 robots/              # 机器人 USD 模型和配置
│   │   │   └── wobble_go.py        # 机器人关节配置
│   │   └── 📂 tasks/
│   │       └── 📂 direct/
│   │           └── 📂 wobblego/
│   │               ├── wobblego_env.py      # 环境实现
│   │               ├── wobblego_env_cfg.py  # 环境配置
│   │               └── 📂 agents/           # RL 智能体配置
│   ├── config/extension.toml       # Isaac Lab 扩展配置
│   └── pyproject.toml
├── 📂 scripts/                      # 训练和评估脚本
│   ├── 📂 rsl_rl/                   # RSL-RL 训练脚本
│   ├── 📂 rl_games/                 # RL Games 训练脚本
│   ├── 📂 sb3/                      # Stable-Baselines3 脚本
│   ├── 📂 skrl/                     # SKRL 训练脚本
│   ├── list_envs.py                 # 列出可用环境
│   ├── zero_agent.py                # 零动作测试智能体
│   └── random_agent.py              # 随机动作测试智能体
└── 📂 deployment/                   # 真实硬件部署代码
```

---

## 🎮 环境详情

### 任务：WobbleGo-Direct-v0

| 属性 | 值 |
|------|-----|
| 📊 观测空间 | 4 维 (cos θ, sin θ, 摆杆角速度, 飞轮角速度) |
| 🎯 动作空间 | 1 维 (飞轮扭矩) |
| ⏱️ 回合时长 | 15 秒 |
| 🔄 仿真频率 | 300 Hz |
| 🕹️ 控制频率 | 150 Hz (decimation=2) |

### 🏆 奖励函数

奖励函数同时支持起摆和平衡：

| 组成部分 | 说明 |
|----------|------|
| 🎯 竖直奖励 | 鼓励摆杆达到垂直位置 |
| 📈 进度奖励 | 奖励向目标角度移动（鼓励起摆） |
| ⚡ 能量奖励 | 奖励起摆阶段积累机械能 |
| 🛑 速度惩罚 | 惩罚接近顶部时的高角速度（鼓励稳定） |
| 💡 能耗惩罚 | 惩罚过度使用电机扭矩 |

### 🎲 域随机化

为实现鲁棒的 sim-to-real 迁移，环境包含以下随机化：

| 参数类型 | 随机化范围 |
|----------|-----------|
| 🎡 飞轮质量 | 0.5x - 2.8x |
| 📏 摆臂质量 | 0.5x - 1.5x |
| ⚙️ 电机扭矩 | 0.6x - 1.5x |
| 🔧 关节阻尼 | 可变摆臂和电机阻尼 |
| 💨 外部扰动 | 模拟线缆干扰力矩 |
| 📡 观测噪声 | 角度 (~1°) 和速度噪声 |
| 🎛️ 动作噪声 | 电机控制不精确模拟 |

---

## 🔧 安装指南

### 1️⃣ 安装 Isaac Lab

按照 [官方安装指南](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) 安装 Isaac Lab。

> 💡 推荐使用 conda 或 uv 安装方式，方便从终端调用 Python 脚本。

### 2️⃣ 克隆本仓库

```bash
git clone https://github.com/noxrick91/WobbleGo.git
cd WobbleGo
```

### 3️⃣ 安装扩展

```bash
# 如果 Isaac Lab 不在你的 Python venv/conda 中，使用 'PATH_TO_isaaclab.sh -p' 替代 'python'
python -m pip install -e source/WobbleGo
```

### 4️⃣ 验证安装

- 📋 列出可用任务：

  ```bash
  python scripts/list_envs.py
  ```

  你应该能看到 `WobbleGo-Direct-v0`。

- 🧪 零动作测试（验证环境配置）：

  ```bash
  python scripts/zero_agent.py --task=WobbleGo-Direct-v0
  ```

- 🎲 随机动作测试：

  ```bash
  python scripts/random_agent.py --task=WobbleGo-Direct-v0
  ```

---

## 🏋️ 训练

### 使用 RSL-RL（推荐）⭐

```bash
python scripts/rsl_rl/train.py --task=WobbleGo-Direct-v0 --headless
```

### 使用 RL Games

```bash
python scripts/rl_games/train.py --task=WobbleGo-Direct-v0
```

### 使用 Stable-Baselines3

```bash
python scripts/sb3/train.py --task=WobbleGo-Direct-v0
```

### 使用 SKRL

```bash
python scripts/skrl/train.py --task=WobbleGo-Direct-v0
```

---

## 🎬 评估 / 运行

训练完成后，评估你的策略：

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

## 💻 IDE 配置（可选）

配置 VSCode 以获得更好的开发体验：

1. 按 `Ctrl+Shift+P`，选择 `Tasks: Run Task`，运行 `setup_python_env`
2. 按提示输入 Isaac Sim 的绝对路径

这会在 `.vscode` 目录下创建 `.python.env` 文件，提供智能代码补全。

---

## 🔌 Omniverse 扩展（可选）

将本项目作为 Omniverse 扩展启用：

1. **添加搜索路径**：
   - 打开 `Window` -> `Extensions`
   - 点击 **汉堡菜单** -> `Settings`
   - 在 `Extension Search Paths` 中添加本项目 `source` 目录的绝对路径
   - 如未添加，也需要添加 Isaac Lab 扩展目录 (`IsaacLab/source`)
   - 点击 **汉堡菜单** -> `Refresh`

2. **启用扩展**：
   - 在 `Third Party` 分类下找到 `WobbleGo`
   - 切换开关启用

---

## 🎨 代码格式化

本项目使用 pre-commit 进行代码格式化：

```bash
pip install pre-commit
pre-commit run --all-files
```

---

## ❓ 常见问题

### 🔍 Pylance 索引缺失

如果 VSCode 无法正确索引扩展，在 `.vscode/settings.json` 中添加路径：

```json
{
    "python.analysis.extraPaths": [
        "<本仓库路径>/source/WobbleGo"
    ]
}
```

### 💥 Pylance 崩溃

如果 Pylance 因内存问题崩溃，在 `.vscode/settings.json` 的 `python.analysis.extraPaths` 中排除未使用的 Omniverse 包：

```json
"<isaac-sim路径>/extscache/omni.anim.*"
"<isaac-sim路径>/extscache/omni.kit.*"
"<isaac-sim路径>/extscache/omni.graph.*"
"<isaac-sim路径>/extscache/omni.services.*"
```

---

## 📄 许可证

本项目基于 MIT 许可证开源。

---

## 👤 作者

- **noxrick91** - [GitHub](https://github.com/noxrick91)

---

<p align="center">
  🌟 如果这个项目对你有帮助，欢迎 Star！🌟
</p>
