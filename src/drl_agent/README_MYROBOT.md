# MyRobot差速机器人DRL导航使用说明

本文档介绍如何使用你的自定义差速机器人（myrobot_sim_gazebo）进行深度强化学习导航训练。

## 新增文件

| 文件 | 说明 |
|------|------|
| `drl_agent/config/environment_myrobot.yaml` | MyRobot专用环境配置文件 |
| `drl_agent/scripts/environment/environment_myrobot.py` | MyRobot专用环境节点 |
| `drl_agent_gazebo/launch/simulation_myrobot.launch.py` | MyRobot仿真启动文件 |

## 使用方法

### 步骤1：重新编译工作空间

```bash
cd ~/drl_agent_ws
colcon build --symlink-install
source install/setup.zsh
```

### 步骤2：启动仿真环境

```bash
# 终端1 - 启动MyRobot仿真
cd ~/drl_agent_ws
source install/setup.zsh
ros2 launch drl_agent_gazebo simulation_myrobot.launch.py
```

### 步骤3：运行环境节点

```bash
# 终端2 - 运行MyRobot环境节点
cd ~/drl_agent_ws
source install/setup.zsh
ros2 run drl_agent environment_myrobot.py
```

### 步骤4：训练智能体

```bash
# 终端3 - 训练TD7智能体
cd ~/drl_agent_ws
source install/setup.zsh
ros2 run drl_agent train_td7_agent.py
```

## 配置参数

### 环境节点参数

- `laser_topic`: 激光雷达话题名称，默认为 `/scan`
- `environment_mode`: 环境模式，可选 `train`、`test`、`random_test`

```bash
ros2 run drl_agent environment_myrobot.py --ros-args -p laser_topic:=/your_laser_topic
```

### Launch文件参数

- `use_gazebo_gui`: 是否显示Gazebo GUI，默认 `true`
- `rviz`: 是否启动RViz，默认 `false`
- `world`: 世界文件名称，默认 `td7_static.world`

```bash
ros2 launch drl_agent_gazebo simulation_myrobot.launch.py rviz:=true use_gazebo_gui:=false
```

## 机器人配置对比

| 特性 | Pioneer3DX | MyRobot |
|------|------------|---------|
| 激光雷达话题 | `/front_laser/scan` | `/scan` |
| 激光雷达范围 | 180° | 360° |
| 最大探测距离 | 10m | 15m |
| 实体名称 | `pioneer_3dx` | `myrobot_diff_bot` |

## 故障排除

### 问题1：找不到配置文件

确保设置了环境变量：
```bash
export DRL_AGENT_SRC_PATH=/home/chenzhaoqi/drl_agent_ws/src/src
```

### 问题2：话题不匹配

检查话题列表：
```bash
ros2 topic list | grep -E "(scan|odom|cmd_vel)"
```

预期输出：
- `/scan`
- `/odom`
- `/cmd_vel`

### 问题3：机器人位置重置失败

确保Gazebo中的机器人实体名称为 `myrobot_diff_bot`，与配置文件中的 `agent_name` 一致。

## 注意事项

1. **状态空间**: 与2D激光雷达版本完全相同，便于模型迁移。

2. **训练迁移**: 使用MyRobot训练的模型可以直接部署到真实机器人上。

3. **世界文件**: 默认使用 `td7_static.world`，与Pioneer3DX训练环境相同。
