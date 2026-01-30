# 2D激光雷达DRL导航使用说明

本文档介绍如何使用2D激光雷达（Hokuyo）代替3D激光雷达（Velodyne）进行深度强化学习导航训练。

## 新增文件

| 文件 | 说明 |
|------|------|
| `drl_agent_description/urdf/p3dx/pioneer3dx_2d_lidar.urdf.xacro` | 仅包含2D激光雷达的机器人模型 |
| `drl_agent/scripts/environment/environment_2d_lidar.py` | 处理2D LaserScan数据的环境节点 |
| `drl_agent_description/launch/agent_description_2d_lidar.launch.py` | 2D雷达版本的机器人描述launch文件 |
| `drl_agent_gazebo/launch/simulation_2d_lidar.launch.py` | 2D雷达版本的仿真launch文件 |

## 使用方法

### 步骤1：重新编译工作空间

```bash
cd ~/drl_agent_ws
colcon build --symlink-install
source install/setup.zsh
```

### 步骤2：启动2D雷达版本的仿真

```bash
# 终端1 - 启动仿真环境（使用2D激光雷达）
cd ~/drl_agent_ws
source install/setup.zsh
ros2 launch drl_agent_gazebo simulation_2d_lidar.launch.py
```

### 步骤3：运行2D雷达版本的环境节点

```bash
# 终端2 - 运行2D雷达环境节点
cd ~/drl_agent_ws
source install/setup.zsh
ros2 run drl_agent environment_2d_lidar.py
```

### 步骤4：训练智能体

```bash
# 终端3 - 训练TD7智能体（与之前相同）
cd ~/drl_agent_ws
source install/setup.zsh
ros2 run drl_agent train_td7_agent.py
```

## 2D vs 3D 雷达对比

| 特性 | 3D雷达 (Velodyne) | 2D雷达 (Hokuyo) |
|------|-------------------|-----------------|
| 数据类型 | `PointCloud2` | `LaserScan` |
| 话题名称 | `/velodyne_points` | `/front_laser/scan` |
| 数据量 | 大（3D点云） | 小（1D距离数组） |
| 处理复杂度 | 需要从笛卡尔坐标计算角度 | 直接获取角度和距离 |
| 计算效率 | 较低 | 较高 |
| 适用场景 | 3D环境感知 | 2D平面导航 |

## 配置参数

2D环境节点支持以下ROS2参数：

- `laser_topic`: 激光雷达话题名称，默认为 `/front_laser/scan`
- `environment_mode`: 环境模式，可选 `train`、`test`、`random_test`

可以通过launch文件或命令行参数进行配置：

```bash
ros2 run drl_agent environment_2d_lidar.py --ros-args -p laser_topic:=/your_laser_topic
```

## 注意事项

1. **状态空间不变**: 2D和3D版本的状态空间维度相同（由`environment.yaml`中的`environment_state_dim`配置），都是将激光数据降采样到固定数量的bin。

2. **训练迁移**: 使用2D雷达训练的模型可以直接部署到只有2D激光雷达的真实机器人上。

3. **仿真效率**: 2D雷达仿真比3D雷达更高效，可以加快训练速度。

4. **原有文件保持不变**: 所有新增功能都是通过新建文件实现的，原有的3D雷达配置完全保留。
