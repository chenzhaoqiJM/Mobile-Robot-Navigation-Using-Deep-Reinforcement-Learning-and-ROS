#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MyRobot差速机器人环境节点
基于environment_2d_lidar.py修改，适配myrobot_sim_gazebo机器人
用于深度强化学习(DRL)导航训练
"""

import os
import sys
import math
import threading
import random
import time
import numpy as np
from collections import deque
from squaternion import Quaternion

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import EntityState

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from drl_agent_interfaces.srv import Step, Reset, Seed, GetDimensions, SampleActionSpace

from file_manager import load_yaml


class EnvironmentMyRobot(Node):
    """MyRobot差速机器人环境节点，为DRL提供所需服务。

    该类通过ROS2服务与环境交互，提供以下服务：
    - step: 执行动作并获取环境反馈
    - reset: 重置环境并获取初始观测
    - get_dimensions: 获取状态、动作的维度和最大动作值
    """

    def __init__(self):
        super().__init__("gym_node_myrobot")

        # 确定环境运行模式（训练或测试）
        self.declare_parameter("environment_mode", "train")
        self.environment_mode = (
            self.get_parameter("environment_mode")
            .get_parameter_value()
            .string_value.lower()
        )
        if not self.environment_mode in ["train", "test", "random_test"]:
            raise NotImplementedError
        # 环境运行模式
        self.train_mode = (
            self.environment_mode == "train" or self.environment_mode == "random_test"
        )
        self.get_logger().info(f"环境运行模式: {self.environment_mode}")
        self.get_logger().info("使用MyRobot差速机器人 (2D LaserScan)")

        # 加载环境配置文件
        drl_agent_src_path_env = "DRL_AGENT_SRC_PATH"
        drl_agent_src_path = os.getenv(drl_agent_src_path_env)
        if drl_agent_src_path is None:
            self.get_logger().error(
                f"环境变量 {drl_agent_src_path_env} 未设置"
            )
            sys.exit(-1)
        # 使用myrobot专用配置文件
        env_config_file_name = "environment_myrobot.yaml"
        start_goal_pairs_file = "test_config.yaml"
        env_config_file_path = os.path.join(
            drl_agent_src_path, "drl_agent", "config", env_config_file_name
        )
        start_goal_pairs_file_path = os.path.join(
            drl_agent_src_path, "drl_agent", "config", start_goal_pairs_file
        )
        # 定义状态、动作的维度和最大动作值
        try:
            self.config = load_yaml(env_config_file_path)
        except Exception as e:
            self.get_logger().info(f"无法加载配置文件: {e}")
            sys.exit(-1)
        self.environment_config = self.config["environment"]
        self.lower = self.environment_config["lower"]
        self.upper = self.environment_config["upper"]
        self.environment_dim = self.environment_config["environment_state_dim"]
        self.agent_dim = self.environment_config["agent_state_dim"]
        self.agent_name = self.environment_config["agent_name"]
        self.num_of_obstacles = self.environment_config["num_of_obstacles"]

        self.action_dim = self.environment_config["action_dim"]
        self.max_action = self.environment_config["max_action"]
        self.actions_low = self.environment_config["actions_low"]
        self.actions_high = self.environment_config["actions_high"]

        self.threshold_params_config = self.config["threshold_parameters"]
        self.goal_threshold = self.threshold_params_config["goal_threshold"]
        self.collision_threshold = self.threshold_params_config["collision_threshold"]
        self.time_delta = self.threshold_params_config["time_delta"]
        self.inter_entity_distance = self.threshold_params_config[
            "inter_entity_distance"
        ]

        self.lidar_max_range = self.threshold_params_config["lidar_max_range"]

        # 声明2D雷达话题参数（myrobot使用/scan话题）
        self.declare_parameter("laser_topic", "/scan")
        self.laser_topic = (
            self.get_parameter("laser_topic")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"订阅激光雷达话题: {self.laser_topic}")

        # 并行处理传感器和服务的回调组
        self.odom_callback_group = MutuallyExclusiveCallbackGroup()
        self.laser_callback_group = MutuallyExclusiveCallbackGroup()
        self.clients_callback_group = MutuallyExclusiveCallbackGroup()

        # 初始化发布者
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_point_marker_pub = self.create_publisher(
            MarkerArray, "goal_point", 10
        )
        self.linear_vel_marker_pub = self.create_publisher(
            MarkerArray, "linear_velocity", 10
        )
        self.angular_vel_marker_pub = self.create_publisher(
            MarkerArray, "angular_velocity", 10
        )

        # 创建服务
        self.srv_seed = self.create_service(Seed, "seed", self.seed_callback)
        self.srv_step = self.create_service(Step, "step", self.step_callback)
        self.srv_reset = self.create_service(Reset, "reset", self.reset_callback)
        self.srv_dimentions = self.create_service(
            GetDimensions, "get_dimensions", self.get_dimensions_callback
        )
        self.srv_action_space_sample = self.create_service(
            SampleActionSpace, "action_space_sample", self.sample_action_callback
        )
        # 初始化客户端
        self.unpause = self.create_client(
            Empty, "/unpause_physics", callback_group=self.clients_callback_group
        )
        self.pause = self.create_client(
            Empty, "/pause_physics", callback_group=self.clients_callback_group
        )
        self.reset_proxy = self.create_client(
            Empty, "/reset_world", callback_group=self.clients_callback_group
        )
        self.set_model_state = self.create_client(
            SetEntityState,
            "gazebo/set_entity_state",
            callback_group=self.clients_callback_group,
        )
        # 传感器订阅QoS
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 里程计订阅
        self.odom = self.create_subscription(
            Odometry,
            "/odom",
            self.update_agent_state,
            qos_profile,
            callback_group=self.odom_callback_group,
        )
        self.odom

        # 2D激光雷达订阅
        self.laser_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.update_environment_state,
            qos_profile,
            callback_group=self.laser_callback_group,
        )
        self.laser_sub

        # 初始化命令
        self.velocity_command = Twist()
        self.set_agent_state = EntityState()
        self.set_agent_state.name = self.agent_name
        self.set_obstacle_state = EntityState()
        # 命令请求
        self.set_agent_state_req = SetEntityState.Request()
        self.set_static_obs_state_req = SetEntityState.Request()

        # 初始化环境和智能体状态
        self.environment_state = None
        self.agent_state = None
        # 初始化锁以保护环境状态和智能体状态免受竞态条件影响
        self.environment_state_lock = threading.Lock()
        self.agent_state_lock = threading.Lock()

        # 加载起点-目标点对
        if not self.train_mode:
            try:
                self.start_goal_pairs = deque(
                    load_yaml(start_goal_pairs_file_path)["start_goal_pairs"]
                )
            except Exception as e:
                self.get_logger().error(f"无法加载起点-目标点对: {e}")
                sys.exit(-1)
            self.current_pairs = None

        # 定义初始目标位置
        self.goal_x = 0.0
        self.goal_y = 0.0

    def terminate_session(self):
        """销毁节点并关闭rclpy"""
        self.get_logger().info("gym_node_myrobot 正在关闭...")
        self.destroy_node()

    def seed_callback(self, request, response):
        """设置环境种子以实现训练过程的可重复性"""
        np.random.seed(request.seed)
        response.success = True
        return response

    def sample_action_callback(self, _, response):
        """从动作空间中采样动作"""
        action = np.random.uniform(self.actions_low, self.actions_high)
        response.action = np.array(action, dtype=np.float32).tolist()
        return response

    def get_dimensions_callback(self, _, response):
        """返回状态、动作的维度和最大动作值"""
        response.state_dim = self.environment_dim + self.agent_dim
        response.action_dim = self.action_dim
        response.max_action = self.max_action
        return response

    def update_environment_state(self, laser_data):
        """使用2D激光扫描数据更新环境状态

        读取LaserScan数据，将360度范围的激光数据降采样到environment_dim个bin，
        每个bin取最小距离值作为状态表示。
        """
        with self.environment_state_lock:
            self.environment_state = (
                np.ones(self.environment_dim) * self.lidar_max_range
            )
            
            # 获取激光扫描参数
            ranges = np.array(laser_data.ranges)
            angle_min = laser_data.angle_min
            angle_max = laser_data.angle_max
            angle_increment = laser_data.angle_increment
            num_readings = len(ranges)
            
            if num_readings == 0:
                return
            
            # 计算每个bin对应的角度范围
            # 假设我们关注的是前方180度范围 (-90度 到 +90度)
            bin_size = np.pi / self.environment_dim  # 每个bin的角度大小
            
            for i in range(num_readings):
                # 计算当前激光束的角度
                angle = angle_min + i * angle_increment
                
                # 只处理前方180度范围 (-pi/2 到 pi/2)
                if angle < -np.pi / 2 or angle > np.pi / 2:
                    continue
                
                # 获取距离值
                dist = ranges[i]
                
                # 跳过无效值（inf, nan, 超出范围）
                if not np.isfinite(dist) or dist < laser_data.range_min:
                    continue
                if dist > laser_data.range_max:
                    dist = self.lidar_max_range
                
                # 计算该角度属于哪个bin
                # 将角度从 [-pi/2, pi/2] 映射到 [0, environment_dim-1]
                bin_index = int((angle + np.pi / 2) / bin_size)
                bin_index = max(0, min(bin_index, self.environment_dim - 1))
                
                # 取该bin内的最小距离
                self.environment_state[bin_index] = min(
                    self.environment_state[bin_index], dist
                )

    def get_environment_state(self):
        """返回环境状态的副本"""
        with self.environment_state_lock:
            return self.environment_state.copy()

    def update_agent_state(self, odom):
        """使用里程计数据更新智能体状态"""
        with self.agent_state_lock:
            # 从里程计数据计算机器人朝向
            odom_x = odom.pose.pose.position.x
            odom_y = odom.pose.pose.position.y
            quaternion = Quaternion(
                odom.pose.pose.orientation.w,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
            )
            euler = quaternion.to_euler(degrees=False)
            angle = round(euler[2], 4)

            # 计算机器人到目标的距离
            distance = np.linalg.norm([odom_x - self.goal_x, odom_y - self.goal_y])

            # 计算机器人朝向与朝向目标之间的相对角度
            skew_x = self.goal_x - odom_x
            skew_y = self.goal_y - odom_y
            dot = skew_x * 1 + skew_y * 0
            mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
            mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
            beta = math.acos(dot / (mag1 * mag2))
            if skew_y < 0:
                if skew_x < 0:
                    beta = -beta
                else:
                    beta = 0 - beta
            theta = beta - angle
            if theta > np.pi:
                theta = np.pi - theta
                theta = -np.pi - theta
            if theta < -np.pi:
                theta = -np.pi - theta
                theta = np.pi - theta

            self.agent_state = np.array([distance, theta, 0, 0])

    def get_agent_state(self):
        """返回智能体状态的副本"""
        with self.agent_state_lock:
            return self.agent_state.copy()

    def set_gazebo_model_state(self, model_state):
        """改变gazebo模型的位置"""
        while not self.set_model_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "服务 /gazebo/set_entity_state 不可用，等待中..."
            )
        try:
            self.set_model_state.call_async(model_state)
        except Exception as e:
            self.get_logger().error(
                "/gazebo/set_entity_state 服务调用失败: %s" % str(e)
            )
            sys.exit(-1)

    def propagate_state(self, time_delta):
        """传播环境状态time_delta秒"""
        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "服务 /unpause_physics 不可用，等待中..."
            )
        try:
            self.unpause.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error("/unpause_physics 服务调用失败: %s" % str(e))
            sys.exit(-1)
        # 传播状态time_delta秒
        time.sleep(time_delta)
        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "服务 /pause_physics 不可用，等待中..."
            )
        try:
            self.pause.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error("/pause_physics 服务调用失败: %s" % str(e))
            sys.exit(-1)

    def step_callback(self, request, response):
        """在环境中执行一步，更新机器人状态并返回新状态"""
        target = False
        action = request.action
        # 发送速度命令
        self.velocity_command.linear.x = action[0]
        self.velocity_command.angular.z = action[1]
        self.velocity_publisher.publish(self.velocity_command)
        self.publish_markers(action)

        # 传播状态time_delta秒
        self.propagate_state(self.time_delta)

        # 计算状态
        environment_state = self.get_environment_state()
        agent_state = self.get_agent_state()
        agent_state[2], agent_state[3] = action[0], action[1]
        state = np.append(environment_state, agent_state)

        # 计算奖励
        done, collision, min_laser = self.check_collision(environment_state)
        if agent_state[0] < self.goal_threshold:
            self.get_logger().info(f"{'目标到达':-^50}")
            target = True
            done = True
        reward = self.get_reward(target, collision, action, min_laser)

        # 构建响应
        response.state = state.tolist()
        response.reward = reward
        response.done = done
        response.target = target
        return response

    def reset_callback(self, _, response):
        """重置环境状态并返回初始观测状态"""

        """*****************************************************
		** 首先重置世界
		*****************************************************"""
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "服务 /reset_world 不可用，等待中..."
            )
        try:
            self.reset_proxy.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().error("/reset_world 服务调用失败: %s" % str(e))
            sys.exit(-1)
        time.sleep(self.time_delta)

        """*****************************************************
		** 确定智能体的起始位置
		*****************************************************"""
        if self.train_mode:
            position_ok = False
            angle = np.random.uniform(-np.pi, np.pi)
            while not position_ok:
                start_x = np.random.uniform(self.lower, self.upper)
                start_y = np.random.uniform(self.lower, self.upper)
                position_ok = not self.check_dead_zone(start_x, start_y)
        else:
            if not self.start_goal_pairs:
                self.get_logger().info(f"{'所有起点-目标点对已遍历完成':-^50}")
                self.terminate_session()
            self.current_pairs = self.start_goal_pairs.popleft()
            start_x = self.current_pairs["start"]["x"]
            start_y = self.current_pairs["start"]["y"]
            angle = self.current_pairs["start"]["theta"]

        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        self.set_agent_state.pose.position.x = start_x
        self.set_agent_state.pose.position.y = start_y
        self.set_agent_state.pose.position.z = 0.0
        self.set_agent_state.pose.orientation.x = quaternion.x
        self.set_agent_state.pose.orientation.y = quaternion.y
        self.set_agent_state.pose.orientation.z = quaternion.z
        self.set_agent_state.pose.orientation.w = quaternion.w

        self.set_agent_state_req.state = self.set_agent_state
        # 设置智能体状态
        self.set_gazebo_model_state(self.set_agent_state_req)

        """*****************************************************
		** 更换目标并随机化障碍物
		*****************************************************"""
        self.change_goal()
        if self.train_mode:
            self.shuffle_obstacles(start_x, start_y)
        # 为rviz发布标记
        self.publish_markers([0.0, 0.0])
        # 传播状态2*time_delta秒
        self.propagate_state(2 * self.time_delta)

        """*****************************************************
		** 计算重置后的状态
		*****************************************************"""
        environment_state = self.get_environment_state()
        agent_state = self.get_agent_state()
        response.state = np.append(environment_state, agent_state).tolist()
        return response

    def change_goal(self):
        """放置新目标并确保其位置不在障碍物上"""
        if self.train_mode:
            goal_ok = False
            while not goal_ok:
                self.goal_x = random.uniform(self.upper, self.lower)
                self.goal_y = random.uniform(self.upper, self.lower)
                goal_ok = not self.check_dead_zone(self.goal_x, self.goal_y)
        else:
            self.goal_x = self.current_pairs["goal"]["x"]
            self.goal_y = self.current_pairs["goal"]["y"]

    def check_collision(self, laser_data):
        """从激光数据检测碰撞"""
        done, collision = False, False
        min_laser = min(laser_data)
        if min_laser < self.collision_threshold:
            done, collision = True, True
        return done, collision, min_laser

    def shuffle_obstacles(self, start_x, start_y):
        """在重置时随机改变障碍物位置"""
        prev_obstacle_positions = []
        for i in range(1, self.num_of_obstacles + 1):
            position_ok = False
            self.set_obstacle_state.name = "obstacle_" + str(i)
            while not position_ok:
                x = np.random.uniform(self.lower, self.upper)
                y = np.random.uniform(self.lower, self.upper)

                position_ok = not self.check_dead_zone(x, y)
                distance_to_robot = np.linalg.norm([x - start_x, y - start_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if (
                    distance_to_robot < self.inter_entity_distance
                    or distance_to_goal < self.inter_entity_distance
                ):
                    position_ok = False
                    continue

                for prev_x, prev_y in prev_obstacle_positions:
                    distance_to_other_obstacles = np.linalg.norm(
                        [x - prev_x, y - prev_y]
                    )
                    if distance_to_other_obstacles < self.inter_entity_distance:
                        position_ok = False

            self.set_obstacle_state.pose.position.x = x
            self.set_obstacle_state.pose.position.y = y
            self.set_obstacle_state.pose.position.z = 0.0
            self.set_obstacle_state.pose.orientation.x = 0.0
            self.set_obstacle_state.pose.orientation.y = 0.0
            self.set_obstacle_state.pose.orientation.z = 0.0
            self.set_obstacle_state.pose.orientation.w = 1.0
            # 设置障碍物状态
            self.set_static_obs_state_req.state = self.set_obstacle_state
            self.set_gazebo_model_state(self.set_static_obs_state_req)
            prev_obstacle_positions.append((x, y))

    def check_dead_zone(self, x, y):
        """检查(x, y)是否在占用空间内"""
        dead_zone = False
        if abs(x) > self.upper or abs(y) > self.upper:
            dead_zone = True
        elif 2.0 < abs(x) < self.upper and abs(y) < 1.0:
            dead_zone = True
        elif abs(x) < 1.0 and 2.0 < abs(y) < self.upper:
            dead_zone = True
        return dead_zone

    def publish_markers(self, action):
        """发布可视化数据供Rviz显示目标和机器人动作"""
        marker_specs = [
            {
                "frame_id": "odom",
                "marker_type": Marker.CYLINDER,
                "scale": (0.1, 0.1, 0.01),
                "color": (1.0, 0.0, 1.0, 0.0),
                "position": (self.goal_x, self.goal_y, 0.0),
                "orientation": (0.0, 0.0, 0.0, 1.0),
                "action": Marker.ADD,
                "ns": "",
                "marker_id": 0,
                "publisher": self.goal_point_marker_pub,
            },
            {
                "frame_id": "odom",
                "marker_type": Marker.CUBE,
                "scale": (abs(action[0]), 0.1, 0.01),
                "color": (1.0, 1.0, 0.0, 0.0),
                "position": (5.0, 0.0, 0.0),
                "orientation": (0.0, 0.0, 0.0, 1.0),
                "action": Marker.ADD,
                "ns": "",
                "marker_id": 1,
                "publisher": self.linear_vel_marker_pub,
            },
            {
                "frame_id": "odom",
                "marker_type": Marker.CUBE,
                "scale": (abs(action[1]), 0.1, 0.01),
                "color": (1.0, 1.0, 0.0, 0.0),
                "position": (5.0, 0.2, 0.0),
                "orientation": (0.0, 0.0, 0.0, 1.0),
                "action": Marker.ADD,
                "ns": "",
                "marker_id": 2,
                "publisher": self.angular_vel_marker_pub,
            },
        ]
        for spec in marker_specs:
            marker = self.create_marker(**spec)
            marker_array = MarkerArray()
            marker_array.markers.append(marker)
            spec["publisher"].publish(marker_array)

    @staticmethod
    def create_marker(**kwargs):
        """创建用于可视化发布的标记"""
        marker = Marker()
        marker.ns = kwargs.get("ns", "")
        marker.id = kwargs.get("marker_id", 0)
        marker.header.frame_id = kwargs.get("frame_id", "odom")
        marker.type = kwargs.get("marker_type", Marker.CYLINDER)
        marker.action = kwargs.get("action", Marker.ADD)
        marker.scale.x, marker.scale.y, marker.scale.z = kwargs.get(
            "scale", (0.1, 0.1, 0.01)
        )
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = kwargs.get(
            "color", (1.0, 0.0, 1.0, 0.0)
        )
        (
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ) = kwargs.get("position", (0.0, 0.0, 0.0))
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = kwargs.get("orientation", (0.0, 0.0, 0.0, 1.0))
        return marker

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        """根据当前状态和采取的动作计算奖励"""
        if target:
            return 100.0
        if collision:
            return -100.0
        obstacle_reward = (min_laser - 1) / 2 if min_laser < 1.0 else 0.0
        action_reward = action[0] / 2 - abs(action[1]) / 2 - 0.001
        return action_reward + obstacle_reward


def main(args=None):
    # 初始化ROS2通信
    rclpy.init(args=args)
    # 创建环境节点
    environment = EnvironmentMyRobot()
    # 使用MultiThreadedExecutor并行处理两个传感器回调
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(environment)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        environment.get_logger().info("gym_node_myrobot 正在关闭...")
        environment.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
