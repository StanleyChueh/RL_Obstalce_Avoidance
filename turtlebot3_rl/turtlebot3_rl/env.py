import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
from .debug_publisher import DebugPublisher

class TurtleBot3Env(gym.Env):
    def __init__(self):
        super(TurtleBot3Env, self).__init__()

        # Define a range for LIDAR data that covers the front, left, right, and rear sides
        self.full_indices = list(range(0, 360))  # Covers the full 360 degrees

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.node = rclpy.create_node('turtlebot3_rl_env')

        # Initialize Debug Publisher
        self.debug_publisher = DebugPublisher()

        # Action space: linear and angular velocity (Normalized to [-1, 1] range)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        # Observation space: Now includes front, left, right, and rear LIDAR data
        self.observation_space = spaces.Box(low=0.0, high=3.5, shape=(len(self.full_indices),), dtype=np.float32)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)  # Use a smaller queue size for faster response
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1  # Smaller queue size to prioritize the latest data
        )

        # Initialize scan data
        self.scan_data = np.zeros(len(self.full_indices), dtype=np.float32)
        self.last_action = np.zeros(2)  # Initialize last_action to avoid uninitialized errors

        # Use a single-threaded executor for simpler, faster node execution
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Start a separate thread to spin the ROS2 node
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

    def scan_callback(self, msg):
        # Capture the full 360-degree LIDAR data
        raw_scan_data = np.array(msg.ranges, dtype=np.float32)[self.full_indices]

        # Replace 0.0 values with a large number, indicating 'no detection'
        raw_scan_data[raw_scan_data == 0.0] = np.inf

        # Ensure min range reflects realistic sensor limits
        self.scan_data = np.clip(raw_scan_data, 0.12, 3.5)

    def reset(self, seed=None, options=None):
        # Reset the environment
        self.node.get_logger().info("Resetting the environment...")

        if seed is not None:
            self.seed_val = seed
            self._np_random, seed = gym.utils.seeding.np_random(seed)

        # Reset last_action to zero on reset
        self.last_action = np.zeros(2)

        return self.scan_data.astype(np.float32), {}

    def step(self, action):
        # Map normalized action back to original range
        linear_velocity = np.clip(action[0], -1.0, 1.0) * 0.2  # Adjusted speed for more effective obstacle avoidance
        angular_velocity = np.clip(action[1], -1.0, 1.0) * 0.2  # Adjust angular speed for better turning

        twist = Twist()
        twist.linear.x = float(linear_velocity)
        twist.angular.z = float(angular_velocity)
        self.cmd_vel_pub.publish(twist)

        # Log scan data for debugging
        self.node.get_logger().debug(f"Current scan data: {self.scan_data}")  # Use debug level to reduce logging overhead

        observation = self.scan_data.astype(np.float32)
        reward = self.calculate_reward(action, linear_velocity, angular_velocity)
        done = self.check_done()

        # Publish debug information
        self.debug_publisher.publish_debug_info(self.scan_data, action, reward, done)

        return observation, reward, done, False, {}

    def calculate_reward(self, action, linear_velocity, angular_velocity):
        # Focus on the full 360-degree sector for calculating rewards
        front_min = np.min(self.scan_data[:90] + self.scan_data[-90:])  # Front sector
        rear_min = np.min(self.scan_data[90:270])  # Rear sector

        # Initialize reward
        reward = 0.0

        # Strong penalty for collisions (front or rear)
        if front_min < 0.25 or rear_min < 0.25:
            reward = -5000.0  # Strong penalty for collision

        # Reward for moving forward safely
        if linear_velocity > 0 and front_min >= 0.5:
            reward += 20.0  # Encourage forward movement if safe

        # Reward for moving backward safely
        if linear_velocity < 0 and rear_min >= 0.5:
            reward += 20.0  # Encourage backward movement if safe

        # Penalty for proximity to obstacles in the front or rear sectors
        if front_min < 0.5 or rear_min < 0.5:
            reward -= 1500.0 * (0.5 - min(front_min, rear_min))  # Stronger penalty for being too close to obstacles

        # Penalize slow movements (to avoid the robot "creeping" forward too slowly)
        if abs(linear_velocity) < 0.05:
            reward -= 100.0  # Penalize very slow movements

        # Reward for turning away from obstacles
        if (front_min < 0.3 or rear_min < 0.3) and angular_velocity != 0:
            reward += 100.0  # Reward for turning away when too close to obstacles

        # Update last action
        self.last_action = action

        return reward

    def check_done(self):
        # Check the broader sector to determine if the episode is done
        front_min = np.min(self.scan_data[:90] + self.scan_data[-90:])  # Front sector
        rear_min = np.min(self.scan_data[90:270])  # Rear sector

        # End the episode if a collision is detected in front or rear
        if front_min < 0.25 or rear_min < 0.25:
            return True
        return False

    def close(self):
        self.executor.shutdown()
        self.spin_thread.join()
        self.debug_publisher.destroy_node()
        rclpy.shutdown()

