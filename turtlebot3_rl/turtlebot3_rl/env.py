import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading

class TurtleBot3Env(gym.Env):
    def __init__(self):
        super(TurtleBot3Env, self).__init__()

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.node = rclpy.create_node('turtlebot3_rl_env')

        # Action space: linear and angular velocity
        # Reduced linear velocity for more cautious movement
        self.action_space = spaces.Box(low=np.array([-0.1, -0.05]), high=np.array([0.1, 0.05]), dtype=np.float32)

        # Observation space: laser scan data
        self.observation_space = spaces.Box(low=0.0, high=3.5, shape=(360,), dtype=np.float32)

        # Publishers and subscribers
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # Queue size
        )

        # Initialize scan data
        self.scan_data = np.zeros(360, dtype=np.float32)

        # Use the executor to manage the node spinning
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        # Start a separate thread to spin the ROS2 node
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

    def scan_callback(self, msg):
        # Handle laser scan data
        raw_scan_data = np.array(msg.ranges, dtype=np.float32)

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

        return self.scan_data.astype(np.float32), {}

    def step(self, action):
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_vel_pub.publish(twist)

        # Log scan data for debugging
        self.node.get_logger().info(f"Current scan data: {self.scan_data[:10]}")  # Log the first 10 laser scan values

        observation = self.scan_data.astype(np.float32)
        reward = self.calculate_reward()
        done = self.check_done()

        # Log the action and the corresponding observation
        self.node.get_logger().info(f"Action: {action}, Min Distance: {np.min(self.scan_data)}, Reward: {reward}")

        return observation, reward, done, False, {}

    def calculate_reward(self):
        min_distance = np.min(self.scan_data)

        # Collision penalty
        if min_distance < 0.2:
            return -300.0  # High penalty for collision

        # Reward for moving forward
        forward_movement_reward = 1.0  # Maintain a modest reward for moving forward

        # Penalize proximity to obstacles more heavily
        proximity_penalty = -50.0 * (0.5 - min_distance) if min_distance < 0.5 else 0.0

        # Reward for maintaining a safe distance
        safe_distance_reward = 15.0 if min_distance > 0.5 else 0.0

        # Small reward for each step the robot avoids collision (time-based reward)
        time_step_reward = 0.2  # Encourage staying in a safe state over time

        # Combine rewards and penalties
        reward = forward_movement_reward + proximity_penalty + safe_distance_reward + time_step_reward

        return reward

    def check_done(self):
        min_distance = np.min(self.scan_data)
        if min_distance < 0.2:
            return True
        return False

    def close(self):
        self.executor.shutdown()
        self.spin_thread.join()
        rclpy.shutdown()

def main():
    rclpy.init(args=None)
    env = TurtleBot3Env()

    # Example test loop
    for _ in range(100):  # Run for 100 steps as an example
        action = env.action_space.sample()  # Random action
        observation, reward, done, _, _ = env.step(action)
        if done:
            env.reset()

    env.close()

if __name__ == '__main__':
    main()
