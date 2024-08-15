from stable_baselines3 import PPO
from turtlebot3_rl.env import TurtleBot3Env
import numpy as np

def main():
    env = TurtleBot3Env()

    # Load the trained model
    model = PPO.load("ppo_turtlebot3")

    obs, _ = env.reset()
    for _ in range(20):  # Test for a small number of steps
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)
        print(f"Action: {action}, Min Distance: {np.min(obs)}, Reward: {reward}, Done: {done}")
        if done or truncated:
            obs, _ = env.reset()

    env.close()

if __name__ == '__main__':
    main()
