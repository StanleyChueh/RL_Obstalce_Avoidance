import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from turtlebot3_rl.env import TurtleBot3Env
import numpy as np
np.bool = np.bool_


def main():
    env = TurtleBot3Env()
    check_env(env)

    # Define the RL algorithm (PPO in this case)
    model = PPO("MlpPolicy", env, ent_coef=0.01, learning_rate=0.000001, n_steps=2048, batch_size=64, n_epochs=10, clip_range=0.1, verbose=1)

    # Train the model
    model.learn(total_timesteps=1000000)#50000
    
    # Save the model
    model.save("ppo_turtlebot3")

    # Close the environment
    env.close()

if __name__ == '__main__':
    main()
