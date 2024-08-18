'''import gym
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from turtlebot3_rl.env import TurtleBot3Env

def main():
    # Initialize the environment
    env = TurtleBot3Env()
    check_env(env)  # Ensure that the environment adheres to the Stable Baselines3 API
    
    # Wrap the environment in a Monitor for logging
    env = Monitor(env)

    # Define the total number of timesteps for training
    timesteps = 1000000  # Define this before using it in the lambda function

    # Define the learning rate schedule
    learning_rate = lambda x: 0.0003 * (1 - x / timesteps)

    # Define the RL algorithm (SAC) with potential adjustments
    model = SAC("MlpPolicy", env, verbose=1, learning_rate=learning_rate, buffer_size=200000, learning_starts=1000, batch_size=128, tau=0.005, gamma=0.99, train_freq=4, gradient_steps=4, use_sde=True)

    # Callbacks for saving models and evaluation
    checkpoint_interval = 50000  # Define the interval for saving checkpoints
    checkpoint_callback = CheckpointCallback(save_freq=checkpoint_interval, save_path='./models/', name_prefix='sac_turtlebot3')
    eval_callback = EvalCallback(env, best_model_save_path='./models/', log_path='./logs/', eval_freq=checkpoint_interval, deterministic=True)

    # Train the model with checkpoints
    model.learn(total_timesteps=timesteps, reset_num_timesteps=False, callback=[checkpoint_callback, eval_callback])

    # Save the final model
    model.save("sac_turtlebot3_final")

    # Close the environment
    env.close()

if __name__ == '__main__':
    main()
'''
import gym
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from turtlebot3_rl.env import TurtleBot3Env

def main():
    # Initialize the environment
    env = TurtleBot3Env()
    check_env(env)  # Ensure that the environment adheres to the Stable Baselines3 API
    
    # Wrap the environment in a Monitor for logging
    env = Monitor(env)

    # Define the total number of additional timesteps for training
    additional_timesteps = 500000  # Define how many more timesteps you want to train

    # Load the existing model
    model = SAC.load("sac_turtlebot3_final.zip", env=env)

    # Callbacks for saving models and evaluation during continued training
    checkpoint_interval = 50000  # Define the interval for saving checkpoints
    checkpoint_callback = CheckpointCallback(save_freq=checkpoint_interval, save_path='./models/', name_prefix='sac_turtlebot3')
    eval_callback = EvalCallback(env, best_model_save_path='./models/', log_path='./logs/', eval_freq=checkpoint_interval, deterministic=True)

    # Continue training the model with additional timesteps
    model.learn(total_timesteps=additional_timesteps, reset_num_timesteps=False, callback=[checkpoint_callback, eval_callback])

    # Save the updated model
    model.save("sac_turtlebot3_final_finetuned")

    # Close the environment
    env.close()

if __name__ == '__main__':
    main()

