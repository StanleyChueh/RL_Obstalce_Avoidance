import gym
from stable_baselines3 import SAC
from turtlebot3_rl.env import TurtleBot3Env

def main():
    # Initialize the environment
    env = TurtleBot3Env()

    # Load the trained model
    model = SAC.load("sac_turtlebot3_final.zip")

    try:
        while True:  # Infinite loop
            obs, _ = env.reset()  # Unpack the observation from the tuple
            done = False
            total_reward = 0.0

            while not done:
                # The model predicts an action based on the observation
                action, _states = model.predict(obs, deterministic=True)
                
                # Apply the action in the environment
                obs, reward, done, _, info = env.step(action)  # Unpack all returned values
                
                # Accumulate the reward for this episode
                total_reward += reward

            print(f"Episode finished: Total Reward: {total_reward}")

    except KeyboardInterrupt:
        print("Testing interrupted by user. Exiting...")

    finally:
        # Close the environment properly
        env.close()

if __name__ == '__main__':
    main()

