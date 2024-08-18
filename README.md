# RL_Obstalce_Avoidance
Welcome to the TurtleBot3 Reinforcement Learning project! This guide will help you set up your environment, train a model, and test the trained model using the provided scripts. Follow the steps below to get started.

## System Requirements
Operating System: Ubuntu 20.04 (Recommended)

ROS2 Distribution: ROS2 Foxy

Python Version: Python 3.8+

## ROS2 and Gazebo & Python dependencies
```
# Install ROS2 Foxy
sudo apt update && sudo apt install ros-foxy-desktop

# Install ROS2 dependencies for TurtleBot3 and Gazebo
sudo apt install ros-foxy-geometry-msgs ros-foxy-sensor-msgs ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash

# Install Python libraries
pip install gymnasium stable-baselines3[extra] numpy pandas numexpr
```
## Setup TurtleBot3 Environment
### TurtleBot3 Packages
```
sudo apt install ros-foxy-turtlebot3*
```
### Clone and Build the Workspace
```
# Navigate to your workspace
cd ~
mkdir rl_ws
cd rl_ws
mkdir src
cd src

# Clone your repository (assuming your repository is named turtlebot3_rl)
git clone https://github.com/StanleyChueh/RL_Obstalce_Avoidance.git

# Build the workspace
cd ~/rl_ws
colcon build

# Source the workspace
source install/setup.bash
```
## Running the Code
### Training the Model
```
# Run the training script
ros2 run turtlebot3_rl train
```
### Testing the Model
```
# Run the testing script
ros2 run turtlebot3_rl test
```
### Continuing Training with an Existing Model
#### Load your existing model:
```
model = SAC.load("sac_turtlebot3_final.zip", env=env)
```
#### Load your existing model:
```
ros2 run turtlebot3_rl train
```
