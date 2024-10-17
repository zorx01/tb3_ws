# TurtleBot3 Deep Reinforcement Learning (Waffle Pi) Environment

This repository contains a deep reinforcement learning (DRL) environment specifically designed for the TurtleBot3 Waffle Pi. The environment supports both DQN and DDPG algorithms and allows you to train, test, or load pre-trained models in a Gazebo simulation.

## Prerequisites

Before setting up the environment, ensure you have ROS 2 Humble installed on your system. Follow these steps to prepare your environment:

1. **Download and execute the ROS 2 Humble installation script**:

   Run the following command to install ROS 2 Humble:

   ```bash
   curl -OL https://raw.githubusercontent.com/zorx01/scripts/main/setup_system.sh
   ```

   Then, execute the script:

   ```bash
   bash setup_system.sh
   ```

2. **Remove Existing TurtleBot3 Packages**:

   To avoid conflicts, first remove any existing TurtleBot3 packages:

   ```bash
   sudo apt remove ros-humble-turtlebot3-msgs
   sudo apt remove ros-humble-turtlebot3
   ```

## Setup Instructions

### 1. Clone the Repository

Navigate to your home directory and clone the repository:

```bash
cd ~
git clone https://github.com/zorx01/tb3_ws.git
```

### 2. Build the Workspace

Navigate to the workspace and build it:

```bash
cd ~/tb3_ws
colcon build --symlink-install
```

### 3. Source the Workspace

After building, source the workspace to use the packages. Either run the following command in every new terminal session:

```bash
source install/setup.bash
```

Or, to make the sourcing permanent, add it to your `.bashrc`:

```bash
echo "source ~/tb3_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Launch Gazebo Simulation

To start the Gazebo simulation, open a new terminal and run:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage9.launch.py
```

### 5. Run the Environment

In a **new terminal**, run:

```bash
ros2 run turtlebot3_drl environment
```

### 6. Run the Goal Publisher

In a **new terminal**, run:

```bash
ros2 run turtlebot3_drl gazebo_goals
```

### 7. Train an Agent

In a **new terminal**, you can train the agent using either DQN or DDPG. To train with DQN, run:

```bash
ros2 run turtlebot3_drl train_agent dqn
```

To train with DDPG, run:

```bash
ros2 run turtlebot3_drl train_agent ddpg
```

### 8. Test Trained Weights

To test your trained agent, run the following command in a **new terminal**:

```bash
ros2 run turtlebot3_drl test_agent dqn <saved weights folder name> <episode>
```

- The folder name for saved weights can be found in the `src/turtlebot3_drl/model<yourusername>/` directory.
- The episode number is the highest episode trained inside the folder.

### 9. Load Pre-Trained Weights

To load the pre-trained weights included in this repository, follow these steps:

1. Launch the Gazebo simulation in a **new terminal**:

   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_drl_stage9.launch.py
   ```

2. Run the environment in a **new terminal**:

   ```bash
   ros2 run turtlebot3_drl environment
   ```

3. Run the goal publisher in a **new terminal**:

   ```bash
   ros2 run turtlebot3_drl gazebo_goals
   ```

4. Load and test the pre-trained weights (e.g., DQN weights at episode 8600) in a **new terminal**:

   ```bash
   ros2 run turtlebot3_drl test_agent dqn 'trained/dqn_9' 8600
   ```

The pre-trained weights are located in the `/src/turtlebot3_drl/trained/` directory.

## Notes

- The trained models you save during training can be found inside `src/turtlebot3_drl/model<yourusername>/`.
- Make sure to use a **new terminal** for each step where indicated.
