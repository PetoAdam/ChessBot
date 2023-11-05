# Chess-Playing KUKA Robot (ROS 2 + Webots)

This repository contains the software stack for a KUKA LBR3R760iisy robot designed to play chess. The project combines the power of ROS 2's control system with a Webots simulation environment to create a chess-playing robot.

![Chess-Playing Robot](docs/resources/chess_robot.png)

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)

## Introduction

Chess-Playing KUKA Robot is a ROS 2 project that demonstrates the capabilities of the [ros2_control](https://index.ros.org/p/ros2_control/) framework for robot control. It combines the control software with a Webots simulation environment to simulate and play chess.

Key components of this project:
- **motion_planner ros2 package**: The project uses the `moveit2` framework to calculate the inverse kinematics and the trajectories for the movement of the robot for chess piece manipulation.
- **controller ros2 package**: The project uses the `ros2_control` framework to define the robot's hardware interfaces and controllers.
- **Webots Simulation**: The KUKA robot is simulated in the Webots environment, allowing it to interact with the chessboard and chess pieces.

## Prerequisites

Before you begin, ensure you have the following prerequisites:

- [ROS 2](https://index.ros.org/doc/ros2/Installation/): Install ROS 2 Humble according to your system's requirements.
- [Webots](https://cyberbotics.com/): Download and install Webots, as it's the simulation environment used for the project.
- A system with the necessary hardware requirements to run the simulation.

    Note: You can use the [install_dependencies.sh](install_dependencies.sh) script to install all the neccessary dependencies for Ubuntu. (TODO)

## Installation

To install and run the chess-playing robot, follow these steps:

1. Clone this repository:
   ```bash
   git clone https://github.com/PetoAdam/chessbot.git
   cd chessbot
   ```
2. Build the ros2 workspace:
    ```bash
    cd ROS2
    colcon build
    source install/setup.bash
    ```
    TODO: might need mixin for moveit

## Usage

To run the chess-playing robot, use the following commands from the repository's root directory:
```bash
# Start the Webots simulation
cd Webots
python3 webots_launch.py default.wbt LBR3R760iisy

# Start ROS 2 control
cd ROS2
colcon build
source install/setup.bash
ros2 launch chessbot lbr3r760iisy_controller.launch.py

# TODO:
# Start inverse kinematics solver (MoveIt)

# TODO: add chess playing stuff...

## TEST ##
# To test the movement, you can run this script from the ROS2/src/controller directory:
./trajectory.sh
# It currently tests if your setup was successful and makes the robot move slightly right and left
# To test the gripper:
ros2 topic pub /gripper_controller/gripper_command std_msgs/msg/Float64 "{data: 0.5}"
```

Note: Webots also has a streaming viewer option, which can be accessed on port 1234 of the machine running it. For example in the case of localhost: http://localhost:1234/index.html

## Contributing

We welcome contributions to this project. If you find any issues or have suggestions for improvements, please create a GitHub issue or submit a pull request. We value your input!

---

Enjoy your chess-playing KUKA robot powered by ROS 2 and Webots! If you have any questions or need assistance, please don't hesitate to reach out to us.