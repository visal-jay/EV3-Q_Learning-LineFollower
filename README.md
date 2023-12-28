# EV3 Q_Learning LineFollower

This project is focused on implementing a Q-learning based line-following robot using the LEGO Mindstorms EV3 platform. The robot navigates paths by learning to follow lines using a reinforcement learning algorithm, adapting to its environment and improving over time.


### Features

- **Q-Learning Algorithm**: Implements a basic Q-learning framework for decision-making.
- **Line Following**: Uses light sensor readings to navigate along a path.
- **Obstacle Avoidance**: Includes basic strategies for avoiding unexpected obstacles.

## Getting Started

### Prerequisites

- LEGO Mindstorms EV3 Kit
- Computer with Python and the ability to flash MicroPython onto the EV3 Brick

### Running the Program

1. Connect to your EV3 Brick via your preferred method (Bluetooth, USB, etc.).
2. Navigate to the folder containing the scripts.
3. Run the main program: `python3 main.py`

## Usage

After starting the script, the EV3-QLineFollower will begin to learn how to follow a line. Designed specifically for a robot equipped with one color sensor, it aims to navigate the midpoint between black and white surfaces. Position it on a path that has a distinct line, and it will commence its journey. Robot will make informed decisions utilizing the Q-learning algorithm, progressively enhancing its navigation skills over time.

## Customization

Feel free to dive into the code and tweak parameters like the learning rate (ALPHA), discount factor (GAMMA), or exploration rate (EPSILON), decaying rate (TEMP) to see how they affect the robot's learning process.

## Demo
https://github.com/visal-jay/EV3-Q_Learning-LineFollower/assets/73787312/c41079d9-bd0d-4d28-a769-1dd8bf13a95d

