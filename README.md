# Inventory-Bot
An autonomous inventory bot using ROS, SLAM, and obstacle avoidance to navigate warehouses. It features a Python-controlled adjustable tray, allowing the bot to deliver inventory to specific drawers or trays at various heights. The bot's design optimizes warehouse efficiency with minimal human intervention.

## Features
- **ROS Integration**: Utilizes ROS for movement, navigation, and control.
- **Tray Movement**: Python script to raise and lower the tray based on the target position.
- **Navigation**: Python scripts that enable the bot to navigate to target locations and set initial poses.

## Usage
### 1. Set Initial Position
To set the initial pose of the robot based on the current position from Gazebo, use:
python set_initial_pose.py
### 2. Set Target Position 
To move the robot to a specific target location, use:
python move_bot.py <x_position> <y_position>
### 3. Move Tray
To move the tray to a specific z-position, use the following command:
python move_tray.py <target_position>

## Installation 
### 1. Clone the Repository
git clone <repository_url>
### 2. Build the Package
cd <workspace>
catkin_make
### 3. Source the Workspace
source devel/setup.bash

## Requirements
- ROS (Robot Operating System)
- Python 2 or 3
- Gazebo (for simulation)
