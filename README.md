# onrobot
ROS onrobot meta-package

![Grippers](/onrobot_description/media/rg_grippers.png)


## Packages

### onrobot_description

This package contains the description of [OnRobot](https://onrobot.com/en/products) RG2 and RG6 grippers. The descriptions are based on the URDF format.

### onrobot_control

This package contains the config controller ile and the launh of  the ros2 controller for the gripper*

### onrobot_interface
This package implements the onrobot_gripper interface for interfacing RG2 or RG6 grippers attached to a Universal Robot (UR3, UR5, UR10) from ROS2. 


## Installation

Clone the repository:

```bash
git clone https://github.com/ikalevatykh/onrobot_ros.git
```

Build the package using `colcon`:

```bash
colcon build
```

Source the workspace setup file:

```bash
source install/setup.bash
```

## Usage

### Simulation

To visualize the gripper and check if the URDF is correct, run:

```bash
ros2 launch onrobot_description test.launch.py
```

To test the gripper standalone in simulation (with Gazebo), run:

```bash
ros2 launch onrobot_description test_gazebo.launch.py
```

### Real Hardware

To use the real gripper:

1. Integrate the gripper into your URDF setup with a Universal Robot.
2. Launch the `io_controller` on the UR.

Refer to the package documentation for detailed integration steps.


> **Important:**
> - The project supports grippers connected to a Universal Robot controlled by the [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
> - The package assumes a gripper in Teach mode (without the UR Caps OnRobot installed). See the *Teach Mode* section in the [gripper manual](https://www.universal-robots.com/media/1226143/rg2-datasheet-v14.pdf) for more details. In this mode, only two gripper positions are supported: fully open and fully closed.



