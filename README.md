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

### IsaacSim USD Scene File Generation
In onrobot_description/USD We have a USD file that can be used in IsaacSim (It was generated using IsaacSim 4.5 but tested on IsaacSim 6.0 with no issues)
To regenerate the the USD file:
1. Generate the urdf file with the absolute paths
```
cd onrobot_description
./USD/urdf_generator.sh
```
2. Import the generated onrobot_rg.urdf using IsaacSim 4.5 (We ran into issues when importing using IsaacSim 6.0. However the generated file run after couple of manual adjustments in IsaacSim 6)
3. Open the generated USD file using IsaacSim and go over all the joints and set the mimic joints properties below:
	- target (Reference joint) -> gripper_joint
	- natural frequency and damping -> 0 (enforce hard contraint)

### Real Hardware

To use the real gripper:

1. Integrate the gripper into your URDF setup with a Universal Robot.
2. Launch the `io_controller` on the UR.

Refer to the package documentation for detailed integration steps.

Voici une version corrigée et mise en forme en **Markdown**, avec une explication claire en anglais :

````markdown
## Shutting Down the Real Gripper without Turning Off the UR

When using the real gripper, if you want to shut it down **without shutting down the UR robot itself**, you can use the following ROS 2 service call:

```bash
ros2 service call /<prefix>io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 4, pin: 0, state: 0.0}"
````

* `fun: 4` means you are setting the **tool power supply** (i.e., controlling the tool output voltage).
* `pin: 0` corresponds to the **tool output voltage** control pin.
* `state: 0.0` disables the power supply to the gripper (turns it off).

> Replace `<prefix>` with the appropriate namespace or controller prefix used in your UR setup.

> **Important:**
> - The project supports grippers connected to a Universal Robot controlled by the [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
> - The package assumes a gripper in Teach mode (without the UR Caps OnRobot installed). See the *Teach Mode* section in the [gripper manual](https://www.universal-robots.com/media/1226143/rg2-datasheet-v14.pdf) for more details. In this mode, only two gripper positions are supported: fully open and fully closed.



