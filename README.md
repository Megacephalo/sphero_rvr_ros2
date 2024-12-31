## About this project

The is my implementation of Sphero RVR+ ROS 2 package which consists of a ROS2 wrapper around the existing [sphero-sdk-raspberrypi-python](https://github.com/sphero-inc/sphero-sdk-raspberrypi-python). This project's implementation is based on the [linorobot/sphero_rvr](https://github.com/linorobot/sphero_rvr) repository which in terms is inspired by the [sphero-sdk-raspberrypi-python](https://github.com/sphero-inc/sphero-sdk-raspberrypi-python) repository's [drive_rc_si_units.py](https://github.com/sphero-inc/sphero-sdk-raspberrypi-python/blob/master/getting_started/asyncio/driving/drive_rc_si_units.py) example code.

Currently it offers only the basic driving commands and does supposely provide feedback from the robot's IMU. Further investigation is underway to enable the feedback from the robot.

## Built with

- Tested on Linux Ubuntu 22.04 LTS 
- Python 3.12 and beyond
- ROS2 Humble
- pyserial 3.5
- Any development kit with UART

### Prerequisites

This project already contains the Sphero RVR SDK, the core driver library, so you do not need to sweat about it. Simply clone this repository to your local end, make sure to set up your Sphero RVR+ udev rule to enable read-write access to the corresponding port. Connect the computer that you will run the script on to your RVR+ robot and check if you can send an signal through to the robot's UART ports, not necessarily have to be in Python.

## Getting started

Once you have the UART communication sorted out, build the package as at the root directory of your ROS2 worksapce. For instance, if your workspace is at `~/ros2_ws/`, then 

```bash
cd ~/ros2_Ws
colcon build
# or building only this package
colcon build --packages-select sphero_rvr_ros2
```
### Installation

Then source the environment such that ROS2 file system can find your package:

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash
```
especially the second line, this command makes sure to register this package into the file system. Note if you want to make any changes to the code base, you need to build it with the command above as the package is installed under the `install/` directory under the workspace and that is where all scripts will be executed. 

## Usage

There is one node: `rvrp_node.py`, so go ahead and execute the node with the command:

```bash
ros2 run sphero_rvr_ros2 rvrp_node
```

If you have the robot properly plugged in and computer well connected, you should be able control the robot's motor with the `/cmd_vel/` topic messages.

A quick way to teset is to teleoperate the robot with an off-the-shelf keyboard speed controller node such as **ros-humble-teleop-twist-keyboard**. To use it, first install its debian

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

Then bring it up in a ROS 2 fashion:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# to which you will have a dashboard
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
```

simply press the "u", "i", "o", "j", "k", "l", "m", ",", "." keys to publish the cmd_vel messages. If the `rvrp_node` is active and within the same network, then it will drives the robot's motors and you should see the wheels spinning in no time.If you can control the robot then you are good to go.

## Roadmap

- [x] Basic cmd_vel to Sphero motor control commands
- [ ] Publish actual feedback sensor signals from robot

## Conributing

I am happy if you want to contribute in any way as long as it will keep the code base cleaner, leaner, and more efficient.

### Top contributers

- [Charly Huang](mailto:charly.charlongo@gmail.com)

## License

Distributed under the license. See `LICENSE.txt` for more information.

## Acknowledgement

- [Linorobot](https://linorobot.org/) for open-sourcing their implementation of the most essential code implementation. 
