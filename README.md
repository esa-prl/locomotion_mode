# Locomotion Mode

## Overview

Locomotion modes used on ESA-PRL rovers should be derived from the base class contained in this package. The base class handles the interfacing with the [locomotion_manager](https://github.com/esa-prl/locomotion_manager), loading and parsing of the robot models located in [rover_config], subscription to the rover_motion_cmd and the joint_states and the publishing of the desired joint_commands.

Each locomotion mode requires these basic functionality. Thus, the implementation of all these interfaces can be reused and the only thing necessary to add a new locomotion mode is to program the mapping from rover velocities to joint velocities and positions.

Find here a list with already implemented locomotion modes as examples:
- [simple_rover_locomotion](https://github.com/esa-prl/simple_rover_locomotion)
- stop_mode (in this package)
- [TODO: Add wheel_walking package]()

**Keywords:** locomotion, library, package

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.com**

The locomotion_mode package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- ([rover_msgs]) (message definitions for ESA-PRL rovers)
- ([rover_config]) (config files and models for ESA-PRL rovers)
- [urdf](http://wiki.ros.org/urdf)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/locomotion_mode.git
	cd ../
	colcon build

## Usage

The locomotion mode package can not be used in isolation. Instead follow the following proceedure to create a new locomotion mode.

1. Create a new package for your new locomotion mode.
2. Create a class which inherits from `LocomotionMode` as shown in [simple_rover_locomotion](https://github.com/esa-prl/simple_rover_locomotion).
3. Populate `NEW_LOCOMOTION_MODE_CLASS_NAME::rover_velocities_callback` with your algorithm and publish the desired joint positions/velocities.

## Model files

Robot model located in [rover_config] `rover_config/urdf`

* **marta.xacro** Contains the 3D-model of MaRTA.

## Nodes

### locomotion_mode

This node should never be started without being inherited by a derived class.

#### Subscribed Topics

* **`/rover_motion_cmd`** ([geometry_msgs/Twist])

	Desired linear and angular velocity of rover.


* **`/joint_states`** ([sensor_msgs/JointState])

	Desired linear and angular velocity of rover.

#### Published Topics
* **`/joint_cmds`** ([rover_msgs/JointCommandArray])

	Desired joint positions and velocities of rover.

#### Services

* **`NODE_NAME/enable`** ([std_srvs/Trigger])
	This service enables the locomotion mode. The subscription to the topic `rover_motion_cmd` is activated. This service should only be called from the locomotion manager.

* **`NODE_NAME/disable`** ([std_srvs/Trigger])
	This service enables the locomotion mode. The subscription to the topic `rover_motion_cmd` is deactivated. This service should only be called from the locomotion manager.


### stop_mode_node

Inherited from `locomotion_mode.hpp` base class. Simply sends a 0 velocity command to all driving modes.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_config]: https://github.com/esa-prl/rover_config.git
[rviz]: http://wiki.ros.org/rviz
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[rover_msgs/JointCommandArray]: https://github.com/esa-prl/rover_msgs/blob/master/msg/JointCommandArray.msg