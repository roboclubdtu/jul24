<h1 align="center">DTU JUL24 Video - Source Code</h1>

This repository contains the code used for the 2024 DTU Elektro Christmas video. The aim is to use the Baxter robot as Santa Claus and make it hand over gifts. To accomplish this, this repository contains codes for:

- **Choreographer**: a node that reads instructions from a file (what to play at which time) and execute it
- **Resource Servers**: nodes that can be called from the choreographer through a set of standardized services and actions
  - **Pose Manager**: a node that manage the robot JointState. It subscribe to the robot output joint state and publish joint commands
- Scripts:
  - `baxter_init.sh` and `baxter_stop.sh`: scripts that manage the commands to configure and stop the Baxter robot,
  - `baxter_cmd_dupli`: to duplicate the joint command for one arm to the other arm topic
  - `baxter_head_cmd`: a Python GUI for sending head pan commands through a slider

---

<p align="center">
    Distributed under the MIT Licence<br>
    © <a href="https://github.com/Meltwin">Meltwin</a> 2024 for <a href="https://roboclubdtu.github.io/">DTU Roboclub</a> & <a href="https://electro.dtu.dk/">DTU Elektro</a>
</p>

## Dependencies

This repository is a ROS 1 package with the following known dependencies:

| Name | Version | Desc |
| :-: | :-: | :- |
| C++ | > C++17 | Most of the nodes are written with modern C++17 specifications in mind |
| CMake | > 3.5 | The toolchain used to compile this package is CMake |
| Python | > 3.8 | Some scripts are written in Python (3) |
| ROS | ROS1 Noetic | The targeted ROS version is ROS1 Noetic. It is unknwon if older distributions are supported. |
| Catkin | - | This repository use catkin as the compiling tool. |

| Python module | Version | Desc |
| :-: | :-: | :- |
| TKinter |  - | Used for GUI scripts |
| Numpy | - | Used for geometry computations in Python scripts |

| ROS Package | Desc |
| :-: | :- |
| roscpp | Used in nodes written in C++ (choreographer & resource servers) |
| rospy | Used in scripts written in Python (baxter_head_cmd) |
| std_msgs | Used for some topics of the Baxter robot |
| sensor_msgs | Using the JointState message to get the Baxter robot actual state |
| actionlib | Using action for playing resource sequences |
| message_generation | This package use custom services and thus need the message generation library |
| message_runtime | This package use custom services and this need the message runtime library |
| baxter_core_msgs | This package use custom Baxter messages to send joint commands. You can find them [here]([https://github.com/CentraleNantesRobotics/baxter_legacy/baxter_common](https://github.com/CentraleNantesRobotics/baxter_legacy/tree/main/baxter_common))|
| baxter_tools | This package uses tools written specificaly for Baxter in the `baxter_init.sh` and `baxter_stop.sh` scripts. You can find them [here](https://github.com/CentraleNantesRobotics/baxter_legacy) (with usage of python3 instead of python2) | 

This package was written and tested on an Ubuntu 22.04 amd64 computer with a ROS1 installation.

## Running the content of the repository

After loading all workspaces (Noetic installation, Baxter dependencies and this repository's workspace), you are ready to launch the several applications.

### Choereograph & Services

The main part of this repository is the choreographing environment. It uses a centralized approach around the choreograph node. Each resource server (nodes that implements the ResourceServer interface) that will instantiated will advertize some standard services and actions:

- Service `capture`: ask to capture the current resource
- Action `play`: ask the server to play a sequence of resources between two indexes
- Service `save`: ask the server to save the resources in memory to a file
- Service `load`: ask the server to load the resources in a file into the memory
- Service `info`: ask the server for the list of resources currently loaded in the memory

Moreover, this servers have several "stacks" implemented with each a custom name so that several sequences can be loaded at the same time. 

--- 

The **Choreographer** node is an application that will fetch all servers on the network and will call them according to a file. This file will be like:

```
capture js0
load js0 -c trajectory1 -f trajectory1.txt
<action> <resource name> <arguments>
...
```

To launch this node, use the following command:

```shell
rosrun dtu_jul24 choreo -f <filename>
```

Please note that you can also start the choreographer in terminal mode (i.e. where you enter the commands by yourself) simply by running:

```shell
rosrun dtu_jul24 choreo
```

Also, all filenames that you enter here or in the files are relative to where you launch the choreographer and not the servers.

--- 

The **ResourceServers** nodes are the executable that actually manage the robot / system. They implement the several actions and services listed above. For the moment the only servers that are implemented are:

- `pose_manager`: a server that only manage the Baxter joints (both arms + head pan)

You can launch the several servers by running (for a Baxter robot) the command below. It will take care of launching everything in one terminal, as well as the needed remappings to send the messages to the right topics.

```shell
roslaunch dtu_jul24 baxter_jul24.launch
```

### Baxter setup scripts

This package also contains two scripts to setup and stop the Baxter. It basically just manage the "enable" and "tuck_arms" scripts, as well as de-activate the head sonars. To run them, simply run:

- **Init**: `rosrun dtu_jul24 baxter_init.sh`
- **Stop**: `rosrun dtu_jul24 baxter_stop.sh`

### Manual commands

This package also provide scripts to manually interact with the robot.

- You can by default move the Baxter arms by grabbing the wrist (detect the presence of a hand).
- A script to move the head manually has been added. Just run `rosrun dtu_jul24 baxter_head_cmd` to display a GUI with a slider that will move the head.
