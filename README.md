# Move in Shapes ROS2 Project for turtlebot

This ROS2 project makes turtlebot move in custom circle or polygon shape and works in both 2D and 3D simulations. It includes a motion controller that allows the user to publish commands and lanchfile to start all necessary nodes.

## Prerequisites

- ROS2 Humble
- Turtlesim for 2D simulation. Can be installed via:
    ```bash
    sudo apt install ros-humble-turtlesim
    ```
- and/or:

    Turtlebot4 simulation for 3D simulation (installation tutorial can be found [here](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) )

## Installation

1. Clone the repository into your workspace src folder:

    ```bash
    git clone https://github.com/yourusername/move_in_shapes.git
    ```

2. Build the ROS2 workspace:

    ```bash
    colcon build
    ```

## Usage

### Launching the Simulation

To launch the simulation, use the provided launch file. For 2D simulation:

```bash
ros2 launch move_in_shapes motion_controller_launch.py
```

For 3D simulation:

```bash
ros2 launch move_in_shapes motion_controller_launch.py launch_3d:=true
```

#### You can also launch components separately:

- Launching 2D simulation:
```bash
ros2 run turtlesim turtlesim_node
```
- Launching 3D simulation:
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```
- move_in_shapes node accepts messages to continuously move in circle or poligon and accespts actions to complete one circle or one polygon
```bash
ros2 run move_in_shapes move_in_shapes # for 3d simulation launch
ros2 run move_in_shapes move_in_shapes --ros-args -p namespace:=/turtle1 # for 2d simulation launch
```
- shape_motion_controller node accepts user input from stdin and sends requests to move_in_shapes_node
```bash
ros2 run move_in_shapes shape_motion_controller
```

### Controlling the Turtlebot

   The `shape_motion_controller` node allows user input for controlling the turtlebot. It provides options to publish circle and polygon commands, execute circle and polygon actions, and quit the program.

   Alternatively, you can write your own node to send meassages to move_in_shapes node or to publish them from terminal
