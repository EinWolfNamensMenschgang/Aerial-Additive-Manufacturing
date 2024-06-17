# Aerial-Additive-Manufacturing



## GCode to ROS nav_msgs/Path converter node

### How to prepare your GCode:

Slice STL file (Tested with open source https://ultimaker.com/software/ultimaker-cura , used Marlin Flavour)

## How to install:
Install ROS2 (https://docs.ros.org/en/humble/Installation.html)

Git clone the package into the src folder of your workspace

Source your installation and your workspace

In your workspace folder run `colcon build`

Run the package with `ros2 run gcode_to_path PathFromGcode <path-to-your-gcode-file>`

The node will publish a nav_msgs/Path message to /nav/path and a custom PrintPath message to /print/path where every point of the Path also has a bool value 'print'. If print is true, material should be extruded while moving to the point. 

Follow the steps from this https://github.com/TIERS/tello-ros2-gazebo and this https://github.com/ptrmu/fiducial_vlam repo to install the dependencies. When using ROS2 Humble, either clone the files from this repo in your workspace instead or make the necessary changes in the CMakeList.txt files.
Source your ros installation and colcon build.

## Starting the Simulation

`ros2 launch tello_gazebo vlam_launch.py`starts the gazebo simulation with three Tellos localizing themselves relative to fiducial markers. To make them takeoff `ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"` has to be called (in total three times with the namespace changed to /drone2 and /drone3). 
The nodes can be run with:
`ros2 run print_controller PrintVisualization`
`ros2 run print_controller PrintController`
`ros2 run print_controller PayloadPositionPublisher`

## Creating the map
'ros2 run print_controller MapGenerator' creates a map for the fiducial vlam with the marker with ID 0 starting at the top left and the offsets to the other markers specified in the programm. Currently it creates a map for 2x5 markers directly next to each other with the IDs from 0 to 9.

## Controlling the real drone
Connect to the Tello wifi, which appears after turning it on. Run 'ros2 launch print_controller start.py'.



