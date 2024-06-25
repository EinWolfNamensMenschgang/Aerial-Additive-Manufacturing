# Aerial-Additive-Manufacturing


## How to install on Ubuntu 22.04:
Install ROS2 (https://docs.ros.org/en/humble/Installation.html)

Git clone the package into the src folder of your workspace

Source your installation and your workspace

Follow the steps from this https://github.com/TIERS/tello-ros2-gazebo and this https://github.com/ptrmu/fiducial_vlam repo to install the dependencies. When using ROS2 Humble, either clone the files from this repo in your workspace instead or make the necessary changes in the CMakeList.txt files.

In your workspace folder run `colcon build`

Run the package with `ros2 run gcode_to_path PathFromGcode <path-to-your-gcode-file>`

The node will publish a nav_msgs/Path message to /nav/path and a custom PrintPath message to /print/path where every point of the Path also has a bool value 'print'. If print is true, material should be extruded while moving to the point. 


Source your ros installation and colcon build.

## How to install using Docker (recommended)
Install Docker, Visual Studio Code and the Remote Development Extensions following this tutorial https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html. Download the ws folder in this repository. Click View->Command Palette...->Dev Containers: Open Folder in Container... in VSCode and select the ws folder. It will start building your Ubuntu 22.04 environment, ignore error messages as long as the container builds. To install the dependencies follow these steps:
```
source /opt/ros/humble/setup.bash
cd /home/ws 
sudo chown -R <USER> . 
colcon build 
source ./install/setup.bash 
sudo apt update 
rosdep update 
rosdep install --from-paths src --ignore-src -r -y 
sudo apt install libasio*
colcon build
source ./install/setup.bash
```
Make sure the Display variable in your Docker environment is correctly set, see the hint at the bottom of the Docker tutorial linked above.

## How to prepare your GCode and run the GCode to ROS nav_msgs/Path converter node:
Create a CAD file of the desired print geometry and save it as an STL file.
Slice STL file (Tested with open source https://ultimaker.com/software/ultimaker-cura , used Marlin Flavour), you can create your own printer settings to have a larger print area or change parameters like layer height. Run 'ros2 run gcode_to_path PathFromGcode --ros-args -p gcode_path:="<Path to your GCode>"' to publish the path and a boolean message if the drone should be currently extruding or not. 

## Starting the Simulation

`ros2 launch tello_gazebo vlam_launch.py`starts the gazebo simulation with three Tellos localizing themselves relative to fiducial markers. To make them takeoff `ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"` has to be called (in total three times with the namespace changed to /drone2 and /drone3). 
The nodes can be run with:
`ros2 run print_controller PrintVisualization`
`ros2 run print_controller PrintController`
`ros2 run print_controller PayloadPositionPublisher`

Some topics maybe have to be remapped, depending on your usecase.

## Creating the map
'ros2 run print_controller MapGenerator' creates a map for the fiducial vlam with the marker with ID 0 starting at the top left and the offsets to the other markers specified in the programm. Currently it creates a map for 2x5 markers directly next to each other with the IDs from 0 to 9. Alternatively, you can print the A2 marker poster in this repository, which is already stored as a map.  

## Controlling the real drone
Connect to the Tello wifi, which appears after turning it on. Run 'ros2 launch print_controller start.py'. This launches the Tello Driver, Fiducial Vlam, and Print Controller. Make sure to change the map and GCode file paths in the launch file according to your environment.  Once the video stream of the tello is displayed run `ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"` to make the Tello take off and follow the path. `ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}" makes it land again. 

## Changing hyperparameters
Hyperparameters like the p-value of the p-controller or the minimum distance to a point to consider it reached are defined at the very top of PrintControllerSingle.cpp. Creating a dynamic reconfigure node to change these values during run time is still work in progress.


