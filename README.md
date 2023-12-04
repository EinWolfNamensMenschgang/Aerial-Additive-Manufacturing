# Aerial-Additive-Manufacturing



## GCode to ROS nav_msgs/Path converter node

### How to prepare your GCode:

Slice STL file (Tested with open source https://ultimaker.com/software/ultimaker-cura , used Marlin Flavour)

## How to install:
Install ROS2 (https://docs.ros.org/en/humble/Installation.html)

Git clone the package into the src folder of your workspace

Source your installation and your workspace

In your workspace folder run colcon build

Run the package with ros2 run gcode_to_path PathFromGcode <path-to-your-gcode-file>

The node will publish a nav_msgs/Path message to /nav/path and a custom PrintPath message to /print/path where every point of the Path also has a bool value 'print'. If print is true, material should be extruded while moving to the point. 
