# ROS2 Robot Control Crash Course, with Flatland

## Installation and Environment Setup

Note: This guide assumes you are using an Ubuntu 20.04 operating system, and can be followed using either a computer running on this OS, WSL, or a Virtual Machine.

### ROS Installation

Follow the instructions in the official [ROS installation guide](http://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). It is recommended to at least install the desktop version, as some tools that will be used here are already contained within.

In this tutorial it is assumed that your ROS installation is placed in /opt/ros/, under the name foxy (which is the second most recent Long Term Support distribution at the time of writing).

Note: From this point on, it is assumed that every terminal has sourced the _setup.bash_ script as indicated in the Environment Setup section of the installation guide. Without this, you will not be able to use ROS or any of its tools. If you moved the installation to _/opt/ros/foxy/_, the command to source it becomes:
```
source /opt/ros/foxy/setup.bash
```

Instead of having to write this command everytime a new terminal is opened, the command can be written onto the .bashrc script, so that it is run automatically when opening a new terminal:
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Some additional ROS2 tools and the colcon build system will be required for this tutorial to function properly. You can install them with:
```
sudo apt install ros-foxy-rviz2 ros-foxy-navigation2 ros-foxy-nav2-bringup
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Workspace Setup

Create your own workspace. Pick where you would like your workspace to be placed (usually in the home directory, a.k.a. _~/_) and create a folder with a name of your choice, for example _ros\_workspace_, and within it create a folder named _src_. Finally, from within the workspace folder, in this case the ros_workspace folder, run the _colcon build_ command. You can use the following commands in a terminal for this process, assuming you place your workspace in your home directory:

```
mkdir -p ~/ros_workspace/src
cd ~/ros_workspace/
colcon build
```

Three new folders should have appeared in your workspace, _build/_, _install/_ and _log/_. Within the _install/_ folder, you'll find multiple shell scripts. One of these, _install/setup.bash_ will be used to allow you to run code from packages within your workspace. After you run _colcon build_, don't forget to execute the following command:

```
source install/setup.bash
```

**Important Note**: Every time you open a new terminal to run ROS code from within your workspace, you must first source ROS's _setup.bash_ script and your workspace's own _setup.bash_. The command to source ROS's _setup.bash_ is already being automatically run, if you added it to _~/.bashrc_, as indicated in the _ROS Installation_ section. In order to avoid having to source the setup.bash of the workspace everytime a new terminal is opened, similarly to what was done for sourcing ROS's setup.bash, the source command can be added to the .bashrc file:
```
echo "source ~/ros_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Flatland Setup

First, clone the flatland repository into the _src_ folder of your workspace, check out the ros2-plugins-port-broken branch and install its dependencies (you may have to install LUA separately if you don't already have it). Then, from within your workspace, build the package with _colcon build_ (and source install/setup.bash), and launch the default server to test whether everything is working. You can do this process with the following commands:

```
cd src/
git clone https://github.com/avidbots/flatland.git
cd flatland
git checkout origin/ros2-plugins-port-broken
cd ../..
sudo apt-get update
sudo apt-get install liblua5.1-0-dev
rosdep install --from-paths src --ignore-src
colcon build
source install/setup.bash
```

If all went well, you should see some feedback on the console from the build process.

## First Run

Now that you have ROS and Flatland set up, you can start experimenting with the code from this tutorial. First clone the repository and build it:
```
cd src/
git clone https://github.com/BerserkingIdiot/flatland_quick_start_ros2.git
cd ..
colcon build
source install/setup.bash
```
You can launch the example with RViz visualization. RViz is complex and requires manual setup for each new environment, but also flexible, in this case providing a visualization of the data generated by the LiDAR. To launch this you can use:
```
ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml
```
Once the simulation is running, in a separate terminal, run:
```
ros2 run nav2_util lifecycle_bringup map_server
```

This will launch a Flatland simulation, containing a small differential drive robot equipped with a LiDAR, and a few obstacles. The RViz visualization can be manipulated with the mouse, allowing you to drag, rotate, zoom and enable and disable certain components of the visualization. The second command allows the map to be visible in RViz by activating the _map\_server_ node (lifecycles are a new feature in ROS2 that allows for more deterministic code development but we won't delve into it in this tutorial). If all went well, you should see something like this:

![Screenshot from 2022-04-20 10-21-54](https://user-images.githubusercontent.com/38168315/164197521-1b16f6f1-7871-43c3-a3a0-69547c89ff2a.png)

You can zoom in the visualization window with the scroll wheel and move it around by using alt + left click and dragging. The front of the robot starts pointed towards the right and the robot looks like this:

![simplified_robot](https://user-images.githubusercontent.com/38168315/175785792-73b931b9-7859-42ff-8570-dd36f6a99a51.png)

## Write your own code

The simulation in this package provides a small differential drive robot equipped with a LiDAR. The differential drive is controlled through Twist messages, and the LiDAR provides data in the form of LaserScan messages. In this package, you will find some sample code that you can use to help develop your own robot controller.

Note: the LiDAR scan data is provided in the form of an array of ranges, each value corresponding to the nearest detected obstacle by that ray, or _nan_ if nothing is found. The rays are defined counter-clockwise, starting from the rear of the robot, meaning that in the first half of the array are the values for obstacles on the right and in the second half are values for obstacles on the left, with the middle of the array corresponding to the front of the robot. Playing around with the RViz visualization may help understanding how these work.

Three examples are provided in comments on the C++ and Python controllers. The GIFS below provide an example in RViz of the robot moving using each controller example.

Example 1: A robot that moves forward

![robot_ex1](https://user-images.githubusercontent.com/21350014/175800451-3afa03c3-36e8-4724-9af9-b2203945e6dc.gif)

Example 2: A robot that moves in a circle

![robot_ex2](https://user-images.githubusercontent.com/21350014/175800477-c88e3275-31aa-45ba-9cb4-8b87ef6cbbed.gif)

Example 3: A robot that moves in circles, and turns around itself when it finds an obstacle in front of it

![robot_ex3](https://user-images.githubusercontent.com/21350014/175800485-cad50162-4098-4471-80f2-5101af6772f3.gif)

### C++

Within the _src/_ folder of this package (not to be confused with the workspace's _src/_), you can find a file named _custom\_robot\_controller.cpp_. The path to this .cpp file is _/ros\_workspace/src/flatland\_quick\_start\_ros2/src/custom\_robot\_controller.cpp_.

You can use this code to write your own code, and experiment with robot control. Don't forget to build the package after you modify the code:
```
cd ~/ros_workspace/src/flatland_quick_start_ros2/src
nano custom_robot_controller.cpp
<edit your code>
cd ~/ros_workspace/
colcon build
```
To run the controller you must first make sure to have a simulation running. You can use one of the launch files from the previous section, for example, and then run the controller (with rosrun). For instance,the following commands can be used in two terminals:

In the first terminal (to start the robot and simulator):
```
ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml
```

In the second terminal (to launch the controller):
```
ros2 run flatland_quick_start_ros2 custom_robot_controller
```
 
### Python

Within the _scripts/_ folder of this package, you can find a file named _custom\_robot\_controller.py_. The path to this .py file is _/ros\_workspace/src/flatland\_quick\_start\_ros2/scripts/custom\_robot\_controller.py_.

You can use this code to write your own code, and experiment with robot control. Don't forget to build the package after you modify the code:
```
cd ~/ros_workspace/src/flatland_quick_start_ros2/scripts
nano custom_robot_controller.py
<edit your code>
cd ~/ros_workspace/
colcon build
``` 
To run the controller you must first make sure to have a simulation running. You can use one of the launch files from the previous section, for example, and then run the controller (with rosrun). For instance,the following commands can be used in two terminals:

In the first terminal (to start the robot and simulator):
```
ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml
```

In the second terminal (to launch the controller):
```
ros2 run flatland_quick_start_ros2 custom_robot_controller.py
```

## Next Steps

### Topics, Publishers and Subscribers

In this tutorial, you were provided with a ROS node with the communication features already set up, and you might have noticed elements like the LiDAR scan subscriber and the Twist message publisher. These are essential elements of the ROS environment, and you can learn more about them in the [official ROS tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

### Exploring Flatland

In this package, there are currently 2 different maps to experiment with, found in the _flatland\_worlds/_ folder. The launch files default to the office test world, but you can use the world_path parameter to choose the world you wish to boot up, for example to boot up the maze world you can run this:
```
ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml world_path:="maze" 
```

To learn more about the Flatland simulator, read the [documentation and tutorials](https://flatland-simulator.readthedocs.io/en/latest/)

## Feedback

Once you have finished following the tutorial, please fill out the following form: https://forms.gle/Pw1brvNvAmvSGhQr9
