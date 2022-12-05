# TurtleBot3_Walker
Simple algorithm to avoid obstacles and move TurtleBot3 in a closed space using ROS2 Humble
<br>
This package is tested on ROS2 Humble with TurtleBot3 Burger - Ubuntu 20.04, Humble Source Installation. If you have ROS2 Humble already installed, please skip the installation steps.
## Installing ROS2 on Ubuntu 22.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html) to install ROS2 Humble on your new Ubuntu 22.04 Desktop (binary installation).
## Installing ROS2 on Ubuntu 20.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) carefully and select Ubuntu 20.04 wherever it is necessary and install from source.
## Tutorials
- Follow these ![tutorial](http://docs.ros.org/en/humble/Tutorials.html) to understand the basics of ROS2. If you have installed the ROS2 from source, the ros2 setup path will be your 
```
. <installation directory>/install/setup.bash
```
- Source installation installs turtlesim and other packages. No need to use the commands listed in the above tutorial. 
- `colcon` can be installed by running the command:
```
sudo apt install python3-colcon-common-extensions
```

## Dependencies
- Make sure that you have TurtleBot3 packages installed from ![TB3 ROBOTIS Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup). In this tutorial, replace foxy with humble.
- To get the gazebo_ros_packages, follow the instructions from ![Gazebo](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallGazebo) manual and replace foxy with humble.

- Required packages to be in the source directory if installed from source:
1. DynamixelSDK
2. gazebo_ros_packages
3. turtlebot3
4. turtlebot3_msgs
5. turtlebot3_simulations
6. vision_opencv


## Clone and Build the package
- Clone this repository in your ROS2 Workspace /src folder. It should create TurtleBot3_Walker folder in your /src.
```
git clone https://github.com/okritvik/TurtleBot3_Walker.git
```
Make sure that your terminal is sourced
- Run the below commands:
```
cd TurtleBot3_Walker
rosdep install -i --from-path src --rosdistro humble -y
cd ../.. # It should take your present working directory to <ROS2_workspace>
colcon build --packages-select obstacle_avoidance
. install/setup.bash
```

## Set the Gazebo model path and TurtleBot3 model
- Run the below commands in the terminal you wish to run the obstacle avoidance.
```
export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=burger
```

## Commands to source a new Terminal window
- When a new terminal is opened, please source your terminal using
```
. <ros2_installation_directory>/install/setup.bash
. install/setup.bash
```

## Using ROS2 Bag files to store the published data
- A launch file is created in the ```launch``` directory that calls the TurtleBot3 gazebo world and obstacle avoidance node.
- Note that walker_bag should not be in your directory to launch with recording. Delete the folder if it exists. 
- Command to launch with recording
```
ros2 launch obstacle_avoidance launch_walker.py record_flag:=True
```

- Command to launch without recording
```
ros2 launch obstacle_avoidance launch_walker.py record_flag:=False
```

- Run this for about 15 seconds and press ctrl+c to store the data published. 
- To play the bag file, run the below command
```
ros2 bag play walker_bag
```

## Issues encountered
- If GAZEBO_MODEL_PATH and TURTLEBOT3_MODEL paths are not set, the launch file will not work.
- The required dependencies should be installed prior to run this package. Otherwise the ros2 launch command will throw errors.
- Might encounter errors in the installation of dependent packages using the tutorial of TB3 ROBOTIS and Gazebo. If you have the required packages as said above and colcon build works correctly, you need not worry about it.

## Static Code Analysis
### cpplint
Run the below command from inside the package folder `TurtleBot3_Walker`
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ./results/cpplint.txt
```
### cppcheck
Run the below command from the project root folder `beginner_tutorials`
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheck.txt
```
## Note:
Please contact me at: okritvik@umd.edu if you have any questions.