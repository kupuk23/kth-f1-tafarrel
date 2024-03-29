# Project Title

An exercise project for KTH F1 Application (Driverless Team)

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)

## Installation


- Install dependencies for exercise 2:
- Install Tktinker (for GUI) : `sudo apt install python3-tk`\
- Install mathplotlib, numpy : `pip3 install mathplotlib numpy`

FOR UBUNTU ONLY :\
- Install Tkinker `sudo apt install python3-tk`
- Install PlotJuggler `sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros`

## Usage

### For Exercise 1
- Create your own workspace 
```
mkdir ~/f1_ws
mkdir ~/f1_ws/src
cd ~/f1_ws/src
```

- Clone my repos to inside of the src folder\
`git clone https://github.com/kupuk23/kth-f1-tafarrel.git`

- Install dependencies, run `rosdep install --from-paths src --ignore-src -r -y`

- Build the colcon
```
cd ~/f1_ws
colcon build
``` 

- Run this in your SEPARATE terminals:

`roscore`\
`rosrun package1 talker.py`\
`rosrun package2 listener.py`

- ***OPTIONAL*** run plotjuggler by using `rosrun plotjuggler plotjuggler` 

<img src="img/plot.png" width="500" height="300">



### For Exercise 2
- go to exercise2 path : `cd ~/f1_ws/src/exercise2`
- run python file : `python3 run main.py`

