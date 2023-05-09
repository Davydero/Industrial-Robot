<p align="center">
  <h2 align="center">APPLICATION OF AN INDUSTRIAL ROBOTIC MANIPULATOR FOR BOXED SNACKS</h2>
  <p align="center">
  Davy Rojas Yana - Universidad Católica Boliviana
  </p>
  <p align="center">
  Based on: Exam project for Fundamental of Robotics
  <br>University of Trento - Prof. <a href="https://webapps.unitn.it/du/it/Persona/PER0002392/Curriculum">Luigi Palopoli</a> and <a href="https://webapps.unitn.it/du/it/Persona/PER0051994/Curriculum">Niculae Sebe</a>
  </p>
</p>
<br>

<img src="[https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/main.png](https://user-images.githubusercontent.com/11477020/237109895-8457fd5c-6ac8-4001-8374-a6df909813fd.PNG)">

## Table of contents
- [Description](#description)
- [Requirement](#requirements)
- [Setup](#setup)
- [Usage](#usage)
- [Owners](#owners)
- [Contributor](#contributor)

### Description
This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a Xbox Kinect cam to detect eleven types of Lego Bricks, and publish its position and angolation. 

The goals of this project are:
- simulate the iteration of a UR5 robot with Lego bricks
- The robotic arm must be able to move a block from position A to B and construct a castle by assembling different bricks

<img src="https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/intro.gif">

### Requirements

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Yolov5` https://github.com/ultralytics/yolov5
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Setup

After installing the libraries needed to run the project. Clone this repo:
```
git clone https://github.com/Davydero/Industrial-Robot
```

Setup the project:
```
cd UR5-Pick-and-Place-Simulation/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
echo "source $PWD/devel/setup.bash" >> $HOME/.bashrc
```

Clone and install [YoloV5](https://github.com/ultralytics/yolov5):
```
cd ~
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip3 install -r requirements.txt
```
### Usage

Launch the world
```
roslaunch levelManager lego_world.launch
```
Choose the level (from 1 to 4):
```
rosrun levelManager levelManager.py -l [level]
```
Start the kinematics process
```
rosrun motion_planning motion_planning.py
```
Start the localization process
```
rosrun vision vision.py -show
```

### Owners

| Name                 | Matricola | Github                               |
|----------------------|-----------|--------------------------------------|
| Davice Cerpelloni    | 213541    | https://github.com/davidecerpelloni  |
| Leonardo Collizzolli | 209316    | https://github.com/leocolliz         |
| Pietro Lechthaler    | 210601    | https://github.com/pietrolechthaler  |
| Stefano Rizzi        | 209684    | https://github.com/StefanoRizzi      |

### Contributor

| Name                 | Github                               |
|----------------------|--------------------------------------|
| Davy Rojas Yana      | https://github.com/Davydero          |
