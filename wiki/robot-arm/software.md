---
layout: article
date: 2021-06-17
title: Software
author: Raghava Uppuluri
---

## System Overview

TODO

## Quick start

1. [Install ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros)

2. Clone this repo into the `src` folder in your ROS workspace
```
git clone https://github.com/purdue-arc/arc_robot_arm.git
```
3. Download all package dependencies
```
cd path/to/workspace
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```
4. Build + source
```
catkin build && source path/to/catkin_ws/devel/setup.bash
```
5. Launch the robot
```
roslaunch protoarm_bringup robot.launch
```
6. Run some test goal positions (in another terminal window)
```
roscd protoarm_kinematics/src && rosrun protoarm_kinematics test_kinematics
```
Should see the robot move something like this (to the right and back to the left) in Gazebo and MoveIt:
<img src="assets/gifs/ik_demo.gif" alt="GIF of arm in Gazebo" width="600"/>

## High level

TODO

## Mid level

TODO

## Low level

TODO

## Resources

TODO


