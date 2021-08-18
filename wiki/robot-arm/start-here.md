---
date: 2021-06-01
title: Start Here
---

## Goal 

Our overarching goal is to explore the robotic manipulation, robot vision, robot control, and reinforcement learning by building a robot arm. In our journey, we plan to publish our progress, tutorials, and understandings.

Right now, we're building a robot arm that can play chess.

## Meet the robots

### Protoarm (stable)

This is protoarm (short for prototype arm), a 5-DOF robot arm adapted slightly from [HowToMechatronics' model](https://www.youtube.com/watch?v=_B3gWd3A_SI) that we built first to understand ROS, MoveIt, and the software stack for robot arms.

<img src="assets/images/protoarm.png" alt="Protoarm CAD" width="400"/>

### Mr. Janktastic (WIP)

## Subsystems 

Hardware
- Arm 
- Gripper
- Mounts + accessories

Software
- High-level
  - Task planner or reinforcement learning policy 
- Mid-level
  - Kinematics + planning, object detection + visual servoing
- Low-level
  - Servo control

### Want to dig deeper? 

Check out the [software docs]({% link wiki/robot-arm/software.md %}) and [GitHub](https://github.com/purdue-arc/arc_robot_arm) for tutorials and a detailed overview of the software behind the robot.

## What have we done?

### May 2021

- Object detection working on chess pieces with 90%+ accuracy using YOLOv5 and usable in ROS
  - Put together a 500+ chess piece dataset for detection

<img src="assets/images/obj_det_may_21.png" alt="Object detection demo" width="400"/>

- Prototype gripper fingers that can pick up chess pieces decently well

{% include googleDrivePlayer.html id="1P8rwWDJa1Yuv88X697RMvEq04j1IgpqW/preview" %}

- Created Gazebo simulation that is controlled by MoveIt pipeline, including a simulated camera, chessboard, and chess pieces

{% include googleDrivePlayer.html id="19FZ7lsqCn6DEChjjdBgsTy0feUANYaVO/preview" %}

### December 2020
- Got an early version of protoarm to stack some boxes using IK and trajectory planning from MoveIt and ROS

{% include googleDrivePlayer.html id="1yms3OuqYp-n4JCt-yBGZg8yEyajXOj_M/preview" %}

- Designed a prototype 6dof arm

<img src="assets/images/6dof_dec_20.png" alt="6DOF arm CAD" width="200"/>

## Quick links 

- [GitHub](https://github.com/purdue-arc/arc_robot_arm)
